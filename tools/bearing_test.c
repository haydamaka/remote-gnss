// Synthetic trips for bearing_cal.c. The vehicle drives, the module is bolted
// to it at an unknown angle, and the magnetometer has whatever the last
// calibration left behind. The estimator should recover all of it.
//
//   cc -O2 -Wall -Wextra -o bearing_test bearing_test.c bearing_cal.c -lm

#include "bearing_cal.h"

#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#define DEG (M_PI / 180.0)
#define HZ 10
#define DT (1.0f / HZ)

static int failures;

static uint32_t rng_state = 12345;
static float uniform(void)
{
    rng_state ^= rng_state << 13;
    rng_state ^= rng_state >> 17;
    rng_state ^= rng_state << 5;
    return (float)(rng_state >> 8) / 16777216.0f;
}
static float gauss(float sigma)
{
    float u1 = uniform() + 1e-7f, u2 = uniform();
    return sigma * sqrtf(-2.0f * logf(u1)) * cosf(2.0f * (float)M_PI * u2);
}

static float wrap360f(float d)
{
    d = fmodf(d, 360.0f);
    return d < 0.0f ? d + 360.0f : d;
}
static float wrap180f(float d)
{
    d = fmodf(d + 180.0f, 360.0f);
    if (d < 0.0f) d += 360.0f;
    return d - 180.0f;
}

// ---- the thing being simulated ----

typedef struct {
    float mount_deg;   // how the module is bolted on: the answer
    float hard_x_ut;   // what the magnetometer calibration left behind
    float hard_y_ut;
    float soft_eps;    // fractional scale error between the two horizontal axes
    float h_field_ut;
    float decl_deg;
    float course_sigma_v;  // GNSS velocity noise, m/s
    float heading_sigma;   // magnetometer heading noise, degrees
} world_t;

// What the magnetometer reads when the vehicle travels at magnetic azimuth psi.
static float measured_heading(const world_t *w, float psi_deg)
{
    float ideal = (psi_deg - w->mount_deg) * (float)DEG;
    float bx = w->h_field_ut * cosf(ideal) + w->hard_x_ut;
    float by = w->h_field_ut * sinf(ideal) * (1.0f + w->soft_eps) + w->hard_y_ut;
    return wrap360f(atan2f(by, bx) / (float)DEG);
}

// What the estimator should end up with, given the world.
static float predicted_once_round(const world_t *w)
{
    return hypotf(w->hard_x_ut, w->hard_y_ut) / w->h_field_ut / (float)DEG;
}
static float predicted_twice_round(const world_t *w)
{
    return fabsf(w->soft_eps) / 2.0f / (float)DEG;
}

// ---- driving ----

typedef struct {
    bearing_cal_t *cal;
    const world_t *w;
    float t;
    float psi;        // current magnetic azimuth of travel
    float speed;
    bool  usable;
    float module_yaw; // the module turning on its own: a loose mount
} trip_t;

static void step(trip_t *tr)
{
    const world_t *w = tr->w;
    float h = measured_heading(w, tr->psi) + tr->module_yaw + gauss(w->heading_sigma);
    // Course error grows as the speed falls: it comes from Doppler velocity.
    float c_sigma = tr->speed > 0.1f ? atanf(w->course_sigma_v / tr->speed) / (float)DEG : 90.0f;
    float course = wrap360f(tr->psi + w->decl_deg + gauss(c_sigma));

    bearing_sample_t s = {
        .t = tr->t,
        .course_deg = course,
        .speed_mps = tr->speed,
        .heading_deg = wrap360f(h),
        .decl_deg = w->decl_deg,
        .usable = tr->usable,
    };
    bearing_cal_feed(tr->cal, &s);
    tr->t += DT;
}

static void drive_straight(trip_t *tr, float heading, float speed, float seconds)
{
    tr->psi = heading;
    tr->speed = speed;
    for (int i = 0; i < (int)(seconds * HZ); i++) {
        step(tr);
    }
}

static void drive_turn(trip_t *tr, float to_heading, float rate_dps)
{
    float d = wrap180f(to_heading - tr->psi);
    int n = (int)(fabsf(d) / rate_dps * HZ) + 1;
    float per = d / (float)n;
    for (int i = 0; i < n; i++) {
        tr->psi = wrap360f(tr->psi + per);
        step(tr);
    }
}

// ---- checks ----

static void check(const char *what, float got, float want, float tol)
{
    bool ok = fabsf(wrap180f(got - want)) <= tol;
    printf("  %-42s %8.2f  (want %7.2f +- %.2f)  %s\n", what, got, want, tol, ok ? "ok" : "FAIL");
    if (!ok) {
        failures++;
    }
}

static void check_true(const char *what, bool got, bool want)
{
    printf("  %-42s %8s  (want %7s)          %s\n", what, got ? "yes" : "no", want ? "yes" : "no",
           got == want ? "ok" : "FAIL");
    if (got != want) {
        failures++;
    }
}

static void report_rejects(const bearing_result_t *r)
{
    printf("    accepted %d window(s), %d sector(s)", r->windows, r->sectors);
    for (int i = 1; i < BEARING_REASONS; i++) {
        if (r->rejected[i]) {
            printf(", %s x%u", bearing_reject_name((bearing_reject_t)i), (unsigned)r->rejected[i]);
        }
    }
    printf("\n");
}

// A trip round a town: straight runs in every direction, turns between them.
static void town_trip(trip_t *tr, int runs, float run_s, float speed)
{
    for (int i = 0; i < runs; i++) {
        float heading = wrap360f((float)i * 360.0f / (float)runs + uniform() * 20.0f);
        drive_turn(tr, heading, 25.0f);
        drive_straight(tr, heading, speed, run_s);
    }
}

// ---- the tests ----

static void test_mounting_offset(void)
{
    printf("\nA plain mounting offset, a perfect magnetometer\n");
    world_t w = {.mount_deg = 137.0f, .h_field_ut = 19.4f, .decl_deg = 8.6f,
                 .course_sigma_v = 0.05f, .heading_sigma = 0.5f};
    bearing_cfg_t cfg;
    bearing_cal_defaults(&cfg);
    cfg.h_field_ut = w.h_field_ut;
    bearing_cal_t cal;
    bearing_cal_init(&cal, &cfg);
    trip_t tr = {.cal = &cal, .w = &w, .usable = true};

    town_trip(&tr, 12, 30.0f, 12.0f);

    bearing_result_t r;
    bearing_cal_result(&cal, &r);
    report_rejects(&r);
    check_true("applies it", r.have, true);
    check("offset", r.offset_deg, w.mount_deg, 1.0f);
    check("once-round (should be nothing)", r.once_round_deg, 0.0f, 0.6f);
    check("twice-round (should be nothing)", r.twice_round_deg, 0.0f, 0.6f);
}

static void test_deviation_is_measured(void)
{
    printf("\nA calibration that left hard and soft iron behind\n");
    // 0.34 uT of residual offset is 1 degree of once-round at this field;
    // 3.7 %% of axis asymmetry is about 1 degree of twice-round.
    world_t w = {.mount_deg = 42.0f, .hard_x_ut = 0.34f, .hard_y_ut = 0.0f, .soft_eps = 0.037f,
                 .h_field_ut = 19.4f, .decl_deg = 8.6f, .course_sigma_v = 0.05f, .heading_sigma = 0.3f};
    bearing_cfg_t cfg;
    bearing_cal_defaults(&cfg);
    cfg.h_field_ut = w.h_field_ut;
    bearing_cal_t cal;
    bearing_cal_init(&cal, &cfg);
    trip_t tr = {.cal = &cal, .w = &w, .usable = true};

    town_trip(&tr, 12, 40.0f, 15.0f);
    town_trip(&tr, 12, 40.0f, 15.0f);

    bearing_result_t r;
    bearing_cal_result(&cal, &r);
    report_rejects(&r);
    printf("    terms fitted: %d\n", r.terms);
    check_true("applies it", r.have, true);
    check("offset", r.offset_deg, w.mount_deg, 1.0f);
    check("once-round amplitude", r.once_round_deg, predicted_once_round(&w), 0.4f);
    check("twice-round amplitude", r.twice_round_deg, predicted_twice_round(&w), 0.4f);
    check("residual hard iron, uT", r.residual_hard_ut, hypotf(w.hard_x_ut, w.hard_y_ut), 0.15f);
    check("axis asymmetry, %", r.soft_asym_pct, w.soft_eps * 100.0f, 1.5f);

    // The corrected heading should now be right in every direction.
    float worst = 0.0f;
    for (float psi = 0.0f; psi < 360.0f; psi += 5.0f) {
        float h = measured_heading(&w, psi);
        float e = fabsf(wrap180f(bearing_cal_correct(&r, h) - psi));
        if (e > worst) {
            worst = e;
        }
    }
    check("worst corrected heading error", worst, 0.0f, 0.8f);
}

static void test_turning_is_rejected(void)
{
    printf("\nA trip that never stops turning\n");
    world_t w = {.mount_deg = 90.0f, .h_field_ut = 19.4f, .decl_deg = 8.6f,
                 .course_sigma_v = 0.05f, .heading_sigma = 0.5f};
    bearing_cfg_t cfg;
    bearing_cal_defaults(&cfg);
    bearing_cal_t cal;
    bearing_cal_init(&cal, &cfg);
    trip_t tr = {.cal = &cal, .w = &w, .usable = true};

    tr.speed = 12.0f;
    for (int i = 0; i < 40; i++) {
        drive_turn(&tr, wrap360f(tr.psi + 120.0f), 8.0f);  // a long, steady curve
    }

    bearing_result_t r;
    bearing_cal_result(&cal, &r);
    report_rejects(&r);
    check_true("refuses to apply anything", r.have, false);
    check("windows accepted", (float)r.windows, 0.0f, 0.5f);
}

static void test_one_direction_only(void)
{
    printf("\nAn hour up and down the same road\n");
    world_t w = {.mount_deg = 200.0f, .h_field_ut = 19.4f, .decl_deg = 8.6f,
                 .course_sigma_v = 0.05f, .heading_sigma = 0.5f};
    bearing_cfg_t cfg;
    bearing_cal_defaults(&cfg);
    bearing_cal_t cal;
    bearing_cal_init(&cal, &cfg);
    trip_t tr = {.cal = &cal, .w = &w, .usable = true};

    for (int i = 0; i < 20; i++) {
        drive_turn(&tr, 40.0f, 25.0f);
        drive_straight(&tr, 40.0f, 14.0f, 60.0f);
        drive_turn(&tr, 220.0f, 25.0f);
        drive_straight(&tr, 220.0f, 14.0f, 60.0f);
    }

    bearing_result_t r;
    bearing_cal_result(&cal, &r);
    report_rejects(&r);
    check_true("refuses to apply it", r.have, false);
    printf("    (it has %d windows and still says no, which is the point)\n", r.windows);
}

static void test_reversing_and_tunnels(void)
{
    printf("\nCar parks, tunnels, and a module loose in the cup holder\n");
    world_t w = {.mount_deg = 310.0f, .h_field_ut = 19.4f, .decl_deg = 8.6f,
                 .course_sigma_v = 0.05f, .heading_sigma = 0.5f};
    bearing_cfg_t cfg;
    bearing_cal_defaults(&cfg);
    bearing_cal_t cal;
    bearing_cal_init(&cal, &cfg);
    trip_t tr = {.cal = &cal, .w = &w, .usable = true};

    for (int i = 0; i < 12; i++) {
        float heading = wrap360f((float)i * 30.0f + 7.0f);
        drive_turn(&tr, heading, 25.0f);
        drive_straight(&tr, heading, 13.0f, 30.0f);

        // Reversing: the module still points forwards while travel is backwards.
        tr.module_yaw = 180.0f;
        drive_straight(&tr, wrap360f(heading + 180.0f), 4.0f, 20.0f);
        tr.module_yaw = 0.0f;

        // A tunnel: fixes stop being usable.
        tr.usable = false;
        drive_straight(&tr, heading, 13.0f, 15.0f);
        tr.usable = true;

        // Stopped at lights.
        drive_straight(&tr, heading, 0.0f, 20.0f);

        // The module slides about while the vehicle goes straight.
        for (int k = 0; k < 15 * HZ; k++) {
            tr.module_yaw += 0.8f * DT * HZ * 0.1f;
            step(&tr);
        }
        tr.module_yaw = 0.0f;
    }

    bearing_result_t r;
    bearing_cal_result(&cal, &r);
    report_rejects(&r);
    check_true("still applies it", r.have, true);
    check("offset, undisturbed", r.offset_deg, w.mount_deg, 1.5f);
}

static void test_remount(void)
{
    printf("\nThe module is unscrewed and put back 90 degrees round\n");
    world_t w = {.mount_deg = 0.0f, .h_field_ut = 19.4f, .decl_deg = 8.6f,
                 .course_sigma_v = 0.05f, .heading_sigma = 0.5f};
    bearing_cfg_t cfg;
    bearing_cal_defaults(&cfg);
    bearing_cal_t cal;
    bearing_cal_init(&cal, &cfg);
    trip_t tr = {.cal = &cal, .w = &w, .usable = true};

    town_trip(&tr, 12, 40.0f, 14.0f);
    bearing_result_t r;
    bearing_cal_result(&cal, &r);
    check("before", r.offset_deg, 0.0f, 1.5f);

    w.mount_deg = 90.0f;
    for (int i = 0; i < 6; i++) {
        town_trip(&tr, 12, 40.0f, 14.0f);
    }
    bearing_cal_result(&cal, &r);
    report_rejects(&r);
    check("after, having faded the old value out", r.offset_deg, 90.0f, 3.0f);
}

static void report_quality(const bearing_result_t *r)
{
    const bearing_quality_t *q = &r->quality;
    printf("    out-of-sample residual: rms %.2f deg, bias %+.2f, worst %.2f, over %u windows\n",
           q->resid_rms_deg, q->resid_bias_deg, q->resid_max_deg, (unsigned)q->resid_n);
    printf("    split halves disagree by %.2f deg (%d and %d sectors)\n",
           q->split_deg, q->split_sectors[0], q->split_sectors[1]);
    printf("    drift %+.2f deg/hour over %.2f h, moved %.2f deg at its widest\n",
           q->drift_deg_per_hour, q->hours, q->range_deg);
    printf("    accepted %.0f %% of %u windows\n", q->accept_pct, (unsigned)q->windows_seen);
}

// Something slowly turning the module: a bracket working loose, or a magnet
// creeping. The quality measure is supposed to see this even though every
// individual window looks perfectly good.
static void test_quality_catches_drift(void)
{
    printf("\nA mounting that creeps round by 6 degrees an hour\n");
    world_t w = {.mount_deg = 100.0f, .h_field_ut = 19.4f, .decl_deg = 8.6f,
                 .course_sigma_v = 0.05f, .heading_sigma = 0.5f};
    bearing_cfg_t cfg;
    bearing_cal_defaults(&cfg);
    cfg.weight_cap = 60.0f;  // a shorter memory, or it cannot follow anything
    bearing_cal_t cal;
    bearing_cal_init(&cal, &cfg);
    trip_t tr = {.cal = &cal, .w = &w, .usable = true};

    float start = 100.0f;
    for (int i = 0; i < 60; i++) {
        w.mount_deg = wrap360f(start + 6.0f * (tr.t / 3600.0f));
        town_trip(&tr, 12, 30.0f, 14.0f);
    }
    bearing_result_t r;
    bearing_cal_result(&cal, &r);
    report_quality(&r);
    printf("    (the mounting really moved %.2f deg over %.2f h)\n",
           6.0f * tr.t / 3600.0f, tr.t / 3600.0f);
    check("drift measured, deg/hour", r.quality.drift_deg_per_hour, 6.0f, 2.0f);
}

static void test_quality_when_steady(void)
{
    printf("\nThe same drive with nothing moving: the quality numbers to beat\n");
    world_t w = {.mount_deg = 100.0f, .hard_x_ut = 0.34f, .soft_eps = 0.037f,
                 .h_field_ut = 19.4f, .decl_deg = 8.6f,
                 .course_sigma_v = 0.05f, .heading_sigma = 0.5f};
    bearing_cfg_t cfg;
    bearing_cal_defaults(&cfg);
    cfg.weight_cap = 60.0f;
    bearing_cal_t cal;
    bearing_cal_init(&cal, &cfg);
    trip_t tr = {.cal = &cal, .w = &w, .usable = true};

    for (int i = 0; i < 60; i++) {
        town_trip(&tr, 12, 30.0f, 14.0f);
    }
    bearing_result_t r;
    bearing_cal_result(&cal, &r);
    report_quality(&r);
    check("drift, should be nothing", r.quality.drift_deg_per_hour, 0.0f, 1.0f);
    check("offset still right", r.offset_deg, w.mount_deg, 1.0f);
}

int main(void)
{
    test_mounting_offset();
    test_deviation_is_measured();
    test_turning_is_rejected();
    test_one_direction_only();
    test_reversing_and_tunnels();
    test_remount();
    test_quality_when_steady();
    test_quality_catches_drift();
    printf("\n%s (%d failure%s)\n", failures ? "FAILED" : "all passed", failures,
           failures == 1 ? "" : "s");
    return failures ? 1 : 0;
}
