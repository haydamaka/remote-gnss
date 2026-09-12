// How long should a straight run have to be? Drive the same trips with
// different window lengths and see what each one gets.
#include "bearing_cal.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define HZ 10
#define DT (1.0f / HZ)
#define DEGR (M_PI / 180.0)

static uint32_t rs = 987654321;
static float uniform(void) { rs ^= rs << 13; rs ^= rs >> 17; rs ^= rs << 5; return (float)(rs >> 8) / 16777216.0f; }
static float gauss(float s) { float u = uniform() + 1e-7f; return s * sqrtf(-2.0f * logf(u)) * cosf(6.2831853f * uniform()); }
static float w360(float d) { d = fmodf(d, 360.0f); return d < 0 ? d + 360 : d; }
static float w180(float d) { d = fmodf(d + 180.0f, 360.0f); if (d < 0) d += 360; return d - 180; }

#define MOUNT 137.0f
#define DECL 8.6f

// A magnetometer with a typical leftover calibration error: 0.34 uT of hard
// iron and 3.7 % of axis asymmetry, which is about 1 degree of each harmonic.
#define HFIELD 19.4f
#define HARD_X 0.34f
#define SOFT_EPS 0.037f

static float measured(float psi) {
    float ideal = (psi - MOUNT) * (float)DEGR;
    float bx = HFIELD * cosf(ideal) + HARD_X;
    float by = HFIELD * sinf(ideal) * (1.0f + SOFT_EPS);
    return w360(atan2f(by, bx) / (float)DEGR);
}

typedef struct { bearing_cal_t *cal; float t, psi, speed; } trip_t;

static void step(trip_t *tr) {
    float c_sigma = tr->speed > 0.1f ? atanf(0.05f / tr->speed) / (float)DEGR : 90.0f;
    bearing_sample_t s = {.t = tr->t, .course_deg = w360(tr->psi + DECL + gauss(c_sigma)),
                          .speed_mps = tr->speed,
                          .heading_deg = measured(tr->psi) + gauss(0.5f),
                          .decl_deg = DECL, .usable = true};
    bearing_cal_feed(tr->cal, &s);
    tr->t += DT;
}
static void straight(trip_t *tr, float h, float sp, float sec) {
    tr->psi = h; tr->speed = sp;
    for (int i = 0; i < (int)(sec * HZ); i++) step(tr);
}
static void turn(trip_t *tr, float to, float rate) {
    float d = w180(to - tr->psi); int n = (int)(fabsf(d) / rate * HZ) + 1;
    for (int i = 0; i < n; i++) { tr->psi = w360(tr->psi + d / n); step(tr); }
}

// How long the straight runs are, and which way they point. A motorway is
// long and runs in two directions; a town is short and goes everywhere.
static float run_length(const char *road) {
    float u = uniform();
    if (!strcmp(road, "town"))     return 6.0f + u * 20.0f;
    if (!strcmp(road, "motorway")) return 60.0f + u * 240.0f;
    return 10.0f + u * 50.0f;
}

static float next_heading(const char *road, float prev) {
    if (!strcmp(road, "motorway")) {
        // Up the bypass and back down it, give or take a few degrees.
        float base = uniform() < 0.5f ? 35.0f : 215.0f;
        return w360(base + gauss(8.0f));
    }
    if (!strcmp(road, "main")) {
        // Four directions, as a grid of main roads.
        return w360(90.0f * (float)(int)(uniform() * 4.0f) + gauss(10.0f));
    }
    return w360(prev + 30.0f + uniform() * 300.0f);
}

static void run(const char *road, float window_s, int minutes) {
    bearing_cfg_t cfg; bearing_cal_defaults(&cfg);
    cfg.window_s = window_s;
    bearing_cal_t cal; bearing_cal_init(&cal, &cfg);
    trip_t tr = {.cal = &cal};
    rs = 987654321;
    float heading = 0.0f;
    while (tr.t < minutes * 60.0f) {
        heading = next_heading(road, heading);
        turn(&tr, heading, 20.0f + uniform() * 20.0f);
        straight(&tr, heading, 8.0f + uniform() * 14.0f, run_length(road));
    }
    bearing_result_t r; bearing_cal_result(&cal, &r);
    // The honest measure: how wrong is the corrected heading, over every
    // direction the vehicle could later point, not just the ones it drove.
    float worst = 0.0f, sum = 0.0f;
    int n = 0;
    for (float psi = 0.0f; psi < 360.0f; psi += 2.0f) {
        float e = fabsf(w180(bearing_cal_correct(&r, measured(psi)) - psi));
        if (e > worst) worst = e;
        sum += e; n++;
    }
    printf("  %-9s T=%4.0fs  %4d windows  %2d/12 sectors  %2d terms  %-7s   A off by %5.2f   "
           "corrected heading: mean %4.2f  worst %5.2f\n",
           road, window_s, r.windows, r.sectors, r.terms, r.have ? "applies" : "refuses",
           fabsf(w180(r.offset_deg - MOUNT)), r.have ? sum / n : 99.0f, r.have ? worst : 99.0f);
}

int main(void) {
    printf("30 minutes of driving. Mounting offset %.0f deg, plus %.2f uT of leftover hard iron\nand %.1f %% of axis asymmetry: about 1 deg of each harmonic to be found.\n\n", MOUNT, HARD_X, SOFT_EPS * 100);
    for (const char **road = (const char *[]){"town", "main", "motorway", NULL}; *road; road++) {
        for (float t = 4; t <= 30; t = t < 8 ? t + 2 : t * 1.5f) run(*road, t, 30);
        printf("\n");
    }
    return 0;
}
