#include "mag_cal.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "nvs.h"

#include "mag_fit.h"
#include "magnetometer.h"

_Static_assert(MAG_CAL_MAX_POINTS <= MAG_FIT_MAX_POINTS, "the fit takes fewer readings than the buffer holds");

#define NVS_NAMESPACE "magcal"
// Version 2's "rough" byte became the quality, and its values still mean the
// same thing (0 good, 1 rough), so stored calibrations survive the change.
#define NVS_VERSION 2
#define TASK_PERIOD_MS 100
#define REFIT_AFTER 20                // new readings before fitting again
#define FIT_EVERY_US (3 * 1000000LL)  // and no sooner: a fit costs a good fraction of a second
#define MIN_FIT_POINTS 20
#define RECENTRE_UT 2.0f              // centre shift that triggers re-binning
#define COVERAGE_DONE (CONFIG_RGNSS_CAL_COVERAGE_PCT / 100.0f)
#define CELL_MIN 2                    // readings for a cell to count as covered
#define CAP_MIN 5                     // readings within 45° of each of ±X, ±Y, ±Z
#define CAP_COS 0.70710678f
#define SESSION_TIMEOUT_US (5 * 60 * 1000000LL)
#define SESSION_MIN_NEW 100           // fresh readings before a continued session may finish
#define SAVE_EVERY_US (30 * 1000000LL)
#define SATURATION_UT 1000.0f

// CALIBRATION.md §2, with the interim fit-error limit of §9: without a noise
// estimate yet, 1.5 % is out of reach on real data.
#define RADIUS_MIN 20.0f
#define RADIUS_MAX 70.0f
#define ACCEPT_RMS (CONFIG_RGNSS_CAL_GOOD_PERMILLE / 1000.0f)
// A rough calibration: the readings it kept sit on the ellipsoid, and it kept
// enough of them. Trimming barely moves the parameters (CALIBRATION.md §9), so
// this is still a usable calibration — it is only labelled as rough.
#define ROUGH_RMS (CONFIG_RGNSS_CAL_ROUGH_PERMILLE / 1000.0f)
#define ROUGH_INLIERS 0.7f
#define ACCEPT_WOBBLE 0.02f
#define ACCEPT_SCATTER 0.2f
#define ACCEPT_INLIERS 0.9f
#define EIG_MIN 0.67f
#define EIG_MAX 1.5f
#define STABLE_FITS 3
#define STABLE_OFFSET_UT 0.5f
#define STABLE_MATRIX 0.01f
#define STABLE_RADIUS 0.005f
#define TIDY_MAX_SHIFT_UT 1.0f  // a tidy-up that moves the centre more than this is refused
#define NEWCOMER_BIAS 1.5f      // how much worse than the stored worst a new reading must be to be refused

static const char *TAG = "mag_cal";

typedef struct {
    float v[3];      // µT
    uint32_t order;  // arrival order, for oldest-first replacement
    uint8_t cell;
} point_t;

typedef struct {
    uint8_t version;
    uint8_t quality;
    float offset[3];
    float matrix[9];
    float radius;
    float fit_rms;
    uint16_t points;
} stored_params_t;

// Everything below is guarded by lock.
static SemaphoreHandle_t lock;
static mag_cal_state_t state = MAG_CAL_IDLE;
static char reason_buf[64];    // empty unless the last session failed
static char blocking_buf[64];  // which criterion is holding a running session up
static char message_buf[200];  // how the last session ended, in plain words
static bool fit_tidied;        // the latest fit was refitted on its inliers
static point_t points[MAG_CAL_MAX_POINTS];
static uint16_t npoints;
static uint32_t arrivals;
static float centre[3];
static float coverage;
static float progress;
static uint8_t missing = 0x3F;
static int64_t session_start;
static int64_t last_save;
static int64_t last_fit;
static uint32_t session_id;
static int session_new;
static uint32_t session_refused;  // readings turned away for disagreeing with their own direction
static mag_cal_params_t params;
static mag_cal_params_t params_before_save;
static mag_fit_t fit;
static bool have_fit;
static int stable_count;
static uint32_t fit_ms;
// The working memory a fit needs: about 17 kB that only matters while a session
// is running. It is the one dynamic allocation in the firmware, and it is
// bounded at a single block - scratch_acquire() hands back what already exists
// rather than taking more, so the total can never exceed one no matter which
// path runs. Release is driven by the invariant below rather than paired with
// each of the four ways a session can end, so a fifth cannot leak.
typedef struct {
    mag_fit_scratch_t fit;
    float snapshot[MAG_CAL_MAX_POINTS][3];
    float kept[MAG_CAL_MAX_POINTS][3];
    bool  keep[MAG_CAL_MAX_POINTS];
} mag_scratch_t;

static mag_scratch_t *scratch;
static uint32_t heap_at_acquire;

// Staging for the NVS blob, 0.1 µT units. One copy, shared by save and load.
static int16_t packed[MAG_CAL_MAX_POINTS][3];

static bool fitting;        // a fit is using the scratch right now, off the lock
static bool loading;        // an upload is parsing into it, off the lock
static bool points_dirty;
static uint32_t points_seq;  // bumped whenever the buffer changes, so the page can tell
static bool params_dirty;
static bool started;
static bool fit_once;  // fit even without a session: readings were just loaded
static TaskHandle_t task;

// Call with the lock held.
static mag_scratch_t *scratch_acquire(void)
{
    if (scratch) {
        return scratch;  // never a second block, whatever called us
    }
    heap_at_acquire = esp_get_free_heap_size();
    scratch = malloc(sizeof *scratch);
    if (!scratch) {
        ESP_LOGW(TAG, "no room for %u bytes of fit scratch; %u bytes of heap free",
                 (unsigned)sizeof *scratch, (unsigned)heap_at_acquire);
    }
    return scratch;
}

// Call with the lock held. Reports the heap back, so a leak is a number in the
// log rather than something to be reasoned about.
static void scratch_release(void)
{
    if (!scratch) {
        return;
    }
    free(scratch);
    scratch = NULL;
    uint32_t now = esp_get_free_heap_size();
    if (now + 512 < heap_at_acquire) {
        ESP_LOGW(TAG, "heap did not come back: %u free, %u before the session",
                 (unsigned)now, (unsigned)heap_at_acquire);
    }
}

// The readings changed: they need saving, and anyone drawing them needs to
// know. Every site that alters the buffer goes through here.
static void points_changed(void)
{
    points_dirty = true;
    points_seq++;
}

static float dist(const float a[3], const float b[3])
{
    float d[3] = {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
    return sqrtf(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
}

// Direction of v about c; false if v sits on it.
static bool direction_from(const float c[3], const float v[3], float u[3])
{
    float len = dist(v, c);
    if (len < 1e-3f) {
        return false;
    }
    for (int i = 0; i < 3; i++) {
        u[i] = (v[i] - c[i]) / len;
    }
    return true;
}

static bool direction(const float v[3], float u[3])
{
    return direction_from(centre, v, u);
}

static int oldest_in(uint8_t cell)
{
    int best = -1;
    for (int i = 0; i < npoints; i++) {
        if (points[i].cell == cell && (best < 0 || points[i].order < points[best].order)) {
            best = i;
        }
    }
    return best;
}

// Which reading in a full cell makes way. Once there is a fit worth trusting,
// the one that disagrees with it most, so a long session cleans itself instead
// of stalling; otherwise the oldest. Guarded, because evicting whatever
// disagrees would let the fit confirm its own mistakes: it needs a plausible
// fit that most readings already agree with, and a newcomer only loses its
// place when it is clearly the worst, so a changed environment still gets in.
static int victim_in(uint8_t cell, const float m[3], bool *refuse_new)
{
    *refuse_new = false;
    int oldest = oldest_in(cell);
    if (!have_fit || !mag_fit_plausible_for_binning(&fit) || fit.inliers * 10 < fit.points * 7) {
        return oldest;
    }
    int worst = -1;
    float worst_res = 0;
    for (int i = 0; i < npoints; i++) {
        if (points[i].cell != cell) {
            continue;
        }
        float r = fabsf(mag_fit_residual(&fit, points[i].v));
        if (worst < 0 || r > worst_res) {
            worst = i;
            worst_res = r;
        }
    }
    if (worst < 0) {
        return oldest;
    }
    *refuse_new = fabsf(mag_fit_residual(&fit, m)) > worst_res * NEWCOMER_BIAS;
    return worst;
}

// Keeps up to MAG_CAL_PER_CELL readings per cell.
static void add_reading(const float m[3])
{
    float u[3];
    if (!direction(m, u)) {
        return;
    }
    point_t p = {.v = {m[0], m[1], m[2]}, .order = arrivals++, .cell = mag_fit_cell(u)};
    int count = 0;
    for (int i = 0; i < npoints; i++) {
        count += points[i].cell == p.cell;
    }
    bool refuse_new;
    int victim = victim_in(p.cell, m, &refuse_new);
    if ((count < MAG_CAL_PER_CELL || victim < 0) && npoints < MAG_CAL_MAX_POINTS) {
        points[npoints++] = p;
        points_changed();
    } else if (refuse_new) {
        session_refused++;  // it disagrees with its own direction more than anything stored there
    } else if (victim >= 0) {
        points[victim] = p;
        points_changed();
    }
}

// Re-assigns cells after the centre moved, dropping the oldest readings of cells
// that end up over capacity.
static void rebin(void)
{
    uint16_t count[MAG_CAL_CELLS] = {0};  // a single cell can hold every reading
    for (int i = 0; i < npoints; i++) {
        float u[3];
        points[i].cell = direction(points[i].v, u) ? mag_fit_cell(u) : 0;
        count[points[i].cell]++;
    }
    for (int c = 0; c < MAG_CAL_CELLS; c++) {
        for (; count[c] > MAG_CAL_PER_CELL; count[c]--) {
            int victim = oldest_in(c);  // before --npoints: the order isn't defined within one expression
            points[victim] = points[--npoints];
        }
    }
}

// Whether re-binning around c would keep at least half the readings.
static bool rebin_keeps_most(const float c[3])
{
    uint16_t count[MAG_CAL_CELLS] = {0};
    for (int i = 0; i < npoints; i++) {
        float u[3];
        count[direction_from(c, points[i].v, u) ? mag_fit_cell(u) : 0]++;
    }
    int evicted = 0;
    for (int k = 0; k < MAG_CAL_CELLS; k++) {
        evicted += count[k] > MAG_CAL_PER_CELL ? count[k] - MAG_CAL_PER_CELL : 0;
    }
    return evicted <= npoints / 2;
}

// The middle of the readings' range, until a fit gives something better.
static void range_centre(float c[3])
{
    float lo[3] = {INFINITY, INFINITY, INFINITY};
    float hi[3] = {-INFINITY, -INFINITY, -INFINITY};
    for (int i = 0; i < npoints; i++) {
        for (int k = 0; k < 3; k++) {
            lo[k] = fminf(lo[k], points[i].v[k]);
            hi[k] = fmaxf(hi[k], points[i].v[k]);
        }
    }
    for (int k = 0; k < 3; k++) {
        c[k] = npoints ? (lo[k] + hi[k]) / 2 : centre[k];
    }
}

static void recentre_to(const float c[3])
{
    if (dist(c, centre) > RECENTRE_UT && rebin_keeps_most(c)) {
        memcpy(centre, c, sizeof(centre));
        rebin();
        points_changed();  // re-binning drops readings that no longer fit a cell
    }
}

static void update_progress(void)
{
    uint16_t count[MAG_CAL_CELLS] = {0};
    int caps[6] = {0};
    for (int i = 0; i < npoints; i++) {
        float u[3];
        count[points[i].cell]++;
        if (direction(points[i].v, u)) {
            for (int k = 0; k < 6; k++) {
                caps[k] += u[k / 2] * (k % 2 ? -1.0f : 1.0f) > CAP_COS;
            }
        }
    }
    int covered = 0;
    for (int c = 0; c < MAG_CAL_CELLS; c++) {
        covered += count[c] >= CELL_MIN;
    }
    int caps_ok = 0;
    missing = 0;
    for (int k = 0; k < 6; k++) {
        if (caps[k] >= CAP_MIN) {
            caps_ok++;
        } else {
            missing |= 1 << k;
        }
    }
    coverage = covered / (float)MAG_CAL_CELLS;
    // Reaches 1 when coverage and the caps are done. Completing also needs the
    // fit to meet CALIBRATION.md §2 and SESSION_MIN_NEW fresh readings.
    progress = 0.85f * fminf(1.0f, coverage / COVERAGE_DONE) + 0.15f * caps_ok / 6.0f;
}

// Which criterion of §2 is not met yet, or NULL when the fit may be accepted.
// The text lands in the failure message, so it says the value and the limit.
// With rough set, the error and inlier limits are the looser ROUGH_* ones.
static const char *unmet_at(bool rough, char *buf, size_t size)
{
    float rms_limit = rough ? ROUGH_RMS : ACCEPT_RMS;
    float inlier_limit = rough ? ROUGH_INLIERS : ACCEPT_INLIERS;
    if (!have_fit) {
        return "no fit yet";
    }
    if (coverage < COVERAGE_DONE || missing) {
        snprintf(buf, size, "coverage %.0f %% (target %.0f %%)", coverage * 100, COVERAGE_DONE * 100);
        return buf;
    }
    if (fit.radius < RADIUS_MIN || fit.radius > RADIUS_MAX) {
        snprintf(buf, size, "field strength %.0f µT (want %.0f-%.0f)", fit.radius, (double)RADIUS_MIN,
                 (double)RADIUS_MAX);
        return buf;
    }
    if (fit.eig_min < EIG_MIN || fit.eig_max > EIG_MAX) {
        snprintf(buf, size, "shape %.2f-%.2f (want %.2f-%.2f)", fit.eig_min, fit.eig_max, (double)EIG_MIN,
                 (double)EIG_MAX);
        return buf;
    }
    if (fit.rms > rms_limit) {
        snprintf(buf, size, "fit error %.1f %% (limit %.1f %%)", fit.rms * 100, rms_limit * 100);
        return buf;
    }
    if (fit.wobble > ACCEPT_WOBBLE) {
        snprintf(buf, size, "wobble %.1f %% (limit %.1f %%)", fit.wobble * 100, (double)ACCEPT_WOBBLE * 100);
        return buf;
    }
    if (fit.scatter < ACCEPT_SCATTER) {
        snprintf(buf, size, "readings too flat (spread %.2f, want %.2f)", fit.scatter, (double)ACCEPT_SCATTER);
        return buf;
    }
    if (fit.inliers < fit.points * inlier_limit) {
        snprintf(buf, size, "only %u of %u readings fit", fit.inliers, fit.points);
        return buf;
    }
    if (stable_count < STABLE_FITS - 1) {
        return "still settling";
    }
    return NULL;
}

static const char *unmet(char *buf, size_t size)
{
    return unmet_at(false, buf, size);
}

static const char *quality_word(uint8_t quality)
{
    return quality == MAG_CAL_GOOD ? "good" : quality == MAG_CAL_ROUGH ? "rough" : "poor";
}

// Where the latest fit stands against the two limits.
static mag_cal_quality_t quality_of(void)
{
    char scratch[sizeof(reason_buf)];
    if (!unmet_at(false, scratch, sizeof(scratch))) {
        return MAG_CAL_GOOD;
    }
    if (!unmet_at(true, scratch, sizeof(scratch))) {
        return MAG_CAL_ROUGH;
    }
    return MAG_CAL_POOR;
}

// Worth keeping at all: the numbers are physical and enough of the sphere was
// seen for them to mean something. A poor calibration still beats none.
static bool worth_saving(void)
{
    return have_fit && fit.radius >= RADIUS_MIN && fit.radius <= RADIUS_MAX && fit.eig_min >= EIG_MIN &&
           fit.eig_max <= EIG_MAX && fit.cells >= 50 && coverage >= 0.5f;
}

static bool settled(const mag_fit_t *a, const mag_fit_t *b)
{
    if (fabsf(a->radius - b->radius) > STABLE_RADIUS * b->radius) {
        return false;
    }
    for (int i = 0; i < 3; i++) {
        if (fabsf(a->offset[i] - b->offset[i]) > STABLE_OFFSET_UT) {
            return false;
        }
    }
    for (int i = 0; i < 9; i++) {
        if (fabsf(a->matrix[i] - b->matrix[i]) > STABLE_MATRIX) {
            return false;
        }
    }
    return true;
}

// Packs the readings oldest first, so a restored buffer keeps its replacement
// order. With the lock held.
static uint16_t pack_points(int16_t (*packed)[3])
{
    static uint16_t by_age[MAG_CAL_MAX_POINTS];
    for (int i = 0; i < npoints; i++) {
        by_age[i] = i;
    }
    for (int i = 1; i < npoints; i++) {  // insertion sort by arrival order
        uint16_t v = by_age[i];
        int j = i - 1;
        for (; j >= 0 && points[by_age[j]].order > points[v].order; j--) {
            by_age[j + 1] = by_age[j];
        }
        by_age[j + 1] = v;
    }
    for (int i = 0; i < npoints; i++) {
        for (int k = 0; k < 3; k++) {
            packed[i][k] = (int16_t)lroundf(points[by_age[i]].v[k] * 10.0f);
        }
    }
    return npoints;
}

static void fail(const char *text)
{
    strlcpy(reason_buf, text, sizeof(reason_buf));
    snprintf(message_buf, sizeof(message_buf), "Nothing usable to save: %s. %s", text,
             params.valid ? "The previous calibration stays in use." : "The compass is using raw readings.");
    state = MAG_CAL_FAILED;
    points_changed();  // keep the progress for the next session
    ESP_LOGW(TAG, "calibration failed: %s", reason_buf);
}

// Writes what's due to NVS. Called from the task without the lock: a flash
// commit takes tens of milliseconds and would stall every HTTP request.
static void flush_nvs(void)
{
    // packed[] is shared with the loader: saving runs on the calibration task,
    // loading only at start-up, so the two never overlap.
    stored_params_t sp = {.version = NVS_VERSION, .quality = params.quality};
    uint16_t n;
    bool write_params;

    xSemaphoreTake(lock, portMAX_DELAY);
    if (!points_dirty && !params_dirty) {
        xSemaphoreGive(lock);
        return;
    }
    n = pack_points(packed);
    write_params = params_dirty;
    sp.radius = params.radius;
    sp.fit_rms = params.fit_rms;
    sp.points = params.points;
    memcpy(sp.offset, params.offset, sizeof(sp.offset));
    memcpy(sp.matrix, params.matrix, sizeof(sp.matrix));
    points_dirty = params_dirty = false;  // restored below if the write fails
    last_save = esp_timer_get_time();
    xSemaphoreGive(lock);

    nvs_handle_t nvs;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs);
    if (err == ESP_OK) {
        // Readings first, parameters last: the parameter commit is the point of
        // no return, so it must not succeed on its own.
        err = n ? nvs_set_blob(nvs, "points", packed, n * sizeof(packed[0])) : nvs_erase_key(nvs, "points");
        if (err == ESP_ERR_NVS_NOT_FOUND) {
            err = ESP_OK;  // nothing to erase
        }
        if (err == ESP_OK && write_params) {
            err = nvs_set_blob(nvs, "params", &sp, sizeof(sp));
        }
        if (err == ESP_OK) {
            err = nvs_commit(nvs);
        }
        nvs_close(nvs);
    }
    if (err == ESP_OK) {
        return;
    }
    ESP_LOGE(TAG, "save: %s", esp_err_to_name(err));
    xSemaphoreTake(lock, portMAX_DELAY);
    points_dirty = true;  // it never reached flash, so try again next time round
    if (write_params) {   // the calibration isn't stored, so don't keep using it
        params_dirty = true;
        params = params_before_save;
        fail("Could not save the calibration");
    }
    xSemaphoreGive(lock);
}

static void load(void)
{
    nvs_handle_t nvs;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs) != ESP_OK) {
        ESP_LOGI(TAG, "nothing saved");
        return;
    }
    stored_params_t sp;
    size_t len = sizeof(sp);
    esp_err_t err = nvs_get_blob(nvs, "params", &sp, &len);
    if (err == ESP_OK && len == sizeof(sp) && sp.version == NVS_VERSION) {
        params.valid = true;
        params.quality = sp.quality;
        memcpy(params.offset, sp.offset, sizeof(params.offset));
        memcpy(params.matrix, sp.matrix, sizeof(params.matrix));
        params.radius = sp.radius;
        params.fit_rms = sp.fit_rms;
        params.points = sp.points;
        memcpy(centre, sp.offset, sizeof(centre));
        ESP_LOGI(TAG, "calibration (%s): offset %.1f %.1f %.1f µT, radius %.1f µT, fit %.1f %%",
                 quality_word(params.quality), params.offset[0], params.offset[1], params.offset[2],
                 params.radius, params.fit_rms * 100);
    } else if (err == ESP_OK) {
        ESP_LOGW(TAG, "saved calibration from another version; ignored");
    }
    size_t points_len = sizeof(packed);
    if (nvs_get_blob(nvs, "points", packed, &points_len) == ESP_OK) {
        npoints = points_len / sizeof(packed[0]);
        for (int i = 0; i < npoints; i++) {
            for (int k = 0; k < 3; k++) {
                points[i].v[k] = packed[i][k] / 10.0f;
            }
            points[i].order = i;
        }
        arrivals = npoints;
    }
    nvs_close(nvs);

    if (npoints) {
        if (!params.valid) {
            range_centre(centre);
        }
        rebin();
        update_progress();
        ESP_LOGI(TAG, "%u saved readings, %.0f %% of the way", npoints, progress * 100);
    }
}

// With the lock held. Every ending saves something if it is worth saving, and
// says plainly what the user got.
static void finish(mag_cal_quality_t quality)
{
    params_before_save = params;
    params.valid = true;
    params.quality = quality;
    memcpy(params.offset, fit.offset, sizeof(params.offset));
    memcpy(params.matrix, fit.matrix, sizeof(params.matrix));
    params.radius = fit.radius;
    params.fit_rms = fit.rms;
    params.points = npoints;
    state = MAG_CAL_DONE;
    params_dirty = true;  // written by flush_nvs(), outside the lock
    points_changed();
    char tidy[40] = "";
    if (fit_tidied) {
        snprintf(tidy, sizeof(tidy), " after tidying to %u readings", fit.points);
    }
    switch (quality) {
    case MAG_CAL_GOOD:
        snprintf(message_buf, sizeof(message_buf), "Calibration saved%s: %.1f %% off the sphere. The compass is ready.",
                 tidy, fit.rms * 100);
        break;
    case MAG_CAL_ROUGH:
        snprintf(message_buf, sizeof(message_buf),
                 "Saved%s, but rough: %.1f %% off the sphere against %.1f %% for good. "
                 "The compass works; headings may be a degree or two out.",
                 tidy, fit.rms * 100, (double)ACCEPT_RMS * 100);
        break;
    default:
        snprintf(message_buf, sizeof(message_buf),
                 "Saved%s, but poor: %.1f %% off the sphere. The compass works roughly; "
                 "calibrate again standing still, clear of metal.",
                 tidy, fit.rms * 100);
        break;
    }
    ESP_LOGI(TAG, "saved (%s): offset %.1f %.1f %.1f µT, radius %.1f µT, fit %.2f %%, level %u, %u readings",
             quality_word(quality), fit.offset[0], fit.offset[1], fit.offset[2], fit.radius, fit.rms * 100,
             fit.level, fit.points);
}

// A fit costs a good fraction of a second on this chip, so it runs on a copy
// with the lock released, and its result is only used if the session it belongs
// to is still the current one.
// A mathematical cleanup: refit on just the readings the fit kept, so strays
// stop pulling on the answer. Taken only when it keeps most of the readings,
// really improves, and barely moves the centre — otherwise it would just be
// inventing a nicer-looking number.
static bool tidy_up(const float (*pts)[3], size_t n, const bool *keep, mag_fit_t *fit_io)
{
    float (*kept)[3] = scratch->kept;
    size_t k = 0;
    for (size_t i = 0; i < n; i++) {
        if (keep[i]) {
            memcpy(kept[k++], pts[i], sizeof(kept[0]));
        }
    }
    if (k < MIN_FIT_POINTS || k * 10 < n * 7) {
        return false;
    }
    mag_fit_t tidy;
    if (!mag_fit(kept, k, 0.0f, fit_io, &tidy, NULL, &scratch->fit) || tidy.rms > fit_io->rms * 0.9f ||
        dist(tidy.offset, fit_io->offset) > TIDY_MAX_SHIFT_UT) {
        return false;
    }
    *fit_io = tidy;
    return true;
}

static void run_fit(void)
{
    mag_fit_t start;
    mag_fit_t result;
    bool have_start;
    uint32_t id;
    size_t n;

    xSemaphoreTake(lock, portMAX_DELAY);
    if (loading || !scratch_acquire()) {
        xSemaphoreGive(lock);
        return;  // an upload owns the snapshot, or there is nothing to fit with
    }
    float (*snapshot)[3] = scratch->snapshot;
    bool *keep = scratch->keep;
    n = npoints;
    for (size_t i = 0; i < n; i++) {
        memcpy(snapshot[i], points[i].v, sizeof(snapshot[i]));
    }
    start = fit;
    have_start = have_fit;
    id = session_id;
    last_fit = esp_timer_get_time();
    fitting = true;  // the scratch is in use from here until the lock is retaken
    xSemaphoreGive(lock);

    int64_t began = esp_timer_get_time();
    bool ok = mag_fit(snapshot, n, 0.0f, have_start ? &start : NULL, &result, keep, &scratch->fit);
    bool tidied = ok && tidy_up(snapshot, n, keep, &result);
    uint32_t took = (uint32_t)((esp_timer_get_time() - began) / 1000);

    xSemaphoreTake(lock, portMAX_DELAY);
    fitting = false;
    if (id != session_id) {
        xSemaphoreGive(lock);  // the session was restarted while we fitted
        return;
    }
    fit_ms = took;
    if (ok) {
        stable_count = have_fit && settled(&result, &fit) ? stable_count + 1 : 0;
        fit = result;
        have_fit = true;
        fit_tidied = tidied;
        if (mag_fit_plausible_for_binning(&result)) {
            recentre_to(result.offset);
        }
        update_progress();
    }
    if (state == MAG_CAL_RUNNING) {
        char detail[sizeof(reason_buf)];
        const char *missing_criterion = unmet(detail, sizeof(detail));
        if (!missing_criterion && session_new < SESSION_MIN_NEW) {
            missing_criterion = "more fresh readings";  // a continued session must bring new data
        }
        strlcpy(blocking_buf, missing_criterion ? missing_criterion : "", sizeof(blocking_buf));
        if (!missing_criterion && session_new >= SESSION_MIN_NEW) {
            finish(MAG_CAL_GOOD);
        } else if (esp_timer_get_time() - session_start > SESSION_TIMEOUT_US) {
            // Rather than throw away five minutes of turning, keep what is usable.
            if (session_new >= SESSION_MIN_NEW && worth_saving()) {
                finish(quality_of());
                ESP_LOGW(TAG, "quality bar not met (%s), saved as %s", missing_criterion,
                         quality_word(params.quality));
            } else {
                char text[sizeof(reason_buf)];
                snprintf(text, sizeof(text), "Gave up after 5 minutes: %s",
                         missing_criterion ? missing_criterion : "too few new readings");
                fail(text);
            }
        }
    }
    xSemaphoreGive(lock);
}

static void cal_task(void *arg)
{
    static magnetometer_sample_t batch[32];
    uint32_t since;
    uint32_t latest;
    magnetometer_samples_since(0, batch, 0, &since);  // start from the newest
    int fresh = 0;

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(TASK_PERIOD_MS));
        size_t n = magnetometer_samples_since(since, batch, sizeof(batch) / sizeof(batch[0]), &latest);
        float k = magnetometer_ut_per_count();

        xSemaphoreTake(lock, portMAX_DELAY);
        if (state != MAG_CAL_RUNNING) {
            fresh = 0;
            // Nothing is calibrating, so nothing owns the fit scratch. Checked
            // here rather than paired with each way a session can end, so that
            // adding another way out cannot leak it.
            // An upload parsing into the scratch owns it too. There is
            // deliberately no timeout on that: memory cannot be taken back from
            // a task that still holds the pointer, and a reclaim on a slow
            // upload would free the block while the web server was writing into
            // it. Every exit from the upload handler gives it back instead, and
            // the body it reads is length-capped so it cannot run long.
            if (scratch && !fit_once && !fitting && !loading) {
                scratch_release();
            }
        }
        for (size_t i = 0; i < n; i++) {
            since = batch[i].seq;
            float m[3] = {batch[i].raw[0] * k, batch[i].raw[1] * k, batch[i].raw[2] * k};
            if (state != MAG_CAL_RUNNING || fabsf(m[0]) > SATURATION_UT || fabsf(m[1]) > SATURATION_UT ||
                fabsf(m[2]) > SATURATION_UT) {
                continue;
            }
            add_reading(m);
            fresh++;
            session_new++;
        }
        bool due = npoints >= MIN_FIT_POINTS &&
                   (fit_once || (state == MAG_CAL_RUNNING && fresh >= REFIT_AFTER &&
                                 esp_timer_get_time() - last_fit > FIT_EVERY_US));
        fit_once = false;
        bool save_due = state == MAG_CAL_RUNNING && esp_timer_get_time() - last_save > SAVE_EVERY_US;
        if (due) {
            fresh = 0;
        }
        if (save_due) {
            points_changed();
        }
        xSemaphoreGive(lock);

        if (due) {
            run_fit();
        }
        flush_nvs();
    }
}

esp_err_t mag_cal_init(void)
{
    lock = xSemaphoreCreateMutex();
    ESP_RETURN_ON_FALSE(lock, ESP_ERR_NO_MEM, TAG, "create lock");
    load();
    ESP_RETURN_ON_FALSE(xTaskCreate(cal_task, "mag_cal", 6144, NULL, 3, &task) == pdPASS, ESP_ERR_NO_MEM,
                        TAG, "create task");
    started = true;
    return ESP_OK;
}

esp_err_t mag_cal_begin(bool fresh)
{
    ESP_RETURN_ON_FALSE(started, ESP_ERR_INVALID_STATE, TAG, "calibration task not running");
    ESP_RETURN_ON_FALSE(magnetometer_ut_per_count() > 0, ESP_ERR_INVALID_STATE, TAG, "no magnetometer");
    xSemaphoreTake(lock, portMAX_DELAY);
    if (!scratch_acquire()) {
        xSemaphoreGive(lock);
        return ESP_ERR_NO_MEM;  // say so now, not silently three seconds later
    }
    if (fresh) {
        npoints = 0;
        arrivals = 0;
        points_changed();
        // The saved calibration is still a good first guess of the centre.
        if (params.valid) {
            memcpy(centre, params.offset, sizeof(centre));
        } else {
            memset(centre, 0, sizeof(centre));
        }
    }
    update_progress();
    reason_buf[0] = '\0';
    blocking_buf[0] = '\0';
    message_buf[0] = '\0';
    have_fit = false;
    fit_tidied = false;  // start the stability count over, and refit from scratch
    stable_count = 0;
    fit_ms = 0;
    session_new = 0;
    session_refused = 0;
    session_id++;
    session_start = last_save = esp_timer_get_time();
    last_fit = 0;
    state = MAG_CAL_RUNNING;
    xSemaphoreGive(lock);
    ESP_LOGI(TAG, "calibration %s at %.0f %%", fresh ? "started over" : "continued", progress * 100);
    return ESP_OK;
}

void mag_cal_stop(void)
{
    xSemaphoreTake(lock, portMAX_DELAY);
    if (state == MAG_CAL_RUNNING) {
        // Keep whatever was earned, rather than ending with nothing.
        if (session_new >= SESSION_MIN_NEW && worth_saving()) {
            finish(quality_of());
            state = MAG_CAL_STOPPED;
        } else {
            state = MAG_CAL_STOPPED;
            snprintf(message_buf, sizeof(message_buf), "Stopped at %.0f %%. Too little to save; %s", progress * 100,
                     params.valid ? "the previous calibration stays in use." : "the compass uses raw readings.");
        }
        points_changed();  // the task writes it, off this thread
        ESP_LOGI(TAG, "calibration stopped at %.0f %%", progress * 100);
    }
    xSemaphoreGive(lock);
}

float (*mag_cal_load_buffer(size_t *max))[3]
{
    if (!started) {
        return NULL;
    }
    xSemaphoreTake(lock, portMAX_DELAY);
    float (*buf)[3] = NULL;
    if (!fitting && !loading && scratch_acquire()) {
        buf = scratch->snapshot;
        *max = MAG_CAL_MAX_POINTS;
        loading = true;  // claims it until set_points or abandon
    }
    xSemaphoreGive(lock);
    return buf;
}

void mag_cal_load_abandon(void)
{
    xSemaphoreTake(lock, portMAX_DELAY);
    loading = false;
    xSemaphoreGive(lock);
}

esp_err_t mag_cal_set_points(const float (*pts)[3], size_t n)
{
    ESP_RETURN_ON_FALSE(started, ESP_ERR_INVALID_STATE, TAG, "calibration task not running");
    ESP_RETURN_ON_FALSE(n > 0, ESP_ERR_INVALID_ARG, TAG, "no readings");
    xSemaphoreTake(lock, portMAX_DELAY);
    loading = false;  // the upload is finished with the buffer
    state = MAG_CAL_IDLE;  // a loaded set is not a session
    session_id++;          // any fit still running belongs to the old set
    npoints = 0;
    have_fit = false;
    fit_tidied = false;
    stable_count = 0;
    session_refused = 0;
    for (size_t i = 0; i < n && i < MAG_CAL_MAX_POINTS; i++) {
        points[npoints].v[0] = pts[i][0];
        points[npoints].v[1] = pts[i][1];
        points[npoints].v[2] = pts[i][2];
        points[npoints].order = i;
        points[npoints].cell = 0;
        npoints++;
    }
    arrivals = npoints;
    // Bin them about the best centre we have: a poor centre scatters readings
    // into the wrong cells, where the per-cell cap then throws them away.
    if (params.valid) {
        memcpy(centre, params.offset, sizeof(centre));
    } else {
        range_centre(centre);
    }
    rebin();
    fit_once = true;  // let the task fit them and re-bin about the result
    update_progress();
    points_changed();
    snprintf(message_buf, sizeof(message_buf),
             "Loaded %u readings from a file, %.0f %% of directions covered. Continue calibration to fit them.",
             npoints, coverage * 100);
    ESP_LOGI(TAG, "loaded %u readings from a file", npoints);
    xSemaphoreGive(lock);
    return ESP_OK;
}

esp_err_t mag_cal_accept(void)
{
    esp_err_t err = ESP_ERR_INVALID_STATE;
    xSemaphoreTake(lock, portMAX_DELAY);
    if (state == MAG_CAL_RUNNING && session_new >= SESSION_MIN_NEW && worth_saving()) {
        finish(quality_of());
        err = ESP_OK;
    }
    xSemaphoreGive(lock);
    return err;
}

void mag_cal_get_status(mag_cal_status_t *out)
{
    xSemaphoreTake(lock, portMAX_DELAY);
    *out = (mag_cal_status_t){
        .state = state,
        .can_accept = state == MAG_CAL_RUNNING && session_new >= SESSION_MIN_NEW && worth_saving(),
        .progress = progress,
        .coverage = coverage,
        .missing = missing,
        .centre = {centre[0], centre[1], centre[2]},
        .points = npoints,
        .points_seq = points_seq,
        .refused = session_refused,
        .params = params,
        .have_fit = have_fit,
        .fit = fit,
        .fit_ms = fit_ms,
    };
    strlcpy(out->reason, reason_buf, sizeof(out->reason));
    strlcpy(out->blocking, blocking_buf, sizeof(out->blocking));
    strlcpy(out->message, message_buf, sizeof(out->message));
    xSemaphoreGive(lock);
}

size_t mag_cal_get_points(float (*out)[3], size_t max)
{
    xSemaphoreTake(lock, portMAX_DELAY);
    size_t n = npoints < max ? npoints : max;
    for (size_t i = 0; i < n; i++) {
        memcpy(out[i], points[i].v, sizeof(out[i]));
    }
    xSemaphoreGive(lock);
    return n;
}

bool mag_cal_get_point(size_t i, float out[3])
{
    bool ok = false;
    xSemaphoreTake(lock, portMAX_DELAY);
    if (i < npoints) {
        memcpy(out, points[i].v, sizeof(float) * 3);
        ok = true;
    }
    xSemaphoreGive(lock);
    return ok;
}

size_t mag_cal_stack_free(void)
{
    return task ? uxTaskGetStackHighWaterMark(task) * sizeof(StackType_t) : 0;
}

bool mag_cal_apply(const float m[3], float b[3])
{
    xSemaphoreTake(lock, portMAX_DELAY);
    mag_cal_params_t p = params;
    xSemaphoreGive(lock);
    if (!p.valid) {
        memcpy(b, m, 3 * sizeof(float));
        return false;
    }
    float d[3] = {m[0] - p.offset[0], m[1] - p.offset[1], m[2] - p.offset[2]};
    for (int i = 0; i < 3; i++) {
        b[i] = p.matrix[3 * i] * d[0] + p.matrix[3 * i + 1] * d[1] + p.matrix[3 * i + 2] * d[2];
    }
    return true;
}
