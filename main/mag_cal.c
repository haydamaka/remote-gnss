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
#define NVS_VERSION 2  // 2 added the rough flag
#define TASK_PERIOD_MS 100
#define REFIT_AFTER 20                // new readings before fitting again
#define FIT_EVERY_US (3 * 1000000LL)  // and no sooner: a fit costs a good fraction of a second
#define MIN_FIT_POINTS 20
#define RECENTRE_UT 2.0f              // centre shift that triggers re-binning
#define COVERAGE_DONE 0.9f            // share of cells counted as covered
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
#define ACCEPT_RMS 0.025f
// A rough calibration: the readings it kept sit on the ellipsoid, and it kept
// enough of them. Trimming barely moves the parameters (CALIBRATION.md §9), so
// this is still a usable calibration — it is only labelled as rough.
#define ROUGH_RMS 0.06f
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

static const char *TAG = "mag_cal";

typedef struct {
    float v[3];      // µT
    uint32_t order;  // arrival order, for oldest-first replacement
    uint8_t cell;
} point_t;

typedef struct {
    uint8_t version;
    uint8_t rough;
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
static mag_cal_params_t params;
static mag_cal_params_t params_before_save;
static mag_fit_t fit;
static bool have_fit;
static int stable_count;
static uint32_t fit_ms;
static bool points_dirty;
static bool params_dirty;
static bool started;
static TaskHandle_t task;

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

// Keeps up to MAG_CAL_PER_CELL readings per cell, replacing the cell's oldest.
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
    int oldest = oldest_in(p.cell);
    if ((count < MAG_CAL_PER_CELL || oldest < 0) && npoints < MAG_CAL_MAX_POINTS) {
        points[npoints++] = p;
    } else if (oldest >= 0) {
        points[oldest] = p;
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

// Whether the fit could be saved as a rough calibration right now.
static bool acceptable_rough(void)
{
    char scratch[sizeof(reason_buf)];
    return unmet_at(true, scratch, sizeof(scratch)) == NULL;
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
    state = MAG_CAL_FAILED;
    points_dirty = true;  // keep the progress for the next session
    ESP_LOGW(TAG, "calibration failed: %s", reason_buf);
}

// Writes what's due to NVS. Called from the task without the lock: a flash
// commit takes tens of milliseconds and would stall every HTTP request.
static void flush_nvs(void)
{
    static int16_t packed[MAG_CAL_MAX_POINTS][3];  // 0.1 µT units
    stored_params_t sp = {.version = NVS_VERSION, .rough = params.rough};
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
    points_dirty = params_dirty = false;
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
    if (write_params) {  // the calibration isn't stored, so don't keep using it
        xSemaphoreTake(lock, portMAX_DELAY);
        params = params_before_save;
        fail("Could not save the calibration");
        xSemaphoreGive(lock);
    }
}

static void load(void)
{
    static int16_t packed[MAG_CAL_MAX_POINTS][3];
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
        params.rough = sp.rough;
        memcpy(params.offset, sp.offset, sizeof(params.offset));
        memcpy(params.matrix, sp.matrix, sizeof(params.matrix));
        params.radius = sp.radius;
        params.fit_rms = sp.fit_rms;
        params.points = sp.points;
        memcpy(centre, sp.offset, sizeof(centre));
        ESP_LOGI(TAG, "calibration%s: offset %.1f %.1f %.1f µT, radius %.1f µT, fit %.1f %%",
                 params.rough ? " (rough)" : "", params.offset[0], params.offset[1], params.offset[2],
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

// With the lock held.
static void finish(bool rough)
{
    params_before_save = params;
    params.valid = true;
    params.rough = rough;
    memcpy(params.offset, fit.offset, sizeof(params.offset));
    memcpy(params.matrix, fit.matrix, sizeof(params.matrix));
    params.radius = fit.radius;
    params.fit_rms = fit.rms;
    params.points = npoints;
    state = MAG_CAL_DONE;
    params_dirty = points_dirty = true;  // written by flush_nvs(), outside the lock
    ESP_LOGI(TAG, "saved%s: offset %.1f %.1f %.1f µT, radius %.1f µT, fit %.2f %%, level %u, %u readings",
             rough ? " (rough)" : "", fit.offset[0], fit.offset[1], fit.offset[2], fit.radius, fit.rms * 100,
             fit.level, npoints);
}

// A fit costs a good fraction of a second on this chip, so it runs on a copy
// with the lock released, and its result is only used if the session it belongs
// to is still the current one.
static void run_fit(void)
{
    static float snapshot[MAG_CAL_MAX_POINTS][3];
    mag_fit_t start;
    mag_fit_t result;
    bool have_start;
    uint32_t id;
    size_t n;

    xSemaphoreTake(lock, portMAX_DELAY);
    n = npoints;
    for (size_t i = 0; i < n; i++) {
        memcpy(snapshot[i], points[i].v, sizeof(snapshot[i]));
    }
    start = fit;
    have_start = have_fit;
    id = session_id;
    last_fit = esp_timer_get_time();
    xSemaphoreGive(lock);

    int64_t began = esp_timer_get_time();
    bool ok = mag_fit(snapshot, n, 0.0f, have_start ? &start : NULL, &result, NULL);
    uint32_t took = (uint32_t)((esp_timer_get_time() - began) / 1000);

    xSemaphoreTake(lock, portMAX_DELAY);
    if (id != session_id) {
        xSemaphoreGive(lock);  // the session was restarted while we fitted
        return;
    }
    fit_ms = took;
    if (ok) {
        stable_count = have_fit && settled(&result, &fit) ? stable_count + 1 : 0;
        fit = result;
        have_fit = true;
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
            finish(false);
        } else if (esp_timer_get_time() - session_start > SESSION_TIMEOUT_US) {
            // Rather than throw away five minutes of turning, keep what is usable.
            if (session_new >= SESSION_MIN_NEW && acceptable_rough()) {
                finish(true);
                ESP_LOGW(TAG, "quality bar not met (%s), saved as rough", missing_criterion);
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
        bool due = state == MAG_CAL_RUNNING && fresh >= REFIT_AFTER && npoints >= MIN_FIT_POINTS &&
                   esp_timer_get_time() - last_fit > FIT_EVERY_US;
        bool save_due = state == MAG_CAL_RUNNING && esp_timer_get_time() - last_save > SAVE_EVERY_US;
        if (due) {
            fresh = 0;
        }
        if (save_due) {
            points_dirty = true;
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
    if (fresh) {
        npoints = 0;
        arrivals = 0;
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
    have_fit = false;  // start the stability count over, and refit from scratch
    stable_count = 0;
    fit_ms = 0;
    session_new = 0;
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
        state = MAG_CAL_STOPPED;
        points_dirty = true;  // the task writes it, off this thread
        ESP_LOGI(TAG, "calibration stopped at %.0f %%", progress * 100);
    }
    xSemaphoreGive(lock);
}

esp_err_t mag_cal_accept(void)
{
    esp_err_t err = ESP_ERR_INVALID_STATE;
    xSemaphoreTake(lock, portMAX_DELAY);
    if (state == MAG_CAL_RUNNING && acceptable_rough()) {
        finish(true);
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
        .can_accept = state == MAG_CAL_RUNNING && acceptable_rough(),
        .progress = progress,
        .coverage = coverage,
        .missing = missing,
        .centre = {centre[0], centre[1], centre[2]},
        .points = npoints,
        .params = params,
        .have_fit = have_fit,
        .fit = fit,
        .fit_ms = fit_ms,
    };
    strlcpy(out->reason, reason_buf, sizeof(out->reason));
    strlcpy(out->blocking, blocking_buf, sizeof(out->blocking));
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
