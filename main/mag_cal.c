#include "mag_cal.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "nvs.h"

#include "magnetometer.h"

#define NVS_NAMESPACE "magcal"
#define NVS_VERSION 1
#define TASK_PERIOD_MS 100
#define REFIT_EVERY 10            // new readings between refits
#define MIN_FIT_POINTS 20
#define RECENTRE_UT 2.0f          // centre shift that triggers re-binning
#define COVERAGE_DONE 0.9f        // share of cells counted as covered
#define CELL_MIN 2                // readings for a cell to count as covered
#define CAP_MIN 5                 // readings within 45° of each of ±X, ±Y, ±Z
#define CAP_COS 0.70710678f
#define SESSION_TIMEOUT_US (5 * 60 * 1000000LL)
#define SESSION_MIN_NEW 100       // new readings before a continued session may complete
#define SAVE_EVERY_US (30 * 1000000LL)
#define RADIUS_MIN 20.0f          // plausible Earth field, µT
#define RADIUS_MAX 70.0f
#define FIT_RMS_MAX 0.10f         // loose: a sphere can't absorb soft iron
#define OUTLIER_FLOOR 0.02f       // never gate tighter than 2 % of the radius
#define SATURATION_UT 1000.0f

static const char *TAG = "mag_cal";

typedef struct {
    float v[3];      // µT
    uint32_t order;  // arrival order, for oldest-first replacement
    uint8_t cell;
} point_t;

typedef struct {
    uint8_t version;
    float offset[3];
    float matrix[9];
    float radius;
    float fit_rms;
    uint16_t points;
} stored_params_t;

// Everything below is guarded by lock.
static SemaphoreHandle_t lock;
static mag_cal_state_t state = MAG_CAL_IDLE;
static char reason_buf[64];  // empty unless the last session failed
static point_t points[MAG_CAL_MAX_POINTS];
static uint16_t npoints;
static uint32_t arrivals;
static float centre[3];
static float coverage;
static float progress;
static uint8_t missing = 0x3F;
static int64_t session_start;
static int64_t last_save;
static int session_new;
static mag_cal_params_t params;
static mag_cal_params_t params_before_save;
static bool points_dirty;
static bool params_dirty;
static bool started;

static float dist(const float a[3], const float b[3])
{
    float d[3] = {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
    return sqrtf(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
}

static uint8_t band(float lon, int n, int first)
{
    int i = (int)(lon / (2.0f * (float)M_PI) * n);
    return first + (i < n - 1 ? i : n - 1);
}

// 100 equal-area cells: bands of 1, 15, 34, 34, 15 and 1 cells split at z = 0.98,
// 0.68, 0, -0.68, -0.98 (a sphere's area is proportional to z). CALIBRATION.md
// §3.2 defines this; tools/mag_calibrate.py has the other copy.
static uint8_t cell_of(const float u[3])
{
    float lon = atan2f(u[1], u[0]) + (float)M_PI;
    if (u[2] > 0.98f) return 0;
    if (u[2] > 0.68f) return band(lon, 15, 1);
    if (u[2] > 0.0f) return band(lon, 34, 16);
    if (u[2] > -0.68f) return band(lon, 34, 50);
    if (u[2] > -0.98f) return band(lon, 15, 84);
    return 99;
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
    point_t p = {.v = {m[0], m[1], m[2]}, .order = arrivals++, .cell = cell_of(u)};
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
        points[i].cell = direction(points[i].v, u) ? cell_of(u) : 0;
        count[points[i].cell]++;
    }
    for (int c = 0; c < MAG_CAL_CELLS; c++) {
        for (; count[c] > MAG_CAL_PER_CELL; count[c]--) {
            int victim = oldest_in(c);  // before --npoints: the order isn't defined within one expression
            points[victim] = points[--npoints];
        }
    }
}

// Least-squares sphere (Kåsa: solves |p|² = 2·p·c + d) through the readings with
// use[i] set, or all if use is NULL. False if degenerate.
static bool sphere_fit(const bool *use, float c[3], float *r)
{
    double m[4][5] = {0};  // normal equations, augmented
    int n = 0;
    for (int i = 0; i < npoints; i++) {
        if (use && !use[i]) {
            continue;
        }
        const float *p = points[i].v;
        double row[4] = {2.0 * p[0], 2.0 * p[1], 2.0 * p[2], 1.0};
        double b = (double)p[0] * p[0] + (double)p[1] * p[1] + (double)p[2] * p[2];
        for (int a = 0; a < 4; a++) {
            for (int k = 0; k < 4; k++) {
                m[a][k] += row[a] * row[k];
            }
            m[a][4] += row[a] * b;
        }
        n++;
    }
    if (n < 10) {
        return false;
    }
    for (int col = 0; col < 4; col++) {  // Gaussian elimination, partial pivoting
        int piv = col;
        for (int r2 = col + 1; r2 < 4; r2++) {
            if (fabs(m[r2][col]) > fabs(m[piv][col])) {
                piv = r2;
            }
        }
        if (fabs(m[piv][col]) < 1e-9) {
            return false;
        }
        for (int k = 0; k < 5; k++) {
            double t = m[col][k];
            m[col][k] = m[piv][k];
            m[piv][k] = t;
        }
        for (int r2 = col + 1; r2 < 4; r2++) {
            double f = m[r2][col] / m[col][col];
            for (int k = col; k < 5; k++) {
                m[r2][k] -= f * m[col][k];
            }
        }
    }
    double s[4];
    for (int i = 3; i >= 0; i--) {
        double acc = m[i][4];
        for (int k = i + 1; k < 4; k++) {
            acc -= m[i][k] * s[k];
        }
        s[i] = acc / m[i][i];
    }
    double r2 = s[3] + s[0] * s[0] + s[1] * s[1] + s[2] * s[2];
    if (!(r2 > 1.0 && r2 < 1e6)) {
        return false;
    }
    for (int i = 0; i < 3; i++) {
        c[i] = (float)s[i];
    }
    *r = (float)sqrt(r2);
    return true;
}

static int cmp_float(const void *a, const void *b)
{
    float x = *(const float *)a;
    float y = *(const float *)b;
    return (x > y) - (x < y);
}

// Sphere fit, then again without the readings more than 3 robust standard
// deviations (MAD-based) from it. Returns the inliers' RMS of |p − c|/r − 1,
// or a negative number if the fit failed.
static float robust_sphere_fit(float c[3], float *r)
{
    // Only the calibration task fits, so these can be static.
    static float res[MAG_CAL_MAX_POINTS];
    static float tmp[MAG_CAL_MAX_POINTS];
    static bool use[MAG_CAL_MAX_POINTS];

    if (!sphere_fit(NULL, c, r)) {
        return -1.0f;
    }
    for (int i = 0; i < npoints; i++) {
        res[i] = tmp[i] = dist(points[i].v, c) / *r - 1.0f;
    }
    qsort(tmp, npoints, sizeof(float), cmp_float);
    float median = tmp[npoints / 2];
    for (int i = 0; i < npoints; i++) {
        tmp[i] = fabsf(res[i] - median);
    }
    qsort(tmp, npoints, sizeof(float), cmp_float);
    float gate = fmaxf(3.0f * 1.4826f * tmp[npoints / 2], OUTLIER_FLOOR);
    for (int i = 0; i < npoints; i++) {
        use[i] = fabsf(res[i] - median) <= gate;
    }
    if (!sphere_fit(use, c, r)) {
        return -1.0f;
    }
    double sum = 0;
    int n = 0;
    for (int i = 0; i < npoints; i++) {
        if (use[i]) {
            float e = dist(points[i].v, c) / *r - 1.0f;
            sum += e * e;
            n++;
        }
    }
    return (float)sqrt(sum / n);
}

// A fit through an arc of readings can come out as a huge sphere. Binning around
// its centre would pile every reading into one or two cells, and the per-cell cap
// would then throw most of them away.
static bool plausible(const float c[3], float r)
{
    const float zero[3] = {0};
    return r > 10.0f && r < 140.0f && dist(c, zero) < 500.0f;
}

// Whether re-binning around c would keep at least half the readings.
static bool rebin_keeps_most(const float c[3])
{
    uint16_t count[MAG_CAL_CELLS] = {0};  // a single cell can hold every reading
    for (int i = 0; i < npoints; i++) {
        float u[3];
        count[direction_from(c, points[i].v, u) ? cell_of(u) : 0]++;
    }
    int evicted = 0;
    for (int k = 0; k < MAG_CAL_CELLS; k++) {
        evicted += count[k] > MAG_CAL_PER_CELL ? count[k] - MAG_CAL_PER_CELL : 0;
    }
    return evicted <= npoints / 2;
}

// The latest plausible fit's centre, or the middle of the readings' range until
// there is one.
static void estimate_centre(float c[3])
{
    float r;
    if (npoints >= MIN_FIT_POINTS && robust_sphere_fit(c, &r) >= 0 && plausible(c, r)) {
        return;
    }
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

// Moves the centre to estimate_centre() and re-bins when it moved noticeably.
static void recentre(void)
{
    float c[3];
    estimate_centre(c);
    if (dist(c, centre) > RECENTRE_UT && rebin_keeps_most(c)) {
        memcpy(centre, c, sizeof(centre));
        rebin();
    }
}

static void update_progress(void)
{
    uint16_t count[MAG_CAL_CELLS] = {0};  // a single cell can hold every reading
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
    // Reaches 1 when coverage and the caps are done. Completing also needs
    // SESSION_MIN_NEW fresh readings, which this doesn't show.
    progress = 0.85f * fminf(1.0f, coverage / COVERAGE_DONE) + 0.15f * caps_ok / 6.0f;
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
    stored_params_t sp = {.version = NVS_VERSION};
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
        memcpy(params.offset, sp.offset, sizeof(params.offset));
        memcpy(params.matrix, sp.matrix, sizeof(params.matrix));
        params.radius = sp.radius;
        params.fit_rms = sp.fit_rms;
        params.points = sp.points;
        memcpy(centre, sp.offset, sizeof(centre));
        ESP_LOGI(TAG, "calibration: offset %.1f %.1f %.1f µT, radius %.1f µT, fit %.1f %%", params.offset[0],
                 params.offset[1], params.offset[2], params.radius, params.fit_rms * 100);
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
            estimate_centre(centre);
        }
        rebin();
        update_progress();
        ESP_LOGI(TAG, "%u saved readings, %.0f %% of the way", npoints, progress * 100);
    }
}

static void finish(void)
{
    float c[3];
    float r;
    char text[sizeof(reason_buf)];
    float rms = robust_sphere_fit(c, &r);
    if (rms < 0) {
        fail("The readings don't form a sphere");
        return;
    }
    if (r < RADIUS_MIN || r > RADIUS_MAX) {
        snprintf(text, sizeof(text), "Field strength %.0f µT is outside %.0f-%.0f µT", r, (double)RADIUS_MIN,
                 (double)RADIUS_MAX);
        fail(text);
        return;
    }
    if (rms > FIT_RMS_MAX) {
        snprintf(text, sizeof(text), "Readings too scattered (%.0f %% from the sphere)", rms * 100);
        fail(text);
        return;
    }
    params_before_save = params;
    params = (mag_cal_params_t){
        .valid = true,
        .offset = {c[0], c[1], c[2]},
        .matrix = {1, 0, 0, 0, 1, 0, 0, 0, 1},
        .radius = r,
        .fit_rms = rms,
        .points = npoints,
    };
    memcpy(centre, c, sizeof(centre));
    state = MAG_CAL_DONE;
    params_dirty = points_dirty = true;  // written by flush_nvs(), outside the lock
    ESP_LOGI(TAG, "saved: offset %.1f %.1f %.1f µT, radius %.1f µT, fit %.1f %%, %u points", c[0], c[1], c[2], r,
             rms * 100, npoints);
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
        if (state == MAG_CAL_RUNNING && fresh >= REFIT_EVERY) {
            fresh = 0;
            recentre();
            update_progress();
            int64_t now = esp_timer_get_time();
            if (coverage >= COVERAGE_DONE && missing == 0 && session_new >= SESSION_MIN_NEW) {
                finish();
            } else if (now - session_start > SESSION_TIMEOUT_US) {
                fail("Timed out after 5 minutes");
            } else if (now - last_save > SAVE_EVERY_US) {
                points_dirty = true;
            }
        }
        xSemaphoreGive(lock);
        flush_nvs();
    }
}

esp_err_t mag_cal_init(void)
{
    lock = xSemaphoreCreateMutex();
    ESP_RETURN_ON_FALSE(lock, ESP_ERR_NO_MEM, TAG, "create lock");
    load();
    ESP_RETURN_ON_FALSE(xTaskCreate(cal_task, "mag_cal", 4096, NULL, 3, NULL) == pdPASS, ESP_ERR_NO_MEM,
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
    session_new = 0;
    session_start = last_save = esp_timer_get_time();
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

void mag_cal_get_status(mag_cal_status_t *out)
{
    xSemaphoreTake(lock, portMAX_DELAY);
    *out = (mag_cal_status_t){
        .state = state,
        .progress = progress,
        .coverage = coverage,
        .missing = missing,
        .centre = {centre[0], centre[1], centre[2]},
        .points = npoints,
        .params = params,
    };
    strlcpy(out->reason, reason_buf, sizeof(out->reason));
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
