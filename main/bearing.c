#include "bearing.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "nvs.h"

#include "gnss.h"
#include "mag_cal.h"
#include "magnetometer.h"
#include "wmm.h"

#define TAG "bearing"
#define NVS_NAMESPACE "bearing"
#define NVS_KEY_STATE "state"
#define NVS_VERSION 2

#define TICK_MS 100          // the receiver and the magnetometer both run at 10 Hz
#define TRACE_EVERY_S 60.0f  // one trace record per minute of qualifying driving
#define SAVE_EVERY_S 300.0f  // and to flash every five minutes, not oftener
#define MAG_STALE_US (500 * 1000)
#define WMM_EVERY_S 60.0f    // declination hardly moves; once a minute is plenty
#define FIELD_TOLERANCE 0.2f // how far the horizontal field may be from the WMM's

// Everything worth keeping between drives, laid out so it can go to NVS as one
// blob with no copy: the live ring *is* the saved ring. It has no pointers, and
// the in-flight window is dropped on a restart.
typedef struct {
    uint32_t version;
    bool learning, applying;
    float total_s;  // qualifying driving, summed across restarts: the readout
    float run_s;    // ...and the clock the estimator sees, which also runs
                    // while stopped, so a window cannot span a traffic light
    bearing_cal_t cal;
    uint32_t trace_first, trace_n;
    bearing_trace_t trace[BEARING_TRACE_MAX];
} stored_t;

static stored_t st = {.version = NVS_VERSION, .learning = true};

// One mutex for everything in here. It was a spinlock for the result and a
// separate mutex for the trace, which left the rest of `st` - the estimator's
// own state - with no owner at all: the web server could be blocked inside
// nvs_set_blob() writing `st` while this task was still feeding readings into
// st.cal, tearing both the live window and the saved copy. A mutex, not a
// spinlock, because a save holds it across a flash write.
static SemaphoreHandle_t st_lock;
static bearing_result_t latest;
static TaskHandle_t task;

// st.total_s is a clock that survives a restart, so a power cycle in a car park
// does not reset the drift measurement. It only advances while the device is
// doing something that could qualify.
static float last_trace_s, last_save_s;
static bool dirty;

static uint32_t fixes_seen, fixes_usable, blocked[BEARING_BLOCKS];

// Call with st_lock held.
static void trace_push(const bearing_trace_t *r)
{
    if (st.trace_n < BEARING_TRACE_MAX) {
        st.trace[(st.trace_first + st.trace_n++) % BEARING_TRACE_MAX] = *r;
    } else {
        st.trace[st.trace_first] = *r;  // the ring is full: the oldest minute goes
        st.trace_first = (st.trace_first + 1) % BEARING_TRACE_MAX;
    }
}

size_t bearing_trace_count(void)
{
    xSemaphoreTake(st_lock, portMAX_DELAY);
    size_t n = st.trace_n;
    xSemaphoreGive(st_lock);
    return n;
}

bool bearing_trace_at(size_t i, bearing_trace_t *out)
{
    bool ok = false;
    xSemaphoreTake(st_lock, portMAX_DELAY);
    if (i < st.trace_n) {
        *out = st.trace[(st.trace_first + i) % BEARING_TRACE_MAX];
        ok = true;
    }
    xSemaphoreGive(st_lock);
    return ok;
}

const char *bearing_block_name(bearing_block_t b)
{
    switch (b) {
    case BEARING_BLOCK_NO_FIX:       return "no fix";
    case BEARING_BLOCK_SPOOFED:      return "the fix looks faked";
    case BEARING_BLOCK_FEW_SATS:     return "too few satellites";
    case BEARING_BLOCK_HDOP:         return "the fix is too loose";
    case BEARING_BLOCK_NO_MAG:       return "no magnetometer reading";
    case BEARING_BLOCK_UNCALIBRATED: return "the magnetometer is not calibrated";
    case BEARING_BLOCK_NO_WMM:       return "no declination for here";
    case BEARING_BLOCK_FIELD_OFF:    return "the field is not what it should be here";
    default:                         return "";
    }
}

// ---- storage ----

// Call with st_lock held.
static esp_err_t save_st(void)
{
    nvs_handle_t nvs;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs);
    if (err != ESP_OK) {
        return err;
    }
    int keep = st.cal.n;
    st.cal.n = 0;  // an unfinished window is not worth keeping
    err = nvs_set_blob(nvs, NVS_KEY_STATE, &st, sizeof(st));
    st.cal.n = keep;
    if (err == ESP_OK) {
        err = nvs_commit(nvs);
    }
    nvs_close(nvs);
    if (err == ESP_OK) {
        dirty = false;
    }
    return err;
}

esp_err_t bearing_save(void)
{
    xSemaphoreTake(st_lock, portMAX_DELAY);
    esp_err_t err = save_st();
    xSemaphoreGive(st_lock);
    return err;
}

static void load(void)
{
    bearing_cfg_t cfg;
    bearing_cal_defaults(&cfg);
    bearing_cal_init(&st.cal, &cfg);

    nvs_handle_t nvs;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs) != ESP_OK) {
        ESP_LOGI(TAG, "nothing learned yet");
        return;
    }
    size_t len = sizeof(st);
    esp_err_t err = nvs_get_blob(nvs, NVS_KEY_STATE, &st, &len);
    nvs_close(nvs);
    if (err != ESP_OK || len != sizeof(st) || st.version != NVS_VERSION) {
        // Anything unexpected and it starts clean rather than half-loaded.
        memset(&st, 0, sizeof(st));
        st.version = NVS_VERSION;
        st.learning = true;
        bearing_cal_init(&st.cal, &cfg);
        ESP_LOGI(TAG, "nothing learned yet");
        return;
    }
    st.cal.n = 0;
    if (st.trace_n > BEARING_TRACE_MAX) {
        st.trace_n = 0;
        st.trace_first = 0;
    }
    bearing_cal_result(&st.cal, &latest);  // or it reads as nothing until the next drive
    ESP_LOGI(TAG, "%.2f h of driving learned so far, %s%s", st.total_s / 3600.0,
             st.learning ? "collecting" : "not collecting",
             st.applying ? ", correcting the heading" : ", shadow only");
    last_trace_s = last_save_s = st.total_s;
}

// ---- collecting ----

// Declination and the field strength to expect here, refreshed now and then.
static float wmm_at_s = -1e9f;
static bool wmm_have;
static wmm_field_t wmm_f;

static bool local_field(const gnss_data_t *g, float *decl, float *horizontal)
{
    // The "or the clock went backwards" case is a reset, which puts run_s back
    // to zero and would otherwise reuse the old field for hours of driving.
    if (st.run_s - wmm_at_s > WMM_EVERY_S || st.run_s < wmm_at_s || !wmm_have) {
        int y, mo, d;
        if (g->fix && sscanf(g->utc, "%d-%d-%d", &y, &mo, &d) == 3) {
            float year = wmm_decimal_year(y, mo, d);
            if (year > WMM_VALID_UNTIL) {
                year = WMM_VALID_UNTIL - 0.01f;  // stale beats none for this
            }
            wmm_have = wmm_field(g->latitude, g->longitude, g->altitude_m, year, &wmm_f);
            wmm_at_s = st.run_s;
        }
    }
    if (!wmm_have) {
        return false;
    }
    *decl = wmm_f.declination;
    *horizontal = wmm_f.horizontal;
    // The once-round amplitude is reported in microtesla, which needs the field
    // here rather than the default the estimator was built with.
    st.cal.cfg.h_field_ut = wmm_f.horizontal;
    return true;
}

// Call with st_lock held.
static void tick(void)
{
    static int64_t last_epoch;

    gnss_data_t g;
    magnetometer_data_t m;
    gnss_get(&g);
    magnetometer_get(&m);

    // One sample per navigation solution, not per task tick: a ZED-F9P fixes
    // once a second by default and an M10 ten times, and feeding the same fix
    // ten times over would put ten copies of one measurement into the window.
    if (g.epoch_us == last_epoch) {
        return;
    }
    last_epoch = g.epoch_us;

    float decl = 0.0f, horizontal = 0.0f;
    bool have_wmm = local_field(&g, &decl, &horizontal);

    float field[3];
    bool calibrated = mag_cal_apply(m.field_ut, field);
    int64_t now_us = esp_timer_get_time();
    bool mag_fresh = m.updated_us && now_us - m.updated_us < MAG_STALE_US;

    float heading = 0.0f, h_mag = 0.0f;
    if (mag_fresh) {
        heading = atan2f(field[1], field[0]) * 180.0f / (float)M_PI;
        if (heading < 0.0f) {
            heading += 360.0f;
        }
        h_mag = hypotf(field[0], field[1]);
    }

    // Everything that has to be true before a fix is worth looking at. The
    // straightness tests are bearing_cal's; these are about the data itself.
    // Each failure is counted, so a drive that produced nothing can say why.
    bool usable = true;
    fixes_seen++;
    if (!g.fix || !isfinite(g.speed_kmh) || !isfinite(g.course_deg)) {
        blocked[BEARING_BLOCK_NO_FIX]++;
        usable = false;
    } else if (g.implausible || g.spoof > GNSS_SPOOF_NONE) {
        blocked[BEARING_BLOCK_SPOOFED]++;
        usable = false;
    } else if (g.satellites < 5) {
        blocked[BEARING_BLOCK_FEW_SATS]++;
        usable = false;
    } else if (!isfinite(g.hdop) || g.hdop >= 2.0f) {
        blocked[BEARING_BLOCK_HDOP]++;
        usable = false;
    } else if (!mag_fresh) {
        blocked[BEARING_BLOCK_NO_MAG]++;
        usable = false;
    } else if (!calibrated) {
        blocked[BEARING_BLOCK_UNCALIBRATED]++;
        usable = false;
    } else if (!have_wmm) {
        blocked[BEARING_BLOCK_NO_WMM]++;
        usable = false;
    } else if (fabsf(h_mag - horizontal) > FIELD_TOLERANCE * horizontal) {
        blocked[BEARING_BLOCK_FIELD_OFF]++;
        usable = false;
    } else {
        fixes_usable++;
    }

    bearing_sample_t s = {
        .t = st.run_s,
        .course_deg = g.course_deg,
        .speed_mps = isfinite(g.speed_kmh) ? g.speed_kmh / 3.6f : 0.0f,
        .heading_deg = heading,
        .decl_deg = decl,
        .usable = usable,
    };
    bearing_cal_feed(&st.cal, &s);

    bearing_result_t r;
    bearing_cal_result(&st.cal, &r);
    latest = r;
}

static void bearing_task(void *arg)
{
    TickType_t last = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&last, pdMS_TO_TICKS(TICK_MS));

        gnss_data_t g;
        gnss_get(&g);
        // The clock only runs while the device is doing something that could
        // qualify, so "two hours of driving" means two hours of driving.
        bool moving = g.fix && isfinite(g.speed_kmh) && g.speed_kmh / 3.6f >= 3.0f;

        xSemaphoreTake(st_lock, portMAX_DELAY);
        // run_s always advances; total_s only while the device is doing
        // something that could qualify, so "two hours of driving" means two
        // hours of driving. The estimator is given run_s: it has its own speed
        // gate, and it needs to see a stop as the gap it is, or a run either
        // side of a traffic light becomes one straight eight seconds.
        st.run_s += TICK_MS / 1000.0f;
        if (moving) {
            st.total_s += TICK_MS / 1000.0f;
        }

        if (st.learning) {
            tick();
            dirty = true;
        }

        if (moving && st.total_s - last_trace_s >= TRACE_EVERY_S) {
            last_trace_s = st.total_s;
            bearing_trace_t rec;
            bearing_cal_trace(&st.cal, (uint32_t)st.total_s, &rec);
            trace_push(&rec);
        }
        if (dirty && moving && st.total_s - last_save_s >= SAVE_EVERY_S) {
            last_save_s = st.total_s;
            esp_err_t err = save_st();
            if (err != ESP_OK) {
                ESP_LOGW(TAG, "could not save: %s", esp_err_to_name(err));
            }
        }
        xSemaphoreGive(st_lock);
    }
}

esp_err_t bearing_start(void)
{
    st_lock = xSemaphoreCreateMutex();
    if (!st_lock) {
        return ESP_ERR_NO_MEM;
    }
    load();
    // wmm_field alone takes 2 kB in one frame, so this is not generous.
    if (xTaskCreate(bearing_task, "bearing", 4608, NULL, 4, &task) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "watching for straight runs above 3 m/s");
    return ESP_OK;
}

void bearing_get(bearing_status_t *out)
{
    // One hold, so the reported snapshot cannot mix a before and an after.
    xSemaphoreTake(st_lock, portMAX_DELAY);
    out->result = latest;
    out->learning = st.learning;
    out->applying = st.applying;
    out->have = out->result.have;
    out->offset_deg = out->result.offset_deg;
    out->total_hours = st.total_s / 3600.0f;
    out->trace_used = st.trace_n;
    out->fixes_seen = fixes_seen;
    out->fixes_usable = fixes_usable;
    memcpy(out->blocked, blocked, sizeof(out->blocked));
    xSemaphoreGive(st_lock);
    out->stack_free = task ? uxTaskGetStackHighWaterMark(task) : 0;
}

esp_err_t bearing_set_learning(bool on)
{
    xSemaphoreTake(st_lock, portMAX_DELAY);
    st.learning = on;
    esp_err_t err = save_st();
    xSemaphoreGive(st_lock);
    return err;
}

esp_err_t bearing_set_applying(bool on)
{
    xSemaphoreTake(st_lock, portMAX_DELAY);
    st.applying = on;
    esp_err_t err = save_st();
    xSemaphoreGive(st_lock);
    return err;
}

esp_err_t bearing_reset(void)
{
    bearing_cfg_t cfg;
    bearing_cal_defaults(&cfg);
    xSemaphoreTake(st_lock, portMAX_DELAY);
    bool was_learning = st.learning, was_applying = st.applying;
    memset(&latest, 0, sizeof(latest));
    memset(&st, 0, sizeof(st));
    st.version = NVS_VERSION;
    st.learning = was_learning;
    st.applying = was_applying;
    bearing_cal_init(&st.cal, &cfg);
    last_trace_s = last_save_s = 0.0f;
    wmm_at_s = -1e9f;  // the clock went back to zero; do not reuse a stale field
    wmm_have = false;
    fixes_seen = fixes_usable = 0;
    memset(blocked, 0, sizeof(blocked));
    esp_err_t err = save_st();
    xSemaphoreGive(st_lock);
    return err;
}

