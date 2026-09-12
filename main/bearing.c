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
#define NVS_VERSION 1

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
    float total_s;  // qualifying driving, summed across restarts
    bearing_cal_t cal;
    uint32_t trace_first, trace_n;
    bearing_trace_t trace[BEARING_TRACE_MAX];
} stored_t;

static stored_t st = {.version = NVS_VERSION, .learning = true};

// The small result is read under a spinlock; the 4 kB trace is not, because
// holding interrupts off for that long would upset the WiFi driver.
static portMUX_TYPE lock = portMUX_INITIALIZER_UNLOCKED;
static SemaphoreHandle_t trace_lock;
static bearing_result_t latest;
static TaskHandle_t task;

// st.total_s is a clock that survives a restart, so a power cycle in a car park
// does not reset the drift measurement. It only advances while the device is
// doing something that could qualify.
static float last_trace_s, last_save_s;
static bool dirty;

static uint32_t fixes_seen, fixes_usable, blocked[BEARING_BLOCKS];

static void trace_push(const bearing_trace_t *r)
{
    xSemaphoreTake(trace_lock, portMAX_DELAY);
    if (st.trace_n < BEARING_TRACE_MAX) {
        st.trace[(st.trace_first + st.trace_n++) % BEARING_TRACE_MAX] = *r;
    } else {
        st.trace[st.trace_first] = *r;  // the ring is full: the oldest minute goes
        st.trace_first = (st.trace_first + 1) % BEARING_TRACE_MAX;
    }
    xSemaphoreGive(trace_lock);
}

size_t bearing_trace_count(void)
{
    return st.trace_n;
}

bool bearing_trace_at(size_t i, bearing_trace_t *out)
{
    bool ok = false;
    xSemaphoreTake(trace_lock, portMAX_DELAY);
    if (i < st.trace_n) {
        *out = st.trace[(st.trace_first + i) % BEARING_TRACE_MAX];
        ok = true;
    }
    xSemaphoreGive(trace_lock);
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

static esp_err_t save_locked(void)
{
    nvs_handle_t nvs;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs);
    if (err != ESP_OK) {
        return err;
    }
    uint16_t keep = st.cal.n;
    st.cal.n = 0;  // an unfinished window is not worth keeping
    xSemaphoreTake(trace_lock, portMAX_DELAY);
    err = nvs_set_blob(nvs, NVS_KEY_STATE, &st, sizeof(st));
    xSemaphoreGive(trace_lock);
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
    return save_locked();
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
    ESP_LOGI(TAG, "%.2f h of driving learned so far, %s%s", st.total_s / 3600.0,
             st.learning ? "collecting" : "not collecting",
             st.applying ? ", correcting the heading" : ", shadow only");
    last_trace_s = last_save_s = st.total_s;
}

// ---- collecting ----

// Declination and the field strength to expect here, refreshed now and then.
static bool local_field(const gnss_data_t *g, float *decl, float *horizontal)
{
    static float at_s = -1e9f;
    static bool have;
    static wmm_field_t f;

    if (st.total_s - at_s > WMM_EVERY_S || !have) {
        int y, mo, d;
        if (g->fix && sscanf(g->utc, "%d-%d-%d", &y, &mo, &d) == 3) {
            float year = wmm_decimal_year(y, mo, d);
            if (year > WMM_VALID_UNTIL) {
                year = WMM_VALID_UNTIL - 0.01f;  // stale beats none for this
            }
            have = wmm_field(g->latitude, g->longitude, g->altitude_m, year, &f);
            at_s = st.total_s;
        }
    }
    if (!have) {
        return false;
    }
    *decl = f.declination;
    *horizontal = f.horizontal;
    return true;
}

static void tick(void)
{
    gnss_data_t g;
    magnetometer_data_t m;
    gnss_get(&g);
    magnetometer_get(&m);

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
        .t = st.total_s,
        .course_deg = g.course_deg,
        .speed_mps = isfinite(g.speed_kmh) ? g.speed_kmh / 3.6f : 0.0f,
        .heading_deg = heading,
        .decl_deg = decl,
        .usable = usable,
    };
    bearing_cal_feed(&st.cal, &s);

    bearing_result_t r;
    bearing_cal_result(&st.cal, &r);
    taskENTER_CRITICAL(&lock);
    latest = r;
    taskEXIT_CRITICAL(&lock);
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
        if (!moving) {
            continue;
        }
        st.total_s += TICK_MS / 1000.0f;

        if (st.learning) {
            tick();
            dirty = true;
        }

        if (st.total_s - last_trace_s >= TRACE_EVERY_S) {
            last_trace_s = st.total_s;
            bearing_trace_t rec;
            bearing_cal_trace(&st.cal, (uint32_t)st.total_s, &rec);
            trace_push(&rec);
        }
        if (dirty && st.total_s - last_save_s >= SAVE_EVERY_S) {
            last_save_s = st.total_s;
            esp_err_t err = save_locked();
            if (err != ESP_OK) {
                ESP_LOGW(TAG, "could not save: %s", esp_err_to_name(err));
            }
        }
    }
}

esp_err_t bearing_start(void)
{
    trace_lock = xSemaphoreCreateMutex();
    if (!trace_lock) {
        return ESP_ERR_NO_MEM;
    }
    load();
    // wmm_field alone takes 2 kB in one frame, so this is not generous.
    if (xTaskCreate(bearing_task, "bearing", 6144, NULL, 4, &task) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "watching for straight runs above 3 m/s");
    return ESP_OK;
}

void bearing_get(bearing_status_t *out)
{
    taskENTER_CRITICAL(&lock);
    out->result = latest;
    taskEXIT_CRITICAL(&lock);
    out->learning = st.learning;
    out->applying = st.applying;
    out->have = out->result.have;
    out->offset_deg = out->result.offset_deg;
    out->total_hours = st.total_s / 3600.0f;
    out->trace_used = st.trace_n;
    out->stack_free = task ? uxTaskGetStackHighWaterMark(task) : 0;
    out->fixes_seen = fixes_seen;
    out->fixes_usable = fixes_usable;
    memcpy(out->blocked, blocked, sizeof(out->blocked));
}

esp_err_t bearing_set_learning(bool on)
{
    st.learning = on;
    return save_locked();
}

esp_err_t bearing_set_applying(bool on)
{
    st.applying = on;
    return save_locked();
}

esp_err_t bearing_reset(void)
{
    bearing_cfg_t cfg;
    bearing_cal_defaults(&cfg);
    bool was_learning = st.learning, was_applying = st.applying;
    taskENTER_CRITICAL(&lock);
    memset(&latest, 0, sizeof(latest));
    taskEXIT_CRITICAL(&lock);
    xSemaphoreTake(trace_lock, portMAX_DELAY);
    memset(&st, 0, sizeof(st));
    st.version = NVS_VERSION;
    st.learning = was_learning;
    st.applying = was_applying;
    bearing_cal_init(&st.cal, &cfg);
    xSemaphoreGive(trace_lock);
    last_trace_s = last_save_s = 0.0f;
    fixes_seen = fixes_usable = 0;
    memset(blocked, 0, sizeof(blocked));
    return save_locked();
}

bool bearing_correct(float *heading_deg)
{
    if (!st.applying) {
        return false;
    }
    bearing_result_t r;
    taskENTER_CRITICAL(&lock);
    r = latest;
    taskEXIT_CRITICAL(&lock);
    if (!r.have) {
        return false;
    }
    *heading_deg = bearing_cal_correct(&r, *heading_deg);
    return true;
}
