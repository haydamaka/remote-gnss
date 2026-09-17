#include "webserver.h"

#include "nmea_ble.h"

#include <ctype.h>
#include <math.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_check.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_app_desc.h"
#include "esp_ota_ops.h"

#include "bearing.h"
#include "gnss.h"
#include "mag_cal.h"
#include "magnetometer.h"
#include "ota.h"
#include "wifi.h"

static const char *TAG = "web";

extern const char index_html_start[] asm("_binary_index_html_start");
extern const char index_html_end[] asm("_binary_index_html_end");

static httpd_handle_t server;

// Minimal JSON writer into a fixed buffer. len >= size afterwards means it overflowed.
typedef struct {
    char *buf;
    size_t size;
    size_t len;
    bool comma;
} json_t;

static void json_append(json_t *j, const char *fmt, ...)
{
    if (j->len >= j->size) {
        return;
    }
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(j->buf + j->len, j->size - j->len, fmt, ap);
    va_end(ap);
    j->len += n > 0 ? n : 0;
}

static void json_key(json_t *j, const char *key)
{
    json_append(j, "%s\"%s\":", j->comma ? "," : "", key);
    j->comma = true;
}

static void json_open(json_t *j, const char *key)
{
    if (key) {
        json_key(j, key);
    }
    json_append(j, "{");
    j->comma = false;
}

static void json_close(json_t *j)
{
    json_append(j, "}");
    j->comma = true;
}

// NaN and infinity become null.
static void json_num(json_t *j, const char *key, double v, int decimals)
{
    json_key(j, key);
    if (isfinite(v)) {
        json_append(j, "%.*f", decimals, v);
    } else {
        json_append(j, "null");
    }
}

static void json_int(json_t *j, const char *key, long long v)
{
    json_key(j, key);
    json_append(j, "%lld", v);
}

static void json_bool(json_t *j, const char *key, bool v)
{
    json_key(j, key);
    json_append(j, v ? "true" : "false");
}

// NULL becomes null.
static void json_str(json_t *j, const char *key, const char *s)
{
    json_key(j, key);
    if (!s) {
        json_append(j, "null");
        return;
    }
    json_append(j, "\"");
    for (; *s; s++) {
        unsigned char c = *s;
        if (c == '"' || c == '\\') {
            json_append(j, "\\%c", c);
        } else if (c < 0x20) {
            json_append(j, "\\u%04x", c);
        } else {
            json_append(j, "%c", c);
        }
    }
    json_append(j, "\"");
}

// Scratch for whichever request is being served. The HTTP server dispatches
// every handler from one task, so no two of them are ever live at once, and a
// separate buffer per handler would mean carrying all of them for ever.
static char req_scratch[4096];

// Sends a built document, or 500 if it didn't fit its buffer.
static esp_err_t send_json(httpd_req_t *req, const json_t *j, const char *what)
{
    if (j->len >= j->size) {
        ESP_LOGE(TAG, "%s needs %u bytes, buffer is %u", what, (unsigned)j->len + 1, (unsigned)j->size);
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "response too large");
    }
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, j->buf, j->len);
}

// Milliseconds since an esp_timer timestamp; NaN if it was never set.
static double age_ms(int64_t us)
{
    return us ? (esp_timer_get_time() - us) / 1000.0 : NAN;
}

static esp_err_t reply(httpd_req_t *req, const char *status, const char *error)
{
    char buf[200];
    json_t j = {.buf = buf, .size = sizeof(buf)};
    json_open(&j, NULL);
    json_bool(&j, "ok", error == NULL);
    if (error) {
        json_str(&j, "error", error);
    }
    json_close(&j);
    httpd_resp_set_status(req, status);
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, buf, j.len < j.size ? j.len : j.size - 1);
}

static esp_err_t read_body(httpd_req_t *req, char *buf, size_t size)
{
    if (req->content_len >= size) {
        return ESP_ERR_INVALID_SIZE;
    }
    size_t got = 0;
    int timeouts = 0;
    while (got < req->content_len) {
        int n = httpd_req_recv(req, buf + got, req->content_len - got);
        if (n == HTTPD_SOCK_ERR_TIMEOUT) {
            if (++timeouts > 3) {
                return ESP_ERR_TIMEOUT;  // a silent client must not hold the only server task
            }
            continue;
        }
        if (n <= 0) {
            return ESP_FAIL;
        }
        got += n;
    }
    buf[got] = '\0';
    return ESP_OK;
}

// Decodes an application/x-www-form-urlencoded value in place.
static void url_decode(char *s)
{
    char *out = s;
    for (; *s; s++) {
        if (*s == '+') {
            *out++ = ' ';
        } else if (*s == '%' && isxdigit((unsigned char)s[1]) && isxdigit((unsigned char)s[2])) {
            char hex[3] = {s[1], s[2], '\0'};
            *out++ = (char)strtol(hex, NULL, 16);
            s += 2;
        } else {
            *out++ = *s;
        }
    }
    *out = '\0';
}

// Called after the reply is sent; the pause lets it reach the browser before WiFi goes down.
static void restart_soon(void)
{
    vTaskDelay(pdMS_TO_TICKS(500));
    esp_restart();
}

static const char *reset_reason(void)
{
    switch (esp_reset_reason()) {
    case ESP_RST_POWERON: return "power-on";
    case ESP_RST_SW: return "restart by the firmware";
    case ESP_RST_PANIC: return "crash";
    case ESP_RST_INT_WDT:
    case ESP_RST_TASK_WDT:
    case ESP_RST_WDT: return "watchdog";
    case ESP_RST_BROWNOUT: return "brownout (power dip)";
    case ESP_RST_USB: return "USB reset";
    case ESP_RST_EXT: return "reset pin";
    case ESP_RST_DEEPSLEEP: return "wake from deep sleep";
    default: return "unknown";
    }
}

static esp_err_t index_get(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, index_html_start, index_html_end - index_html_start);
}

static esp_err_t status_get(httpd_req_t *req)
{
    gnss_data_t g;
    magnetometer_data_t m;
    wifi_status_t w;
    gnss_get(&g);
    magnetometer_get(&m);
    wifi_get_status(&w);

    char buf[1536];
    json_t j = {.buf = buf, .size = sizeof(buf)};
    json_open(&j, NULL);

    json_open(&j, "gnss");
    json_int(&j, "sentences", g.sentences);
    json_int(&j, "baud", g.baud);
    json_num(&j, "age_ms", age_ms(g.updated_us), 0);
    json_bool(&j, "fix", g.fix);
    json_int(&j, "quality", g.fix_quality);
    json_int(&j, "sats", g.satellites);
    json_num(&j, "lat", g.latitude, 7);
    json_num(&j, "lon", g.longitude, 7);
    json_num(&j, "alt", g.altitude_m, 1);
    json_num(&j, "hdop", g.hdop, 2);
    json_num(&j, "speed_kmh", g.speed_kmh, 1);
    json_num(&j, "course", g.course_deg, 1);
    json_str(&j, "utc", g.utc[0] ? g.utc : NULL);
    json_int(&j, "spoof", g.spoof);
    json_bool(&j, "implausible", g.implausible);
    json_str(&j, "implausible_why", g.implausible_why[0] ? g.implausible_why : NULL);
    json_close(&j);

    float field[3];
    bool calibrated = mag_cal_apply(m.field_ut, field);
    json_open(&j, "mag");
    json_str(&j, "chip", m.chip);
    json_num(&j, "age_ms", age_ms(m.updated_us), 0);
    json_bool(&j, "calibrated", calibrated);
    json_num(&j, "x", m.updated_us ? field[0] : NAN, 2);
    json_num(&j, "y", m.updated_us ? field[1] : NAN, 2);
    json_num(&j, "z", m.updated_us ? field[2] : NAN, 2);
    json_num(&j, "heading_offset", magnetometer_heading_offset(), 1);
    json_open(&j, "radio");
    json_int(&j, "clean", m.samples_clean);
    json_int(&j, "corrected", m.samples_corrected);
    json_int(&j, "skipped", m.samples_skipped);
    json_key(&j, "offset");
    if (m.radio_offset_learned) {
        json_append(&j, "[%.1f,%.1f,%.1f]", m.radio_offset_ut[0], m.radio_offset_ut[1], m.radio_offset_ut[2]);
    } else {
        json_append(&j, "null");
    }
    json_close(&j);
    // What driving has taught the compass, but only once the user has asked for
    // it to be used. The page applies it the same way it applies the manual
    // offset, to both the readout and the card.
    bearing_status_t bs;
    bearing_get(&bs);
    if (bs.applying && bs.have) {
        json_key(&j, "heading_coef");
        json_append(&j, "[");
        for (int i = 0; i < 5; i++) {
            json_append(&j, "%s%.3f", i ? "," : "", bs.result.coef[i]);
        }
        json_append(&j, "]");
    } else {
        json_str(&j, "heading_coef", NULL);
    }
    json_close(&j);

    json_open(&j, "wifi");
    json_bool(&j, "ap", w.ap_active);
    json_str(&j, "ap_ssid", w.ap_ssid);
    json_bool(&j, "sta_configured", w.sta_configured);
    json_bool(&j, "sta_saved", w.sta_saved);
    json_str(&j, "sta_ssid", w.sta_ssid);
    json_bool(&j, "sta_connected", w.sta_connected);
    json_str(&j, "sta_ip", w.sta_ip);
    json_int(&j, "rssi", w.sta_rssi);
    json_str(&j, "hostname", CONFIG_RGNSS_HOSTNAME ".local");
    json_int(&j, "fallback_s", CONFIG_RGNSS_STA_FALLBACK_S);
    json_close(&j);

    json_open(&j, "ble");
#if CONFIG_RGNSS_BLE_ENABLED
    nmea_ble_status_t b;
    nmea_ble_status(&b);
    json_bool(&j, "available", true);
    json_bool(&j, "preferred", b.preferred);
    json_bool(&j, "connected", b.connected);
    json_int(&j, "idle_s", CONFIG_RGNSS_BLE_IDLE_S);
#else
    json_bool(&j, "available", false);
#endif
    json_close(&j);

    const esp_app_desc_t *app = esp_app_get_description();
    char built[40];
    snprintf(built, sizeof(built), "%s %s", app->date, app->time);
    json_open(&j, "system");
    json_int(&j, "uptime_s", esp_timer_get_time() / 1000000);
    json_str(&j, "reset", reset_reason());
    json_str(&j, "built", built);
    json_str(&j, "partition", esp_ota_get_running_partition()->label);
    json_int(&j, "heap_min", esp_get_minimum_free_heap_size());
    json_int(&j, "heap_free", esp_get_free_heap_size());
    json_int(&j, "cal_stack_free", mag_cal_stack_free());
    json_close(&j);

    json_close(&j);
    return send_json(req, &j, "status");
}

#define MAG_SAMPLES_PER_REPLY 80

static esp_err_t mag_samples_get(httpd_req_t *req)
{
    // One httpd task serves every request, so these can be static.
    static magnetometer_sample_t samples[MAG_SAMPLES_PER_REPLY];
    char *buf = req_scratch;

    uint32_t since = 0;
    char query[64];  // a short buffer would turn extra parameters into "no since at all"
    char value[16];
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK &&
        httpd_query_key_value(query, "since", value, sizeof(value)) == ESP_OK) {
        since = strtoul(value, NULL, 10);
    }
    uint32_t latest;
    size_t n = magnetometer_samples_since(since, samples, MAG_SAMPLES_PER_REPLY, &latest);
    magnetometer_data_t m;
    magnetometer_get(&m);

    json_t j = {.buf = buf, .size = sizeof(req_scratch)};
    json_open(&j, NULL);
    json_str(&j, "chip", m.chip);
    json_num(&j, "ut_per_count", magnetometer_ut_per_count(), 6);
    json_int(&j, "seq", latest);
    json_bool(&j, "more", n > 0 && samples[n - 1].seq < latest);
    json_key(&j, "samples");
    json_append(&j, "[");
    for (size_t i = 0; i < n; i++) {
        json_append(&j, "%s[%lu,%lu,%d,%d,%d]", i ? "," : "", (unsigned long)samples[i].seq,
                    (unsigned long)samples[i].t_ms, samples[i].raw[0], samples[i].raw[1], samples[i].raw[2]);
    }
    json_append(&j, "]");
    json_close(&j);
    return send_json(req, &j, "samples");
}

static void json_floats(json_t *j, const char *key, const float *v, size_t n, int decimals)
{
    json_key(j, key);
    json_append(j, "[");
    for (size_t i = 0; i < n; i++) {
        json_append(j, "%s", i ? "," : "");
        if (isfinite(v[i])) {
            json_append(j, "%.*f", decimals, v[i]);
        } else {
            json_append(j, "null");  // "nan" would not be JSON
        }
    }
    json_append(j, "]");
}

static esp_err_t cal_get(httpd_req_t *req)
{
    static const char *const STATES[] = {"idle", "running", "done", "stopped", "failed"};
    static const char *const CAPS[] = {"+X", "-X", "+Y", "-Y", "+Z", "-Z"};
    // The readings are not in here: they are several kilobytes, they change
    // only while calibrating, and the compass view never draws them. The page
    // fetches them from GET /api/cal/points when points_seq says they moved.
    char *buf = req_scratch;

    mag_cal_status_t s;
    mag_cal_get_status(&s);

    json_t j = {.buf = buf, .size = sizeof(req_scratch)};
    json_open(&j, NULL);
    json_str(&j, "state", STATES[s.state]);
    json_str(&j, "reason", s.reason[0] ? s.reason : NULL);
    json_str(&j, "blocking", s.blocking[0] ? s.blocking : NULL);
    json_bool(&j, "can_accept", s.can_accept);
    json_str(&j, "message", s.message[0] ? s.message : NULL);
    json_int(&j, "refused", s.refused);
    json_num(&j, "progress", s.progress, 3);
    json_num(&j, "coverage", s.coverage, 3);
    json_key(&j, "missing");
    json_append(&j, "[");
    for (int k = 0, first = 1; k < 6; k++) {
        if (s.missing & (1 << k)) {
            json_append(&j, "%s\"%s\"", first ? "" : ",", CAPS[k]);
            first = 0;
        }
    }
    json_append(&j, "]");
    json_floats(&j, "centre", s.centre, 3, 2);
    if (s.params.valid) {
        json_open(&j, "cal");
        json_floats(&j, "offset", s.params.offset, 3, 2);
        json_floats(&j, "matrix", s.params.matrix, 9, 4);
        json_num(&j, "radius", s.params.radius, 2);
        json_num(&j, "rms", s.params.fit_rms, 4);
        json_str(&j, "quality", s.params.quality == MAG_CAL_GOOD ? "good"
                                : s.params.quality == MAG_CAL_ROUGH ? "rough" : "poor");
        json_int(&j, "points", s.params.points);
        json_close(&j);
    } else {
        json_str(&j, "cal", NULL);
    }
    if (s.have_fit) {
        json_open(&j, "fit");
        json_num(&j, "radius", s.fit.radius, 2);
        json_num(&j, "rms", s.fit.rms, 4);
        json_int(&j, "level", s.fit.level);
        json_int(&j, "inliers", s.fit.inliers);
        json_int(&j, "points", s.fit.points);
        json_int(&j, "cells", s.fit.cells);
        json_int(&j, "ms", s.fit_ms);
        json_close(&j);
    } else {
        json_str(&j, "fit", NULL);
    }
    json_int(&j, "points", s.points);
    json_int(&j, "points_seq", s.points_seq);
    json_close(&j);
    return send_json(req, &j, "calibration");
}

// The readings themselves, streamed a line at a time so no copy of the buffer
// is needed anywhere. Same shape the page drew before: [[x,y,z],...].
static esp_err_t cal_points_get(httpd_req_t *req)
{
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    httpd_resp_send_chunk(req, "[", 1);
    float p[3];
    for (size_t i = 0; mag_cal_get_point(i, p); i++) {
        char one[48];
        int len = snprintf(one, sizeof(one), "%s[%.1f,%.1f,%.1f]", i ? "," : "", p[0], p[1], p[2]);
        httpd_resp_send_chunk(req, one, len);
    }
    httpd_resp_send_chunk(req, "]", 1);
    return httpd_resp_send_chunk(req, NULL, 0);
}

// ?fresh=1 discards the saved readings; otherwise the session continues from them.
static esp_err_t cal_start_post(httpd_req_t *req)
{
    char query[64];  // see mag_samples_get: truncation would silently mean "not fresh"
    char value[4];
    bool fresh = httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK &&
                 httpd_query_key_value(query, "fresh", value, sizeof(value)) == ESP_OK && value[0] == '1';
    esp_err_t err = mag_cal_begin(fresh);
    if (err == ESP_ERR_NO_MEM) {
        return reply(req, HTTPD_500, "Not enough memory free to calibrate; try switching Bluetooth off");
    }
    if (err != ESP_OK) {
        return reply(req, HTTPD_400, "No magnetometer detected");
    }
    return reply(req, HTTPD_200, NULL);
}

// Readings from a CSV the user saved earlier: any line whose first three
// numbers parse is taken, so both our column layouts work and a header is
// simply skipped.
static esp_err_t cal_points_post(httpd_req_t *req)
{
    // Parsed straight into the calibration scratch: a 4.8 kB buffer of our own
    // would sit here for the life of the device to serve an occasional upload.
    // Bounded so a client cannot hold the calibration scratch for minutes:
    // the 400 readings the buffer holds are about 10 kB of CSV.
    if (req->content_len > 64 * 1024) {
        return reply(req, HTTPD_400, "That file is far too large to be readings");
    }
    size_t max = 0;
    float (*pts)[3] = mag_cal_load_buffer(&max);
    if (!pts) {
        return reply(req, HTTPD_400, "Busy fitting; try again in a moment");
    }
    char chunk[512];
    char line[80];
    size_t line_len = 0, n = 0, got = 0;
    int timeouts = 0;

    while (got < req->content_len) {
        size_t want = req->content_len - got < sizeof(chunk) ? req->content_len - got : sizeof(chunk);
        int r = httpd_req_recv(req, chunk, want);
        if (r == HTTPD_SOCK_ERR_TIMEOUT) {
            if (++timeouts > 3) {
                mag_cal_load_abandon();
                return reply(req, HTTPD_400, "The upload stalled");
            }
            continue;
        }
        if (r <= 0) {
            mag_cal_load_abandon();
            return ESP_FAIL;
        }
        timeouts = 0;  // it is moving again
        got += r;
        for (int i = 0; i < r; i++) {
            char ch = chunk[i];
            if (ch != '\n' && ch != '\r') {
                if (line_len < sizeof(line) - 1) {
                    line[line_len++] = ch;
                }
                continue;
            }
            line[line_len] = '\0';
            float v[6];
            int k = 0;
            for (char *p = line; k < 6; k++) {
                char *end;
                v[k] = strtof(p, &end);
                if (end == p) {
                    break;
                }
                for (p = end; *p == ',' || *p == ';' || *p == ' ' || *p == '\t'; p++) {
                }
            }
            // The last three numbers on the line are x, y, z: that takes both a bare
            // x,y,z file and our recordings, which are seq,t_ms,x,y,z.
            if (n < max && k >= 3) {
                memcpy(pts[n++], v + k - 3, 3 * sizeof(float));
            }
            line_len = 0;
        }
    }
    if (n < 20) {
        mag_cal_load_abandon();
        return reply(req, HTTPD_400, "Too few readings in that file");
    }
    if (mag_cal_set_points(pts, n) != ESP_OK) {
        mag_cal_load_abandon();
        return reply(req, HTTPD_500, "Could not load the readings");
    }
    return reply(req, HTTPD_200, NULL);
}

// Saves a fit that is usable but below the quality bar, labelled as rough.
static esp_err_t cal_accept_post(httpd_req_t *req)
{
    if (mag_cal_accept() != ESP_OK) {
        return reply(req, HTTPD_400, "The fit isn't good enough to save yet");
    }
    return reply(req, HTTPD_200, NULL);
}

// What has been learned from driving, and how much it can be trusted.
static esp_err_t bearing_get_handler(httpd_req_t *req)
{
    bearing_status_t b;
    bearing_get(&b);
    const bearing_quality_t *q = &b.result.quality;

    char buf[1400];
    json_t j = {.buf = buf, .size = sizeof(buf)};
    json_open(&j, NULL);
    json_bool(&j, "learning", b.learning);
    json_bool(&j, "applying", b.applying);
    json_bool(&j, "have", b.have);
    json_num(&j, "offset", b.offset_deg, 2);
    json_num(&j, "hours", b.total_hours, 3);
    json_int(&j, "windows", b.result.windows);
    json_int(&j, "sectors", b.result.sectors);
    json_int(&j, "terms", b.result.terms);
    json_int(&j, "remounts", (int)b.result.remounts);
    json_int(&j, "trace", (int)b.trace_used);
    json_num(&j, "once_round", b.result.once_round_deg, 2);
    json_num(&j, "twice_round", b.result.twice_round_deg, 2);
    json_num(&j, "residual_hard_ut", b.result.residual_hard_ut, 3);
    json_num(&j, "soft_asym_pct", b.result.soft_asym_pct, 2);

    json_open(&j, "coef");
    static const char *names[5] = {"a", "b", "c", "d", "e"};
    for (int i = 0; i < 5; i++) {
        json_num(&j, names[i], b.result.coef[i], 3);
    }
    json_close(&j);

    // The numbers a drive is meant to produce: none of them come from the
    // fit's own standard error, which is far too pleased with itself.
    json_open(&j, "quality");
    json_num(&j, "resid_rms", q->resid_rms_deg, 3);
    json_num(&j, "resid_bias", q->resid_bias_deg, 3);
    json_num(&j, "resid_max", q->resid_max_deg, 2);
    json_int(&j, "resid_n", (int)q->resid_n);
    json_num(&j, "split", q->split_deg, 3);
    json_num(&j, "drift_per_hour", q->drift_deg_per_hour, 3);
    json_num(&j, "range", q->range_deg, 3);
    json_num(&j, "accept_pct", q->accept_pct, 1);
    json_int(&j, "windows_seen", (int)q->windows_seen);
    json_close(&j);

    json_int(&j, "stack_free", (int)b.stack_free);
    json_int(&j, "fixes_seen", (int)b.fixes_seen);
    json_int(&j, "fixes_usable", (int)b.fixes_usable);
    json_open(&j, "blocked");
    for (int i = 0; i < BEARING_BLOCKS; i++) {
        if (b.blocked[i]) {
            json_int(&j, bearing_block_name((bearing_block_t)i), (int)b.blocked[i]);
        }
    }
    json_close(&j);

    json_open(&j, "rejected");
    for (int i = 1; i < BEARING_REASONS; i++) {
        if (b.result.rejected[i]) {
            json_int(&j, bearing_reject_name((bearing_reject_t)i), (int)b.result.rejected[i]);
        }
    }
    json_close(&j);

    json_close(&j);
    return send_json(req, &j, "bearing");
}

// The whole drive, a row a minute, for working out offline whether it settled.
static esp_err_t bearing_log_get(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/csv");
    httpd_resp_set_hdr(req, "Content-Disposition", "attachment; filename=\"bearing.csv\"");
    const char *head = "t_s,offset_deg,resid_rms_deg,split_deg,sectors,terms,accept_pct,remounts\n";
    httpd_resp_send_chunk(req, head, HTTPD_RESP_USE_STRLEN);
    // A record at a time: there is no RAM to spare for a second copy of the ring.
    bearing_trace_t r;
    for (size_t i = 0; bearing_trace_at(i, &r); i++) {
        char line[96];
        int len = snprintf(line, sizeof(line), "%u,%.2f,%.2f,%.2f,%u,%u,%u,%u\n",
                           (unsigned)r.t_s, r.offset_cdeg / 100.0f,
                           r.resid_rms_cdeg / 100.0f, r.split_cdeg / 100.0f,
                           r.sectors, r.terms, r.accept_pct, r.remounts);
        httpd_resp_send_chunk(req, line, len);
    }
    return httpd_resp_send_chunk(req, NULL, 0);
}

static esp_err_t bearing_post(httpd_req_t *req)
{
    char body[96], value[8];
    if (read_body(req, body, sizeof(body)) != ESP_OK) {
        return reply(req, HTTPD_400, "The request was too long");
    }
    esp_err_t err = ESP_OK;
    bool did = false;
    if (httpd_query_key_value(body, "reset", value, sizeof(value)) == ESP_OK && value[0] == '1') {
        err = bearing_reset();
        did = true;
    }
    if (err == ESP_OK && httpd_query_key_value(body, "learning", value, sizeof(value)) == ESP_OK) {
        err = bearing_set_learning(value[0] == '1');
        did = true;
    }
    if (err == ESP_OK && httpd_query_key_value(body, "applying", value, sizeof(value)) == ESP_OK) {
        err = bearing_set_applying(value[0] == '1');
        did = true;
    }
    if (err == ESP_OK && httpd_query_key_value(body, "save", value, sizeof(value)) == ESP_OK) {
        err = bearing_save();
        did = true;
    }
    if (!did) {
        return reply(req, HTTPD_400, "Nothing to do");
    }
    if (err != ESP_OK) {
        return reply(req, HTTPD_500, "Could not save the setting");
    }
    return reply(req, HTTPD_200, NULL);
}

// Turns the compass card so it reads north where the user says north is.
static esp_err_t heading_post(httpd_req_t *req)
{
    char body[64], value[16];
    if (read_body(req, body, sizeof(body)) != ESP_OK) {
        return reply(req, HTTPD_400, "The request was too long");
    }
    if (httpd_query_key_value(body, "offset", value, sizeof(value)) != ESP_OK) {
        return reply(req, HTTPD_400, "No offset given");
    }
    char *end;
    float degrees = strtof(value, &end);
    if (end == value || !isfinite(degrees)) {
        return reply(req, HTTPD_400, "The offset isn't a number");
    }
    if (magnetometer_set_heading_offset(degrees) != ESP_OK) {
        return reply(req, HTTPD_500, "Couldn't save the offset");
    }
    return reply(req, HTTPD_200, NULL);
}

static esp_err_t cal_stop_post(httpd_req_t *req)
{
    mag_cal_stop();
    return reply(req, HTTPD_200, NULL);
}

// Firmware update over WiFi: POST the .bin. The image is checked before the
// bootloader is pointed at it, and a bad one leaves the running firmware alone.
static esp_err_t ota_post(httpd_req_t *req)
{
    char *chunk = req_scratch;
    size_t total = req->content_len;
    if (total < 4096) {
        return reply(req, HTTPD_400, "Too small to be a firmware image");
    }
    if (ota_begin(total) != ESP_OK) {
        return reply(req, HTTPD_500, "Could not start the update");
    }
    size_t got = 0;
    int timeouts = 0;
    while (got < total) {
        size_t want = total - got < sizeof(req_scratch) ? total - got : sizeof(req_scratch);
        int n = httpd_req_recv(req, chunk, want);
        if (n == HTTPD_SOCK_ERR_TIMEOUT) {
            if (++timeouts > 5) {
                ota_abort();
                return reply(req, HTTPD_400, "The upload stalled");
            }
            continue;
        }
        if (n <= 0) {
            ota_abort();
            return ESP_FAIL;
        }
        if (got == 0 && (unsigned char)chunk[0] != 0xE9) {  // ESP image magic
            ota_abort();
            return reply(req, HTTPD_400, "Not an ESP32 firmware image");
        }
        if (ota_write(chunk, n) != ESP_OK) {
            ota_abort();
            return reply(req, HTTPD_500, "Could not write the image");
        }
        timeouts = 0;
        got += n;
    }
    if (ota_finish() != ESP_OK) {
        return reply(req, HTTPD_400, "The image was rejected");
    }
    ESP_LOGI(TAG, "update installed, restarting");
    reply(req, HTTPD_200, NULL);
    restart_soon();
    return ESP_OK;
}

// Hands the one radio between WiFi and BLE. Switching to BLE stops WiFi, so
// the reply goes out before the change is made -- it is the last thing this
// client will hear. Nothing is saved: a restart comes back on WiFi, and so
// does the device by itself once no BLE client has been connected for
// CONFIG_RGNSS_BLE_IDLE_S.
static esp_err_t ble_post(httpd_req_t *req)
{
#if CONFIG_RGNSS_BLE_ENABLED
    char body[8] = "";
    int len = req->content_len < sizeof(body) - 1 ? req->content_len : (int)sizeof(body) - 1;
    if (len > 0 && httpd_req_recv(req, body, len) <= 0) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "no body");
    }
    body[len > 0 ? len : 0] = '\0';
    bool on = strcmp(body, "on") == 0;
    if (!on && strcmp(body, "off") != 0) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "body must be \"on\" or \"off\"");
    }

    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, on ? "{\"ok\":true,\"wifi\":\"stopping\"}" : "{\"ok\":true}");
    nmea_ble_set_enabled(on);
    return ESP_OK;
#else
    return httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "this firmware was built without BLE");
#endif
}

static esp_err_t restart_post(httpd_req_t *req)
{
    ESP_LOGI(TAG, "restart requested");
    reply(req, HTTPD_200, NULL);
    restart_soon();
    return ESP_OK;
}

static esp_err_t wifi_post(httpd_req_t *req)
{
    char body[512];
    char ssid[200];
    char password[200];
    if (read_body(req, body, sizeof(body)) != ESP_OK) {
        return reply(req, HTTPD_400, "Could not read the request");
    }
    if (httpd_query_key_value(body, "ssid", ssid, sizeof(ssid)) != ESP_OK) {
        return reply(req, HTTPD_400, "Network name missing");
    }
    esp_err_t err = httpd_query_key_value(body, "password", password, sizeof(password));
    if (err == ESP_ERR_NOT_FOUND) {
        password[0] = '\0';
    } else if (err != ESP_OK) {
        return reply(req, HTTPD_400, "Password too long");
    }
    url_decode(ssid);
    url_decode(password);

    err = wifi_save_station(ssid, password);
    if (err == ESP_ERR_INVALID_ARG) {
        return reply(req, HTTPD_400, "The network name must be 1-32 characters, the password empty or 8-63");
    }
    if (err != ESP_OK) {
        return reply(req, HTTPD_500, "Could not save the settings");
    }
    ESP_LOGI(TAG, "station network set to \"%s\", restarting", ssid);
    reply(req, HTTPD_200, NULL);
    restart_soon();
    return ESP_OK;
}

static esp_err_t forget_post(httpd_req_t *req)
{
    if (wifi_forget_station() != ESP_OK) {
        return reply(req, HTTPD_500, "Could not clear the settings");
    }
    ESP_LOGI(TAG, "station network forgotten, restarting");
    reply(req, HTTPD_200, NULL);
    restart_soon();
    return ESP_OK;
}

esp_err_t webserver_start(void)
{
    static const httpd_uri_t uris[] = {
        {.uri = "/", .method = HTTP_GET, .handler = index_get},
        {.uri = "/api/status", .method = HTTP_GET, .handler = status_get},
        {.uri = "/api/mag/samples", .method = HTTP_GET, .handler = mag_samples_get},
        {.uri = "/api/cal", .method = HTTP_GET, .handler = cal_get},
        {.uri = "/api/cal/points", .method = HTTP_GET, .handler = cal_points_get},
        {.uri = "/api/cal/start", .method = HTTP_POST, .handler = cal_start_post},
        {.uri = "/api/cal/stop", .method = HTTP_POST, .handler = cal_stop_post},
        {.uri = "/api/cal/accept", .method = HTTP_POST, .handler = cal_accept_post},
        {.uri = "/api/cal/points", .method = HTTP_POST, .handler = cal_points_post},
        {.uri = "/api/heading", .method = HTTP_POST, .handler = heading_post},
        {.uri = "/api/bearing", .method = HTTP_GET, .handler = bearing_get_handler},
        {.uri = "/api/bearing/log", .method = HTTP_GET, .handler = bearing_log_get},
        {.uri = "/api/bearing", .method = HTTP_POST, .handler = bearing_post},
        {.uri = "/api/ota", .method = HTTP_POST, .handler = ota_post},
        {.uri = "/api/ble", .method = HTTP_POST, .handler = ble_post},
        {.uri = "/api/restart", .method = HTTP_POST, .handler = restart_post},
        {.uri = "/api/wifi", .method = HTTP_POST, .handler = wifi_post},
        {.uri = "/api/wifi/forget", .method = HTTP_POST, .handler = forget_post},
    };

    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.server_port = CONFIG_RGNSS_HTTP_PORT;
    cfg.stack_size = 6144;
    cfg.lru_purge_enable = true;
    // Sized from the table above, so adding an endpoint can't run it out of slots.
    cfg.max_uri_handlers = sizeof(uris) / sizeof(uris[0]);
    ESP_RETURN_ON_ERROR(httpd_start(&server, &cfg), TAG, "start");

    for (size_t i = 0; i < sizeof(uris) / sizeof(uris[0]); i++) {
        ESP_RETURN_ON_ERROR(httpd_register_uri_handler(server, &uris[i]), TAG, "register %s", uris[i].uri);
    }

    ESP_LOGI(TAG, "listening on port %d", cfg.server_port);
    return ESP_OK;
}
