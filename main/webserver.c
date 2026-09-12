#include "webserver.h"

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

    const esp_app_desc_t *app = esp_app_get_description();
    char built[40];
    snprintf(built, sizeof(built), "%s %s", app->date, app->time);
    json_open(&j, "system");
    json_int(&j, "uptime_s", esp_timer_get_time() / 1000000);
    json_str(&j, "reset", reset_reason());
    json_str(&j, "built", built);
    json_str(&j, "partition", esp_ota_get_running_partition()->label);
    json_int(&j, "heap_min", esp_get_minimum_free_heap_size());
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
    static char buf[4096];

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

    json_t j = {.buf = buf, .size = sizeof(buf)};
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
    // One httpd task serves every request, so these can be static.
    static float points[MAG_CAL_MAX_POINTS][3];
    static char buf[12288];

    mag_cal_status_t s;
    mag_cal_get_status(&s);
    size_t n = mag_cal_get_points(points, MAG_CAL_MAX_POINTS);

    json_t j = {.buf = buf, .size = sizeof(buf)};
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
    json_key(&j, "points");
    json_append(&j, "[");
    for (size_t i = 0; i < n; i++) {
        json_append(&j, "%s[%.1f,%.1f,%.1f]", i ? "," : "", points[i][0], points[i][1], points[i][2]);
    }
    json_append(&j, "]");
    json_close(&j);
    return send_json(req, &j, "calibration");
}

// ?fresh=1 discards the saved readings; otherwise the session continues from them.
static esp_err_t cal_start_post(httpd_req_t *req)
{
    char query[64];  // see mag_samples_get: truncation would silently mean "not fresh"
    char value[4];
    bool fresh = httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK &&
                 httpd_query_key_value(query, "fresh", value, sizeof(value)) == ESP_OK && value[0] == '1';
    if (mag_cal_begin(fresh) != ESP_OK) {
        return reply(req, HTTPD_400, "No magnetometer detected");
    }
    return reply(req, HTTPD_200, NULL);
}

// Readings from a CSV the user saved earlier: any line whose first three
// numbers parse is taken, so both our column layouts work and a header is
// simply skipped.
static esp_err_t cal_points_post(httpd_req_t *req)
{
    static float pts[MAG_CAL_MAX_POINTS][3];
    char chunk[512];
    char line[80];
    size_t line_len = 0, n = 0, got = 0;
    int timeouts = 0;

    while (got < req->content_len) {
        size_t want = req->content_len - got < sizeof(chunk) ? req->content_len - got : sizeof(chunk);
        int r = httpd_req_recv(req, chunk, want);
        if (r == HTTPD_SOCK_ERR_TIMEOUT) {
            if (++timeouts > 3) {
                return reply(req, HTTPD_400, "The upload stalled");
            }
            continue;
        }
        if (r <= 0) {
            return ESP_FAIL;
        }
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
            if (n < MAG_CAL_MAX_POINTS && k >= 3) {
                memcpy(pts[n++], v + k - 3, 3 * sizeof(float));
            }
            line_len = 0;
        }
    }
    if (n < 20) {
        return reply(req, HTTPD_400, "Too few readings in that file");
    }
    if (mag_cal_set_points(pts, n) != ESP_OK) {
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
    static char chunk[2048];
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
        size_t want = total - got < sizeof(chunk) ? total - got : sizeof(chunk);
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
        {.uri = "/api/cal/start", .method = HTTP_POST, .handler = cal_start_post},
        {.uri = "/api/cal/stop", .method = HTTP_POST, .handler = cal_stop_post},
        {.uri = "/api/cal/accept", .method = HTTP_POST, .handler = cal_accept_post},
        {.uri = "/api/cal/points", .method = HTTP_POST, .handler = cal_points_post},
        {.uri = "/api/heading", .method = HTTP_POST, .handler = heading_post},
        {.uri = "/api/ota", .method = HTTP_POST, .handler = ota_post},
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
