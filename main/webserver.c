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

#include "gnss.h"
#include "magnetometer.h"
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
    return httpd_resp_send(req, buf, j.len);
}

static esp_err_t read_body(httpd_req_t *req, char *buf, size_t size)
{
    if (req->content_len >= size) {
        return ESP_ERR_INVALID_SIZE;
    }
    size_t got = 0;
    while (got < req->content_len) {
        int n = httpd_req_recv(req, buf + got, req->content_len - got);
        if (n == HTTPD_SOCK_ERR_TIMEOUT) {
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

    char buf[1024];
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
    json_close(&j);

    json_open(&j, "mag");
    json_str(&j, "chip", m.chip);
    json_num(&j, "age_ms", age_ms(m.updated_us), 0);
    json_num(&j, "x", m.updated_us ? m.field_ut[0] : NAN, 2);
    json_num(&j, "y", m.updated_us ? m.field_ut[1] : NAN, 2);
    json_num(&j, "z", m.updated_us ? m.field_ut[2] : NAN, 2);
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

    json_close(&j);
    if (j.len >= j.size) {
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "status too large");
    }
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, buf, j.len);
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
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.server_port = CONFIG_RGNSS_HTTP_PORT;
    cfg.stack_size = 6144;
    cfg.lru_purge_enable = true;
    ESP_RETURN_ON_ERROR(httpd_start(&server, &cfg), TAG, "start");

    static const httpd_uri_t uris[] = {
        {.uri = "/", .method = HTTP_GET, .handler = index_get},
        {.uri = "/api/status", .method = HTTP_GET, .handler = status_get},
        {.uri = "/api/wifi", .method = HTTP_POST, .handler = wifi_post},
        {.uri = "/api/wifi/forget", .method = HTTP_POST, .handler = forget_post},
    };
    for (size_t i = 0; i < sizeof(uris) / sizeof(uris[0]); i++) {
        ESP_RETURN_ON_ERROR(httpd_register_uri_handler(server, &uris[i]), TAG, "register %s", uris[i].uri);
    }

    ESP_LOGI(TAG, "listening on port %d", cfg.server_port);
    return ESP_OK;
}
