#include "wifi.h"

#include <stdio.h>
#include <string.h>

#include "esp_check.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_netif_sntp.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "nvs.h"

#if __has_include("wifi_credentials.h")
#include "wifi_credentials.h"
#endif
#ifndef WIFI_STA_SSID
#define WIFI_STA_SSID ""
#endif
#ifndef WIFI_STA_PASSWORD
#define WIFI_STA_PASSWORD ""
#endif
_Static_assert(sizeof(WIFI_STA_SSID) <= 33, "WIFI_STA_SSID is longer than 32 characters");
_Static_assert(sizeof(WIFI_STA_PASSWORD) == 1 || (sizeof(WIFI_STA_PASSWORD) >= 9 && sizeof(WIFI_STA_PASSWORD) <= 64),
               "WIFI_STA_PASSWORD must be empty or 8-63 characters");

#define NVS_NAMESPACE "wifi"
#define RECONNECT_MS 3000
// Reconnect attempts scan other channels and disturb the access point, so
// they're spaced out while it's up.
#define RECONNECT_WITH_AP_MS 15000

ESP_EVENT_DEFINE_BASE(RGNSS_WIFI_EVENT);
enum { EV_FALLBACK };

static const char *TAG = "wifi";

// Set once in wifi_start().
static esp_netif_t *sta_netif;
static bool sta_configured;
static bool sta_saved;
static char sta_ssid[33];
static wifi_config_t sta_cfg;
static TimerHandle_t reconnect_timer;
static TimerHandle_t fallback_timer;

// Event loop task only.
static bool sta_connected;

// Picks the network saved from the web page, else the built-in one.
// False if there is neither.
static bool load_station(void)
{
    char pass[64];
    nvs_handle_t nvs;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs) == ESP_OK) {
        size_t ssid_len = sizeof(sta_ssid);
        size_t pass_len = sizeof(pass);
        sta_saved = nvs_get_str(nvs, "ssid", sta_ssid, &ssid_len) == ESP_OK &&
                    nvs_get_str(nvs, "pass", pass, &pass_len) == ESP_OK;
        nvs_close(nvs);
    }
    if (!sta_saved) {
        strlcpy(sta_ssid, WIFI_STA_SSID, sizeof(sta_ssid));
        strlcpy(pass, WIFI_STA_PASSWORD, sizeof(pass));
    }
    if (sta_ssid[0] == '\0') {
        return false;
    }

    // A 32-character SSID fills the field without a terminator.
    memcpy(sta_cfg.sta.ssid, sta_ssid, strlen(sta_ssid));
    memcpy(sta_cfg.sta.password, pass, strlen(pass));
    sta_cfg.sta.threshold.authmode = pass[0] ? WIFI_AUTH_WPA2_PSK : WIFI_AUTH_OPEN;
    return true;
}

static bool ap_active(void)
{
    wifi_mode_t mode;
    return esp_wifi_get_mode(&mode) == ESP_OK && (mode & WIFI_MODE_AP);
}

static esp_err_t ap_enable(void)
{
    wifi_config_t cfg = {
        .ap = {
            .ssid = CONFIG_RGNSS_AP_SSID,
            .ssid_len = strlen(CONFIG_RGNSS_AP_SSID),
            .channel = CONFIG_RGNSS_AP_CHANNEL,
            .max_connection = CONFIG_RGNSS_AP_MAX_CONN,
            .authmode = WIFI_AUTH_OPEN,
        },
    };
    ESP_RETURN_ON_ERROR(esp_wifi_set_mode(sta_configured ? WIFI_MODE_APSTA : WIFI_MODE_AP), TAG, "set mode");
    ESP_RETURN_ON_ERROR(esp_wifi_set_config(WIFI_IF_AP, &cfg), TAG, "set AP config");
    ESP_LOGI(TAG, "open access point \"%s\" up", CONFIG_RGNSS_AP_SSID);
    return ESP_OK;
}

static void on_wifi_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    switch (id) {
    // The timers only exist when a station network is configured, and only then
    // can these two events arrive.
    case WIFI_EVENT_STA_START:
        if (!sta_configured) {
            break;
        }
        ESP_LOGI(TAG, "joining \"%s\" (%s)", sta_ssid, sta_saved ? "set from the web page" : "built in");
        esp_wifi_connect();
        if (xTimerStart(fallback_timer, 0) != pdPASS) {
            ESP_LOGW(TAG, "could not arm the access point fallback");
        }
        break;
    case WIFI_EVENT_STA_DISCONNECTED: {
        if (!sta_configured) {
            break;
        }
        wifi_event_sta_disconnected_t *e = data;
        int delay_ms = ap_active() ? RECONNECT_WITH_AP_MS : RECONNECT_MS;
        ESP_LOGW(TAG, "not connected to \"%s\" (reason %d), retrying in %d s", sta_ssid, e->reason, delay_ms / 1000);
        sta_connected = false;
        if (xTimerChangePeriod(reconnect_timer, pdMS_TO_TICKS(delay_ms), 0) != pdPASS) {
            ESP_LOGE(TAG, "could not schedule the retry");
        }
        if (!ap_active() && !xTimerIsTimerActive(fallback_timer) && xTimerStart(fallback_timer, 0) != pdPASS) {
            ESP_LOGW(TAG, "could not arm the access point fallback");
        }
        break;
    }
    case WIFI_EVENT_AP_STACONNECTED: {
        wifi_event_ap_staconnected_t *e = data;
        ESP_LOGI(TAG, "client " MACSTR " joined the access point", MAC2STR(e->mac));
        break;
    }
    case WIFI_EVENT_AP_STADISCONNECTED: {
        wifi_event_ap_stadisconnected_t *e = data;
        ESP_LOGI(TAG, "client " MACSTR " left the access point", MAC2STR(e->mac));
        break;
    }
    }
}

// Network time, once there is a network. It is the one clock the GNSS
// receiver cannot lie about, so gnss.c checks the receiver's time against it.
static void start_time_sync(void)
{
    static bool started;
    if (started) {
        return;
    }
    esp_sntp_config_t cfg = ESP_NETIF_SNTP_DEFAULT_CONFIG("pool.ntp.org");
    esp_err_t err = esp_netif_sntp_init(&cfg);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "no network time: %s", esp_err_to_name(err));
        return;
    }
    started = true;
}

static void on_got_ip(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    ip_event_got_ip_t *e = data;
    ESP_LOGI(TAG, "connected to \"%s\", IP " IPSTR, sta_ssid, IP2STR(&e->ip_info.ip));
    sta_connected = true;
    xTimerStop(fallback_timer, 0);
    start_time_sync();
    if (ap_active()) {
        ESP_LOGI(TAG, "access point down");
        esp_wifi_set_mode(WIFI_MODE_STA);
    }
}

static void on_fallback(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (!sta_connected && !ap_active()) {
        ESP_LOGW(TAG, "no connection to \"%s\" for %d s", sta_ssid, CONFIG_RGNSS_STA_FALLBACK_S);
        ap_enable();
    }
}

static void reconnect_timer_cb(TimerHandle_t timer)
{
    esp_wifi_connect();
}

// Hands over to the event loop task, which owns the connection state.
static void fallback_timer_cb(TimerHandle_t timer)
{
    esp_event_post(RGNSS_WIFI_EVENT, EV_FALLBACK, NULL, 0, 0);
}

esp_err_t wifi_start(void)
{
    sta_configured = load_station();

    esp_netif_create_default_wifi_ap();
    sta_netif = esp_netif_create_default_wifi_sta();

    wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT();
    ESP_RETURN_ON_ERROR(esp_wifi_init(&init), TAG, "init");
    // Credentials live in our own NVS namespace, not the driver's.
    ESP_RETURN_ON_ERROR(esp_wifi_set_storage(WIFI_STORAGE_RAM), TAG, "set storage");
    ESP_RETURN_ON_ERROR(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, on_wifi_event, NULL),
                        TAG, "register WiFi handler");

    if (!sta_configured) {
        ESP_LOGI(TAG, "no station network configured");
        ESP_RETURN_ON_ERROR(ap_enable(), TAG, "enable AP");
        return esp_wifi_start();
    }

    ESP_RETURN_ON_ERROR(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, on_got_ip, NULL),
                        TAG, "register IP handler");
    ESP_RETURN_ON_ERROR(esp_event_handler_register(RGNSS_WIFI_EVENT, EV_FALLBACK, on_fallback, NULL),
                        TAG, "register fallback handler");
    reconnect_timer = xTimerCreate("wifi_reconnect", pdMS_TO_TICKS(RECONNECT_MS), pdFALSE, NULL, reconnect_timer_cb);
    fallback_timer = xTimerCreate("wifi_fallback", pdMS_TO_TICKS(CONFIG_RGNSS_STA_FALLBACK_S * 1000), pdFALSE,
                                  NULL, fallback_timer_cb);
    ESP_RETURN_ON_FALSE(reconnect_timer && fallback_timer, ESP_ERR_NO_MEM, TAG, "create timers");

    ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_STA), TAG, "set mode");
    ESP_RETURN_ON_ERROR(esp_wifi_set_config(WIFI_IF_STA, &sta_cfg), TAG, "set STA config");
    return esp_wifi_start();
}

esp_err_t wifi_save_station(const char *ssid, const char *password)
{
    size_t ssid_len = strlen(ssid);
    size_t pass_len = strlen(password);
    ESP_RETURN_ON_FALSE(ssid_len >= 1 && ssid_len <= 32, ESP_ERR_INVALID_ARG, TAG, "SSID must be 1-32 characters");
    ESP_RETURN_ON_FALSE(pass_len == 0 || (pass_len >= 8 && pass_len <= 63), ESP_ERR_INVALID_ARG,
                        TAG, "password must be empty or 8-63 characters");

    nvs_handle_t nvs;
    ESP_RETURN_ON_ERROR(nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs), TAG, "open NVS");
    esp_err_t err = nvs_set_str(nvs, "ssid", ssid);
    if (err == ESP_OK) {
        err = nvs_set_str(nvs, "pass", password);
    }
    if (err == ESP_OK) {
        err = nvs_commit(nvs);
    }
    nvs_close(nvs);
    return err;
}

esp_err_t wifi_forget_station(void)
{
    nvs_handle_t nvs;
    ESP_RETURN_ON_ERROR(nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs), TAG, "open NVS");
    esp_err_t err = nvs_erase_all(nvs);
    if (err == ESP_OK) {
        err = nvs_commit(nvs);
    }
    nvs_close(nvs);
    return err;
}

void wifi_get_status(wifi_status_t *out)
{
    memset(out, 0, sizeof(*out));
    out->ap_active = ap_active();
    strlcpy(out->ap_ssid, CONFIG_RGNSS_AP_SSID, sizeof(out->ap_ssid));
    out->sta_configured = sta_configured;
    out->sta_saved = sta_saved;
    strlcpy(out->sta_ssid, sta_ssid, sizeof(out->sta_ssid));

    wifi_ap_record_t ap;
    esp_netif_ip_info_t ip;
    if (sta_configured && esp_wifi_sta_get_ap_info(&ap) == ESP_OK &&
        esp_netif_get_ip_info(sta_netif, &ip) == ESP_OK && ip.ip.addr != 0) {
        out->sta_connected = true;
        out->sta_rssi = ap.rssi;
        snprintf(out->sta_ip, sizeof(out->sta_ip), IPSTR, IP2STR(&ip.ip));
    }
}
