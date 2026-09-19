#include "radio_power.h"

#include <math.h>

#include "esp_bt.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "nvs.h"

#define NVS_NAMESPACE "radio"
#define NVS_KEY_WIFI "wifi_qdbm"  // quarter dBm, as the WiFi driver takes it
#define NVS_KEY_BLE "ble_dbm"

static const char *TAG = "radio_power";

static int8_t wifi_qdbm = (int8_t)(RADIO_POWER_WIFI_DEFAULT_DBM * 4);
static int8_t ble_dbm = RADIO_POWER_BLE_DEFAULT_DBM;

static bool wifi_ok(int q)
{
    return q >= 8 && q <= 80;
}

// The controller's levels: -24 dBm is level 0, then 3 dB a level, up to +18
// dBm at 14; level 15 is +20.
static bool ble_level(int dbm, esp_power_level_t *out)
{
    if (dbm == 20) {
        *out = ESP_PWR_LVL_P20;
        return true;
    }
    if (dbm < 0 || dbm > 18 || dbm % 3 != 0) {
        return false;
    }
    *out = (esp_power_level_t)((dbm + 24) / 3);
    return true;
}

float radio_power_wifi_dbm(void)
{
    return wifi_qdbm / 4.0f;
}

int radio_power_ble_dbm(void)
{
    return ble_dbm;
}

static esp_err_t save(const char *key, int8_t v)
{
    nvs_handle_t nvs;
    ESP_RETURN_ON_ERROR(nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs), TAG, "open NVS");
    esp_err_t err = nvs_set_i8(nvs, key, v);
    if (err == ESP_OK) {
        err = nvs_commit(nvs);
    }
    nvs_close(nvs);
    return err;
}

esp_err_t radio_power_set_wifi_dbm(float dbm)
{
    int q = (int)lroundf(dbm * 4);
    ESP_RETURN_ON_FALSE(isfinite(dbm) && wifi_ok(q), ESP_ERR_INVALID_ARG, TAG, "WiFi power %.2f dBm", dbm);
    ESP_RETURN_ON_ERROR(save(NVS_KEY_WIFI, (int8_t)q), TAG, "save WiFi power");
    wifi_qdbm = (int8_t)q;
    radio_power_apply_wifi();
    return ESP_OK;
}

esp_err_t radio_power_set_ble_dbm(int dbm)
{
    esp_power_level_t level;
    ESP_RETURN_ON_FALSE(ble_level(dbm, &level), ESP_ERR_INVALID_ARG, TAG, "BLE power %d dBm", dbm);
    ESP_RETURN_ON_ERROR(save(NVS_KEY_BLE, (int8_t)dbm), TAG, "save BLE power");
    ble_dbm = (int8_t)dbm;
    ESP_LOGI(TAG, "BLE at %d dBm from its next start", dbm);
    return ESP_OK;
}

void radio_power_load(void)
{
    nvs_handle_t nvs;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs) != ESP_OK) {
        return;
    }
    int8_t v;
    esp_power_level_t level;
    if (nvs_get_i8(nvs, NVS_KEY_WIFI, &v) == ESP_OK && wifi_ok(v)) {
        wifi_qdbm = v;
    }
    if (nvs_get_i8(nvs, NVS_KEY_BLE, &v) == ESP_OK && ble_level(v, &level)) {
        ble_dbm = v;
    }
    nvs_close(nvs);
}

void radio_power_apply_wifi(void)
{
    esp_err_t err = esp_wifi_set_max_tx_power(wifi_qdbm);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "WiFi at up to %.2f dBm", wifi_qdbm / 4.0f);
    } else if (err != ESP_ERR_WIFI_NOT_INIT && err != ESP_ERR_WIFI_NOT_STARTED) {
        ESP_LOGW(TAG, "set WiFi power: %s", esp_err_to_name(err));
    }
}

void radio_power_apply_ble(void)
{
    esp_power_level_t level;
    if (!ble_level(ble_dbm, &level)) {
        return;
    }
    // DEFAULT covers every state not set on its own, connections included.
    esp_err_t err = esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, level);
    if (err == ESP_OK) {
        err = esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, level);
    }
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "BLE at %d dBm", ble_dbm);
    } else {
        ESP_LOGW(TAG, "set BLE power: %s", esp_err_to_name(err));
    }
}
