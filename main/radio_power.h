#pragma once

#include "esp_err.h"

// Transmit power for each radio, set from the page and kept in NVS. More power
// reaches further and costs current; on this board's small antenna a lower
// WiFi power is sometimes the more reliable one.
//
// WiFi takes 2-20 dBm, applied at once; the driver rounds to its own steps.
// BLE takes 0 dBm up to RADIO_POWER_BLE_MAX_DBM in 3 dB steps (and 20 on the
// C3), applied when BLE starts: the page is only there on WiFi, so it is for
// the next boot on BLE.

#define RADIO_POWER_WIFI_DEFAULT_DBM 20.0f
#define RADIO_POWER_BLE_DEFAULT_DBM 9
#if CONFIG_IDF_TARGET_ESP32
#define RADIO_POWER_BLE_MAX_DBM 9
#else
#define RADIO_POWER_BLE_MAX_DBM 20
#endif

float radio_power_wifi_dbm(void);
int radio_power_ble_dbm(void);

// ESP_ERR_INVALID_ARG outside the ranges above.
esp_err_t radio_power_set_wifi_dbm(float dbm);
esp_err_t radio_power_set_ble_dbm(int dbm);

// Loads what was saved. Before either radio starts.
void radio_power_load(void);

// Apply the saved power: WiFi once the driver has started, BLE once the
// controller is up.
void radio_power_apply_wifi(void);
void radio_power_apply_ble(void);
