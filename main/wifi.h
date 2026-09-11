#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

typedef struct {
    bool ap_active;
    char ap_ssid[33];
    bool sta_configured;
    bool sta_saved;       // the network was set from the web page, not built in
    char sta_ssid[33];
    bool sta_connected;
    char sta_ip[16];
    int8_t sta_rssi;
} wifi_status_t;

// Starts WiFi. The station joins the network saved from the web page (NVS), or
// else the built-in WIFI_STA_SSID from wifi_credentials.h. An open access point
// comes up when there is neither, or when the station has been without a
// connection for CONFIG_RGNSS_STA_FALLBACK_S, and goes down once it connects.
// Requires NVS, esp_netif and the default event loop to be initialised first.
esp_err_t wifi_start(void);

// Saves station credentials to NVS; they take effect on the next boot.
// ESP_ERR_INVALID_ARG unless the SSID is 1-32 characters and the password
// empty or 8-63 characters.
esp_err_t wifi_save_station(const char *ssid, const char *password);

// Removes the saved station credentials, falling back to the built-in network
// on the next boot.
esp_err_t wifi_forget_station(void);

void wifi_get_status(wifi_status_t *out);
