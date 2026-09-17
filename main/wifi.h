#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

// Networks saved from the web page. Each is 97 bytes in NVS and in RAM.
#define WIFI_SAVED_MAX 5

// Nearby networks one scan reports, strongest first; the rest are dropped.
#define WIFI_SCAN_MAX 20

typedef struct {
    bool ap_active;
    char ap_ssid[33];
    bool sta_configured;  // there is at least one network it may join
    bool sta_saved;       // the network below was saved from the web page, not built in
    char sta_ssid[33];    // the network being joined or joined
    bool sta_connected;
    char sta_ip[16];
    int8_t sta_rssi;
} wifi_status_t;

typedef struct {
    char ssid[33];
    int8_t rssi;
    uint8_t channel;
    bool secure;  // needs a password
    bool known;   // saved, or the built-in one
} wifi_scan_entry_t;

// Starts WiFi. The station joins the strongest network in range among the ones
// saved from the web page (NVS) and the built-in WIFI_STA_SSID from
// wifi_credentials.h, moving on to the next when one fails. An open access
// point comes up when there is none to join, or when the station has been
// without a connection for CONFIG_RGNSS_STA_FALLBACK_S, and goes down once it
// connects. Requires NVS, esp_netif and the default event loop first.
esp_err_t wifi_start(void);

// Saves a network, or replaces the password of one already saved; no restart
// needed. ESP_ERR_INVALID_ARG unless the SSID is 1-32 characters and the
// password empty or 8-63 characters; ESP_ERR_NO_MEM when WIFI_SAVED_MAX other
// networks are saved already.
esp_err_t wifi_save_station(const char *ssid, const char *password);

// Forgets one saved network, or with NULL every one; leaves it if connected.
// ESP_ERR_NOT_FOUND if that network is not saved.
esp_err_t wifi_forget_station(const char *ssid);

// Names of the saved networks, newest first. Passwords never leave wifi.c.
size_t wifi_get_saved(char (*names)[33], size_t max);

// The network built into the firmware, or NULL if there is none.
const char *wifi_builtin_ssid(void);

// Scans for nearby networks, one entry per name, strongest first. Blocks for
// two or three seconds. ESP_ERR_INVALID_STATE while the station is busy joining
// a network or scanning for one; try again shortly.
esp_err_t wifi_scan(wifi_scan_entry_t *out, size_t max, size_t *n);

void wifi_get_status(wifi_status_t *out);
