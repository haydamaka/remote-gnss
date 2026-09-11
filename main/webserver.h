#pragma once

#include "esp_err.h"

// Starts the HTTP server on menuconfig "Remote GNSS → Web server port".
// Serves the embedded html/index.html and its JSON API:
//   GET  /api/status        GNSS, magnetometer and WiFi state
//   POST /api/wifi          ssid, password (form-encoded): save and reboot
//   POST /api/wifi/forget   drop the saved network and reboot
esp_err_t webserver_start(void);
