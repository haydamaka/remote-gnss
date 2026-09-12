#pragma once

#include "esp_err.h"

// Starts the HTTP server on menuconfig "Remote GNSS → Web server port".
// Serves the embedded html/index.html and its JSON API:
//   GET  /api/status        GNSS, magnetometer and WiFi state
//   GET  /api/mag/samples   ?since=SEQ: recent raw magnetometer samples,
//                           [seq, t_ms, x, y, z] in counts; ut_per_count converts
//   GET  /api/cal           calibration state, progress, parameters and readings
//   POST /api/cal/start     start a calibration session; continues from the
//                           saved readings unless ?fresh=1
//   POST /api/cal/stop      interrupt it
//   POST /api/ota           a firmware .bin as the body: install it and restart
//   POST /api/restart       restart the device
//   POST /api/wifi          ssid, password (form-encoded): save and reboot
//   POST /api/wifi/forget   drop the saved network and reboot
esp_err_t webserver_start(void);
