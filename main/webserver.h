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
//
// Bearing. The page shows one switch for these, "Automatic adjustment", which
// turns learning and applying on together. Everything else here is deliberately
// not on the page: it is for servicing and diagnosis, not for the person using
// the device.
//   GET  /api/bearing       what driving has taught the compass: hours above the
//                           speed threshold, straight runs kept of those seen,
//                           directions covered, the offset and once/twice-round
//                           terms it would apply, how steady that estimate is
//                           (drift per hour, range, split between halves,
//                           residual), and what is blocking the fixes it rejects
//   GET  /api/bearing/log   the drive log as CSV, one row per straight run
//   POST /api/bearing       form-encoded, any of:
//                             learning=0|1  collect while driving
//                             applying=0|1  apply what has been learned
//                             reset=1       throw away the estimate and the log
//                             save=1        write the estimate to NVS now
//   POST /api/restart       restart the device
//   POST /api/wifi          ssid, password (form-encoded): save and reboot
//   POST /api/wifi/forget   drop the saved network and reboot
esp_err_t webserver_start(void);
