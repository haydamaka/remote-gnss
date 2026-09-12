#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

// Serves the NMEA stream to TCP clients on menuconfig "Remote GNSS → NMEA
// output → TCP port" (10110, the registered NMEA-0183 port). gpsd takes it
// directly: gpsd -N -n tcp://gnss-haydamaka.local:10110
esp_err_t nmea_tcp_start(void);

typedef struct {
    char peer[24];        // "192.168.4.2:51234"
    uint32_t sentences;
    uint32_t bytes;
    uint32_t dropped;
    int64_t connected_us;  // esp_timer time the client arrived
} nmea_tcp_client_t;

// Copies the connected clients into out, and returns how many were written.
size_t nmea_tcp_clients(nmea_tcp_client_t *out, size_t max);
