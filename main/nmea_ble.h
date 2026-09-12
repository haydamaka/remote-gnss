#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

// Serves the NMEA stream over BLE as a Nordic UART Service (NUS), the
// de-facto serial-over-BLE profile that GNSS receivers use:
//
//   service 6E400001-B5A3-F393-E0A9-E50E24DCCA9E
//   TX      6E400003-...  notify, sentences out
//   RX      6E400002-...  write, discarded for now
//
// The ESP32-C3 has no Bluetooth Classic, so there is no SPP and no RFCOMM;
// a client has to speak NUS. On Linux, ble-serial binds it to a pty:
//   ble-serial -d <MAC>   then point gpsd at the /dev/pts it prints.
esp_err_t nmea_ble_start(void);

typedef struct {
    bool connected;
    bool subscribed;   // the client enabled notifications on the TX characteristic
    uint16_t mtu;      // negotiated ATT MTU; 23 until the client asks for more
    char peer[18];     // "aa:bb:cc:dd:ee:ff"
    uint32_t sentences;
    uint32_t bytes;
    uint32_t dropped;
} nmea_ble_status_t;

void nmea_ble_status(nmea_ble_status_t *out);
