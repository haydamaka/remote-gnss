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
// Whether this boot should run BLE instead of WiFi. Ask before starting either:
// the chip has one radio, and with both running the station could not hold a
// distant access point, so the device does one or the other.
//
// It says no when the last BLE session ended without a client, even if the
// saved preference says BLE -- that is what guarantees the web page comes back.
bool nmea_ble_boot_mode(void);

// Starts BLE. Only valid when nmea_ble_boot_mode() returned true, so that WiFi
// was never started and the radio is free.
esp_err_t nmea_ble_start(void);

// Chooses which radio the device comes up on, saves it, and restarts into it.
// Does not return.
//
// It cannot make the device unreachable: whatever the setting, a BLE session
// with nothing connected for CONFIG_RGNSS_BLE_IDLE_S restarts into WiFi, so
// the web page always comes back on its own.
esp_err_t nmea_ble_set_enabled(bool on);

typedef struct {
    bool enabled;      // BLE has the radio now, and WiFi is stopped
    bool preferred;    // and it is what the device starts with
    bool connected;
    bool subscribed;   // the client enabled notifications on the TX characteristic
    uint16_t mtu;      // negotiated ATT MTU; 23 until the client asks for more
    char peer[18];     // "aa:bb:cc:dd:ee:ff"
    uint32_t sentences;
    uint32_t bytes;
    uint32_t dropped;
} nmea_ble_status_t;

void nmea_ble_status(nmea_ble_status_t *out);
