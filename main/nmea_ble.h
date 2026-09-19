#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

// Serves the NMEA stream over BLE as a Nordic UART Service (NUS), the
// de-facto serial-over-BLE profile that GNSS receivers use:
//
//   service 6E400001-B5A3-F393-E0A9-E50E24DCCA9E
//   TX      6E400003-...  notify, sentences out
//   RX      6E400002-...  write, commands (below); anything else is ignored
//
// Commands are NMEA-style lines with a required checksum; lines not starting
// "$PPUCK," are ignored.
//   $PPUCK,WIFI,<SSID hex>,<password hex>*hh   save a network (wifi_save_station)
//   $PPUCK,RADIO,WIFI*hh                       restart on WiFi, whatever the switch
// The answer comes back on TX: $PPUCK,WIFI,OK, $PPUCK,WIFI,ERR,<FORMAT|INVALID|
// FULL|STORE>, $PPUCK,RADIO,OK, or $PPUCK,ERR,<CHECKSUM|UNKNOWN|BUSY|MEMORY>. A
// saved network is used on the next boot on WiFi, since until then the radio is
// BLE's. RADIO,WIFI holds for that one boot: a power cycle or a reset goes back
// to the switch, and so does flipping the switch either way.
//
// The ESP32-C3 has no Bluetooth Classic, so there is no SPP and no RFCOMM;
// a client has to speak NUS. On Linux, ble-serial binds it to a pty:
//   ble-serial -d <MAC>   then point gpsd at the /dev/pts it prints.
// Whether this boot should run BLE instead of WiFi, as the radio switch on
// CONFIG_RGNSS_RADIO_SWITCH_GPIO says: on is WiFi, off is BLE. Ask before
// starting either: the chip has one radio, and with both running the station
// could not hold a distant access point, so the device does one or the other.
// From then on it watches the switch, and restarts when it is flipped.
// Says WiFi, whatever the switch, on the boot after nmea_ble_restart_on_wifi()
// or a $PPUCK,RADIO,WIFI command.
bool nmea_ble_boot_mode(void);

// Restarts into one boot on WiFi, whatever the switch. For when BLE failed to
// start. Does not return.
void nmea_ble_restart_on_wifi(void);

// Starts BLE. Only valid when nmea_ble_boot_mode() returned true, so that WiFi
// was never started and the radio is free.
esp_err_t nmea_ble_start(void);

typedef struct {
    bool enabled;      // BLE has the radio now, and WiFi is stopped
    bool connected;
    bool subscribed;   // the client enabled notifications on the TX characteristic
    uint16_t mtu;      // negotiated ATT MTU; 23 until the client asks for more
    char peer[18];     // "aa:bb:cc:dd:ee:ff"
    uint32_t sentences;
    uint32_t bytes;
    uint32_t dropped;
} nmea_ble_status_t;

void nmea_ble_status(nmea_ble_status_t *out);
