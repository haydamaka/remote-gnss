#pragma once

#include <stdbool.h>

#include "esp_err.h"

// Shows the device's state on the onboard LED, in a 2 s cycle:
//
//   flashes    1 is WiFi, 2 is BLE; 100 ms each without a link, 400 ms with
//              one (WiFi: joined a network, or a phone on the access point;
//              BLE: a client connected)
//   resting    dark without a fix, lit with one (the flashes then go dark)
//   error      a 10 Hz blink instead: no magnetometer, or it fails to read;
//              or the GNSS receiver silent
//   spoofing   a 5 Hz blink instead, while the fix looks spoofed and there is
//              no error
//
// ble says which radio this boot ended up on.
esp_err_t led_start(bool ble);
