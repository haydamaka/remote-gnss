# remote-gnss — Specification

Firmware for an ESP32-C3 SuperMini wired to a Walksnail WS-M181 GNSS + compass
module. It serves the position and magnetometer readings on a web page over WiFi.

## Platform

- ESP32-C3 SuperMini, ESP-IDF v5.5, running on FreeRTOS (ESP-IDF's).
- Settings are kept in NVS.

## Hardware

Walksnail WS-M181 (from vendor listings):
- GNSS: u-blox M10 (UBX-M10050), GPS + GLONASS + BDS by default. Outputs NMEA
  and UBX at 115200 baud, 10 Hz by default.
- Compass: QMC5883 on I2C. Both chip variants are handled: QMC5883P (0x2C) and
  QMC5883L (0x0D).

The first WS-M181 held its UART and I2C lines at 0 V and never responded. A
replacement GPS module, fitted on 2026-09-11, streams NMEA at 115200 baud and has
an IST8310 magnetometer at 0x0C, which is also supported (any of 0x0C-0x0F).

Wiring:

| WS-M181 | SuperMini pad |
|---------|---------------|
| 5V      | 5V            |
| GND     | G             |
| TX      | 4 (UART1 RX)  |
| RX      | 3 (UART1 TX)  |
| DA      | 1 (I2C SDA)   |
| CL      | 0 (I2C SCL)   |

Pins to leave alone: 2 (strapping pin, must read high at reset), 8 (onboard
LED), 9 (BOOT button), 20/21 (UART0, serial console).

## WiFi

- **Station first.** The device joins a WiFi network when it has credentials:
  1. the network set from the web page (saved in NVS), or else
  2. the built-in one: thcoe macros `WIFI_STA_SSID` and `WIFI_STA_PASSWORD` in
     `main/wifi_credentials.h`. That file is kept out of git; see
     `main/wifi_credentials.h.example`.
- **Soft access point.** An open access point (default name `remote-gnss`, no
  password) comes up when there are no credentials, or when the station has
  been without a connection for 30 s. It goes down again once the station
  connects.
- **mDNS.** The device answers as `gnss-haydamaka.local` and advertises the web
  server as `_http._tcp`. (`gnss.haydamaka.local` was asked for; see Open
  questions.)

## Web interface

- HTTP server on port 80.
- The page source is a separate file in `html/`, embedded in the firmware at
  build time (not inline in C).
- Two tabs:
  - **GNSS** (opens by default): coordinates and magnetometer readings as text.
  - **WiFi**: connection status; set the network to join; forget a network set
    from the page, which falls back to the built-in one.

## Magnetometer calibration (requested, not implemented)

- Calibrate the hard- and soft-iron distortion iteratively: keep taking
  readings, recompute the centre and scaling, exclude outliers, and continue
  until the readings form a "good" sphere. Continuous or in iterations.
- Algorithm proposal: [CALIBRATION.md](CALIBRATION.md).

## Firmware modules

| Module           | Responsibility                                          |
|------------------|---------------------------------------------------------|
| `wifi.c`         | Station and soft access point, credentials in NVS       |
| `mdns_service.c` | mDNS hostname and service advertisement                 |
| `webserver.c`    | HTTP server, embedded page, JSON API                    |
| `gnss.c`         | Reads the receiver on UART1; NMEA parsing with minmea   |
| `magnetometer.c` | Reads the QMC5883P/L or IST8310 on I2C                  |

minmea (https://github.com/kosma/minmea) is vendored in `components/minmea`.

## Stage 1 scope

Everything above: WiFi (station + soft AP), mDNS, web server, and the two-tab
page with text readings.

## Open questions

- `gnss.haydamaka.local`: checked on hardware 2026-09-11. ESP-IDF's mDNS
  component (espressif/mdns 1.12.0) answers it with the labels
  `gnss.haydamaka` + `local`, one label containing a literal dot, which
  resolvers asking for `gnss` + `haydamaka` + `local` reject. Using
  `gnss-haydamaka.local` for now; the two-label name would need a patched
  mDNS component.
- The magnetometer readings are raw sensor axes, uncalibrated. Their
  orientation relative to the module isn't established yet.
