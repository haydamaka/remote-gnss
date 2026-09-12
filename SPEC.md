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
  2. the built-in one: the macros `WIFI_STA_SSID` and `WIFI_STA_PASSWORD` in
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
  - **GNSS** (opens by default): coordinates and magnetometer readings as text,
    and a plot that switches between two views with a radio pair:
    - **Compass**: a card that turns so north sits at magnetic north, with an
      index mark for the module's +X axis and the heading read out. The default.
    - **Sphere**: the calibration readings as a point cloud with the current
      field as an arrow. A calibration session switches to this view and never
      switches back on its own.
  - **WiFi**: connection status; set the network to join (the name is
    pre-filled); forget a network set from the page, which falls back to the
    built-in one; the device's uptime and why it last restarted.

## Web API

Everything the page does goes through this, so anything else can drive the
device too. All replies are JSON. There is no authentication: it is meant for a
home network.

| Method and path | What it does |
|---|---|
| `GET /` | the page itself, embedded in the firmware |
| `GET /api/status` | GNSS fix, magnetometer reading (calibrated when there is a calibration), WiFi, and the device: uptime, last reset reason, build time, running slot, free heap |
| `GET /api/mag/samples?since=SEQ` | raw magnetometer readings after that sequence number, with the scale factor; the board keeps the last 1024 |
| `GET /api/cal` | calibration state, progress, coverage, the criterion it is waiting on, how the last session ended, the live fit, the stored calibration, and the buffered readings |
| `POST /api/cal/start[?fresh=1]` | start a session; it continues from the saved readings unless `fresh=1` |
| `POST /api/cal/stop` | interrupt a session, keeping what is worth keeping |
| `POST /api/cal/accept` | save the current fit although it is below the quality bar |
| `POST /api/cal/points` | replace the buffered readings with a CSV (`x,y,z` or `seq,t_ms,x,y,z`) |
| `POST /api/wifi` | `ssid` and `password`, form-encoded: save and restart |
| `POST /api/wifi/forget` | drop the network set from the page and restart |
| `POST /api/ota` | a firmware `.bin` as the body: check it, install it, restart into it |
| `POST /api/restart` | restart the device |

## Updating the firmware

- The partition table (`partitions.csv`) holds two app slots, so firmware can be
  installed over WiFi with `POST /api/ota` and the device restarts into it. A
  bad image leaves the running one alone.
- Only a change to the partition table itself needs USB.
- NVS keeps its offset in that table, so saved WiFi credentials and calibration
  survive an update.

## External GNSS interfaces

- The device should expose the GNSS stream over common interfaces, so a PC or
  an Android phone can use it as a wireless GPS.
- **TCP and Bluetooth** are the wanted connections. Tested with ready-to-use
  client programs on Linux and Android where possible, rather than purpose-built
  ones.
- Plan: [EXTERNAL-GNSS.md](EXTERNAL-GNSS.md). Note that the ESP32-C3 has no
  Bluetooth Classic (no SPP): Bluetooth there means BLE with the Nordic UART
  Service.

## Spoofing

Satellite signals can be faked, so the page warns when the fix cannot be
trusted. Two independent sources, because either alone can be fooled:

- **The receiver's own verdict.** The device polls UBX-NAV-STATUS every 2 s and
  reads `spoofDetState`. Polling changes nothing about the receiver's
  configuration.
- **Our own checks**, which do not depend on the receiver being honest:
  - the receiver's clock against **network time** (SNTP over WiFi), which the
    spoofer does not control — more than two minutes apart is a warning;
  - the receiver's year against the firmware's build year, for when there is no
    network;
  - a jump in position faster than 900 km/h.

## Magnetometer calibration

- Calibrate the hard- and soft-iron distortion iteratively: keep taking
  readings, recompute the centre and scaling, exclude outliers, and continue
  until the readings form a "good" sphere. Continuous or in iterations.
- Our own algorithm, not based on ArduPilot's. It should make re-calibration
  easy, and the user should always see how well the calibration is going.
- Started with a button on the web page. It completes on its own once good
  enough, or is interrupted with a Stop button.
- While calibrating, the GNSS page shows a small 3D plot of the magnetometer
  points (the dotted sphere).
- Fit 9 parameters: 3 offsets plus a symmetric 3×3 soft-iron matrix.
- A completed calibration is saved to NVS and loaded again at start.
- The calibration readings are saved too, so a later calibration continues
  from them instead of starting from nothing.
- **Every session ends with something saved**, as long as the numbers are
  physical, and the page says plainly what was saved: **good**, **rough** or
  **poor**. All three are applied — the compass keeps working whatever the
  quality — and anything but good is shown as a warning. A session that has
  nothing physical to offer says so and leaves the previous calibration alone.
- **Save anyway** stores the current fit before it meets the bar.
- **Save and load calibration** write and read the *readings* (not the fitted
  parameters) as CSV, named `mag-YYYYMMDD-HHMMSS.csv`, in the same columns as
  the recordings under `tools/`.
- The quality limits and the coverage target are `menuconfig` settings under
  **Remote GNSS → Magnetometer calibration**.
- Outside calibration, the same plot shows the current magnetometer vector as
  an arrow (not a point), in the same colour scheme as the points.
- World Magnetic Model: a later phase.
- Before implementing it in firmware: record real readings and try the
  algorithm offline.
- Algorithm proposal: [CALIBRATION.md](CALIBRATION.md).

## Firmware modules

| Module           | Responsibility                                          |
|------------------|---------------------------------------------------------|
| `wifi.c`         | Station and soft access point, credentials in NVS       |
| `mdns_service.c` | mDNS hostname and service advertisement                 |
| `webserver.c`    | HTTP server, embedded page, JSON API                    |
| `gnss.c`         | Reads the receiver on UART1; NMEA parsing with minmea   |
| `magnetometer.c` | Reads the QMC5883P/L or IST8310 on I2C                  |
| `mag_cal.c`      | Calibration sessions, direction cells, criteria, NVS storage |
| `mag_fit.c`      | The fit itself: robust 9-parameter ellipsoid, no RTOS or storage |
| `ota.c`          | Firmware updates over WiFi                              |

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
- The orientation of the magnetometer's axes relative to the module isn't
  established yet. The horizontal plane looks like XY, from a flat spin.
