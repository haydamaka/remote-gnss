# remote-gnss — Specification

Firmware for an ESP32-C3 SuperMini wired to a Walksnail WS-M181 GNSS + compass
module. It serves the position and magnetometer readings on a web page over WiFi.

## Platform

- ESP32-C3 SuperMini, ESP-IDF v5.5, running on FreeRTOS (ESP-IDF's).
- Settings are kept in NVS.

## Hardware

Two receivers are supported, chosen at build time in menuconfig under
"Remote GNSS → GNSS → Receiver". Only one is compiled in; everything above
`gnss.c` is the same either way.

### Walksnail WS-M181 — the default

From vendor listings:
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

A toggle switch between pad 10 and G chooses the radio (see External GNSS
interfaces).

**Status LED** (added 2026-09-18). The onboard blue LED on GPIO8 shows the
state in a 2 s cycle: one flash for WiFi, two for Bluetooth; dark between the
flashes without a GNSS fix, lit with one (the flashes then go dark). Each
flash is short (100 ms) without a link and long (400 ms) with one (added
2026-09-19): on WiFi, the station has joined a network or a phone has joined
the access point; on Bluetooth, a client is connected. While the
fix looks spoofed it blinks fast (5 Hz) instead; on an error - no magnetometer
or its reads failing, or the GNSS receiver silent for 3 s - very fast (10 Hz).
An error shows over spoofing. A task of
its own drives the LED, every 50 ms.

Pins to leave alone: 2 (strapping pin, must read high at reset), 8 (onboard
LED), 9 (BOOT button), 20/21 (UART0, serial console).

### u-blox ZED-F9P — the RTK alternative

A bare ZED-F9P-02B on a breakout board with an SMA antenna connector, with no
compass of its own: with it selected, the magnetometer either comes from a
separate board on the same I2C pins or the compass tab has nothing to show.

It arrives at 38400 baud, 1 Hz, and NMEA rounded to five decimals of a minute —
about 2 cm, which throws away most of what RTK is for. So unlike the WS-M181 it
is configured on every lock (`gnss_f9p.c`): the link rate, the navigation rate
(menuconfig, default 5 Hz), high-precision NMEA, and the sentence set with GSV
slowed to once a second. All of it goes to the receiver's RAM layer, never its
flash, so pulling its power restores whatever its owner had.

Without RTCM3 corrections on its RX line it is a 1 m receiver with a good
antenna; an NTRIP client to supply them is not written yet.

Wiring is the same four lines as the WS-M181 — 5V, GND, the receiver's TX to
GPIO 4 and its RX to GPIO 3 — minus the I2C pair.

## WiFi

- **Station first.** The device joins a WiFi network when it has credentials:
  1. the network set from the web page (saved in NVS), or else
  2. the built-in one: the macros `WIFI_STA_SSID` and `WIFI_STA_PASSWORD` in
     `main/wifi_credentials.h`. That file is kept out of git; see
     `main/wifi_credentials.h.example`.
- **Soft access point.** An open access point (default name `haydamaka-puck-gnss`, no
  password) comes up when there are no credentials, or when the station has
  been without a connection for 30 s. It goes down again once the station
  connects.
- **mDNS.** The device answers as `haydamaka-puck-gnss.local` and advertises the web
  server as `_http._tcp`. (`gnss.haydamaka.local` was asked for; see Open
  questions.)
- On the WiFi page, add the possibility to save multiple WiFi networks. Also add
  a WiFi network scan.
  - Up to five networks are saved in NVS, newest first, names and passwords
    together. The built-in one is still there as well, and cannot be forgotten.
  - The station scans and joins the strongest network it knows that is in
    range, and moves straight on to the next when one fails; networks the scan
    did not see are tried last, in case they are hidden. With a single known
    network it skips the scan and joins it directly, as before. The 30 s access
    point fallback is unchanged.
  - Adding or forgetting a network takes effect without a restart: an access
    point with nothing to join starts joining, a device waiting to retry tries
    at once, and a device on a network that was just forgotten leaves it.
  - The page lists the saved networks with a Forget button each and marks the
    connected one. Passwords are never sent back to the page.
  - **Scan** lists nearby networks, one row per name, strongest first, with
    signal, secured or open, and whether it is already known. Picking one fills
    in the name. While the station is busy joining a network the scan is
    refused and the page says to try again.
  - The network saved by earlier firmware (NVS keys `ssid`/`pass`) moves into
    the list on the first boot, so an update keeps the device on its network.

## Web interface

- HTTP server on port 80.
- The page source is a separate file in `html/`, embedded in the firmware at
  build time (not inline in C).
- Three tabs:
  - **Compass** (opens by default): a large compass card with azimuth markings
    every 30°, the heading read out in the middle, and the basic fix underneath
    — quality, satellites, position, altitude, speed. A **desired heading** box
    marks that bearing on the card and says which way to turn to reach it. No
    configuration of any kind, and sized for a phone.
  - **GNSS/Magnetometer**: coordinates and magnetometer readings as text,
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
| `GET /api/status` | GNSS fix, magnetometer reading (calibrated when there is a calibration, and `mag.smooth`, the same smoothed at rest), WiFi, and the device: uptime, last reset reason, build time, running slot, free heap |
| `GET /api/mag/samples?since=SEQ` | raw magnetometer readings after that sequence number, with the scale factor, and `smooth`, the latest smoothed at rest in uT, uncalibrated; the board keeps the last 256 |
| `GET /api/cal` | calibration state, progress, coverage, the criterion it is waiting on, how the last session ended, the live fit, the stored calibration, and how many readings are buffered |
| `GET /api/cal/points` | the buffered readings themselves, streamed; fetch when `points_seq` changes |
| `POST /api/cal/start[?fresh=1]` | start a session; it continues from the saved readings unless `fresh=1` |
| `POST /api/cal/stop` | interrupt a session, keeping what is worth keeping |
| `POST /api/cal/accept` | save the current fit although it is below the quality bar |
| `POST /api/waypoint` | `lat`, `lon` (decimal degrees) and an optional `id`, form-encoded: set the destination; or `clear=1` |
| `GET /api/waypoint` | the destination set over BLE (`set`, `id`, `lat`, `lon`) and, with a fix, the way to it under `nav`: `distance_m`, `bearing_true`, `bearing_mag`, `closing_kn`, `arrived` |
| `GET /api/bearing` | what driving has taught it, how much it wanders, and why fixes were turned away |
| `GET /api/bearing/log` | the drive as CSV, a row a minute |
| `POST /api/bearing` | `learning`, `applying`, `reset` or `save`, form-encoded |
| `POST /api/heading` | `offset` in degrees, form-encoded: the correction added to the measured heading |
| `POST /api/gnss` | form-encoded, any of: `withhold_spoofed=0\|1`, whether a fix that looks spoofed is kept from TCP and Bluetooth (`/api/status` reports it under `gnss`, with `withholding` when it is happening); `filter_speed=0\|1`, `motion=none\|walking\|vehicle\|both` (reported under `gnss` with `receiver_config`: `pending`, `ok`, `rejected` or `unsent`) |
| `POST /api/radio` | form-encoded, any of: `wifi_dbm` (2-20, at once) and `ble_dbm` (0-18 in 3 dB steps, or 20; from the next BLE start); `/api/status` reports them under `radio` |
| `POST /api/cal/points` | replace the buffered readings with a CSV (`x,y,z` or `seq,t_ms,x,y,z`) |
| `GET /api/wifi` | the saved networks by name (never their passwords), how many can be saved, and the built-in one's name |
| `POST /api/wifi` | `ssid` and `password`, form-encoded: save a network, or replace its password; no restart |
| `POST /api/wifi/forget` | `ssid`, form-encoded: forget that network; with no body, every saved one; no restart |
| `GET /api/wifi/scan` | nearby networks: `ssid`, `rssi`, `channel`, `secure`, `known`, strongest first; takes two or three seconds, and 503 while busy joining |
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
- **Compass as the course at low speed** (added 2026-09-19). Below 5 km/h
  (`CONFIG_RGNSS_NMEA_COMPASS_BELOW_KMH`, 0 turns it off) the GNSS course over
  ground is unreliable, so the RMC and VTG sent over TCP and BLE carry the
  compass heading instead: the heading the page shows (learned correction and
  offset included) plus the WMM declination as the true course, and without it
  in VTG's magnetic field. Only with a fix, a fresh calibrated magnetometer
  reading and the declination known; otherwise the receiver's course passes
  unchanged. The page and the bearing learning still see the receiver's own.
- **The radio is chosen by a switch only** (added 2026-09-18). A switch on GPIO10
  to ground: on is WiFi, off is Bluetooth. It is read at boot, and flipping it
  restarts the device on the other radio. The web page has no Bluetooth
  control and there is no idle fallback to WiFi; the page only explains the
  switch. If BLE fails to start, the device restarts into one
  boot on WiFi, whatever the switch; flipping it, or a power cycle, tries BLE
  again.
- **Navigate to a destination** (added 2026-09-19, for the Android app, which
  takes a place shared from Google Maps). A client sets one destination with
  `$--WPL,ddmm.mmmm,N,dddmm.mmmm,E,<id>` on the BLE command channel and forgets
  it with `$PPUCK,WPL,CLEAR`; answered `$PPUCK,WPL,OK` or
  `$PPUCK,WPL,ERR,<FORMAT|INVALID|STORE>`. The identifier is up to 8 printable
  characters. It is kept in NVS, so it survives a restart and serves either
  radio. While there is a destination and a fix, `$GPRMB` and `$GPBWC` go out
  about once a second to BLE and TCP: bearing (true, and magnetic from the
  bearing task's WMM declination), range, closing velocity, arrival within
  20 m; not while a spoofed fix is withheld. Details in EXTERNAL-GNSS.md,
  "Destination: WPL in, RMB and BWC out". Not settable over TCP: the TCP server
  doesn't read what clients send.
- **Destination tab** on the page (added 2026-09-19): shows the destination
  (name, position with a map link), the distance, the bearing true and
  magnetic, the closing speed, and an arrow toward it against the compass
  heading with how far to turn. It can be set there too (added 2026-09-19): a
  coordinates field that reads what is typed or pasted in the common forms -
  decimal degrees; degrees and minutes, or degrees, minutes and seconds, with
  any of the usual marks (° º ' ′ " ″, d m s) or none; N/S/E/W letters before
  or after, also Ukrainian (пн/пд/сх/зх); a minus sign; a decimal comma; the
  two swapped when the first is over 90; Google Maps, OpenStreetMap and geo:
  links - and says how it read it, or why it can't, before anything is sent.
  A short maps.app.goo.gl link has no coordinates in it, and the page says so.
  A name of up to 8 characters, reduced to what NMEA can carry; DEST without
  one. A Clear button forgets the destination.
- **Speed at rest** (added 2026-09-19). The receiver measures speed from
  Doppler; at rest its noise is a small vector in a random direction, so the
  speed never reaches 0 and the course spins. Under GNSS config on the page,
  kept in NVS:
  - *Speed measuring profile*: not set, walking, in a vehicle, or both. Sets the
    receiver's dynamic model (pedestrian, automotive, portable); not set leaves
    the receiver's own.
  - *Filter speed jitter* (one checkbox for both ends, on by default):
    - the receiver's static hold (UBX-CFG-VALSET, CFG-MOT-GNSSSPEED_THRS /
      GNSSDIST_THRS: 0.25 m/s and 3 m walking, 0.5 m/s and 10 m in a vehicle,
      0.3 m/s and 5 m otherwise), sent to the receiver's RAM whenever it is
      found, so again after it loses power; the page says whether it
      acknowledged;
    - below 2 m/s the firmware smooths the velocity vector (1 s time
      constant), not the speed, so the noise averages to 0; stopped within
      twice the receiver's own speed accuracy (UBX-NAV-PVT sAcc, polled once a
      second; 0.1-0.4 m/s), moving again 0.1 m/s above that. RMC and VTG carry
      the speed. The course is not filtered - below 5 km/h the compass replaces
      it (CONFIG_RGNSS_NMEA_COMPASS_BELOW_KMH, not on the page) - and goes empty
      while stopped.
  The page, the compass-course threshold and the destination's closing speed
  use the same speed; the bearing learning keeps the receiver's. Speed and
  course are cleared when the receiver loses the fix, rather than kept from
  the last one.
- **Transmit power** (added 2026-09-19) on the WiFi/BLE tab, kept in NVS: WiFi
  2-20 dBm (default 20), applied at once; Bluetooth 0-20 dBm (default 9),
  applied on its next start.
- **Tabs** (2026-09-19): Compass, Destination, GNSS/Magnetometer, WiFi/BLE.
  GNSS/Magnetometer has two cards, side by side on a wide screen and stacked
  on a phone: GNSS on the left (the position, then GNSS config: the spoofing
  and speed settings) and Magnetometer on the right (the readings, then
  Magnetometer config: calibration, Set bearing, Automatic adjustment). The
  heading is always smoothed at rest; there is no setting for it. The speed
  settings carry no explanation on the page.
- **Phone app** (2026-09-19): GNSS Master is the Android bridge known to work
  over BLE; GPS Connector and SW Maps don't connect. The page says so.
- **Restart button** (added 2026-09-19) on the WiFi/BLE tab, next to the device
  line (`POST /api/restart`). A restart follows the switch: after a boot on
  WiFi against it (`$PPUCK,RADIO,WIFI` over BLE, or a failed BLE start), it goes
  back to BLE. The page shows where the switch stands (`ble.switch`,
  `ble.wifi_forced` in `/api/status`) and warns before a restart that leads to
  BLE, since the page then stops answering.

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
- The receiver's year against the build year allows one year ahead, not five: a
  spoofer on 2026-09-17 was sending 2028-03-10 and passed the old limit. It also
  sent 2026-01-01 at other times, which only network time catches - about
  45 s after joining WiFi, once SNTP has answered.
- **Don't send positions that look spoofed** — a checkbox under Position on the
  GNSS/Magnetometer tab, on by default, kept in NVS. While either detector
  says the fix looks faked, and for a minute after the last sign, TCP and
  Bluetooth clients get the receiver's "no fix" form of GGA, RMC, GLL and VTG,
  with no time or position in them; GSV and TXT pass, everything else is held
  back. A phone using the device as its location then falls back to its own.

## Compass bearing

- Below the compass: a **bearing** box and a **Set bearing** button. Point the
  module along a direction whose bearing is known, type that bearing in, and
  the device works out the offset between it and the measured heading.
- The offset is kept in NVS and added to every heading shown, on the card and
  in the readout. The current correction is shown under the button.
- Both controls are disabled in sphere view, where a heading means nothing.
- The row shows the resulting offset, signed, next to the button.
- **Learning the offset from GNSS:** when the device is moving and the fix is
  not spoofed, the GNSS course over ground says which way it is travelling, and
  the difference from the measured heading is the mounting offset. Switched on
  with a checkbox. Algorithm: [BEARING-CALIBRATION.md](BEARING-CALIBRATION.md).
  - It starts in **shadow mode**: it measures and says what it *would* set, and
    changes nothing until "Use what it learns" is ticked as well.
  - A **drive decides whether it is worth keeping**, so the page and the API
    report how much the estimate wanders rather than how confident the fit is:
    drift in degrees per hour, how far it moved at its widest, how far two
    independent halves of the data disagree, and how well it predicts the next
    straight run it has not seen. The fit's own standard error is not reported;
    it is far too pleased with itself to be useful.
  - The state and a **record a minute of qualifying driving** are kept in NVS,
    so a trip survives the engine being switched off, and the log can be
    downloaded as CSV afterwards.
  - When nothing qualifies, it says what stopped it — no fix, faked signals,
    too few satellites, an uncalibrated magnetometer, a disturbed field — so a
    fruitless drive can be told apart from one that never got going.
  - The offset data collected while driving is stored relative to the field
    vector, not the bearing, so it is invariant to the magnetometer
    calibration, and it is stored in NVS. A recalibration also recalibrates the
    offset, reusing what was already collected: each straight run is kept with
    its raw field vector and magnetic course, and they are all learned again
    under the new calibration ([BEARING-CALIBRATION.md](BEARING-CALIBRATION.md)
    §3.8).
  - A **Reset adjustment data** button. The row under the compass holds the
    bearing, **Set bearing** and the offset; the row below it, **Automatic
    adjustment** and **Reset adjustment data**.

## Magnetometer readings

- The noise in the readings is periodic. Filter it in software (EMI is also to
  be fixed in hardware), and read the magnetometer at a higher rate.
- Can't we figure out the WiFi beacon interval and cancel out those readings?
- Measured on 2026-09-16 with the module still: while the WiFi radio is
  powered, the IST8310 reads about 31 µT off along a fixed direction in its own
  axes, and transmissions push it to 26-48 µT. In modem sleep the radio powers
  up and down many times a second, for beacons and for traffic, so a third to
  two thirds of the readings were off. Turning power save off makes the offset
  constant but costs current, so instead:
  - The firmware learns from the WiFi driver itself when the radio is powered
    (its `esp_phy_enable`/`esp_phy_disable` calls, wrapped at link time), not
    from the beacon interval, which says nothing about traffic.
  - The chip is read at 100 Hz. Each 10 Hz sample is the per-axis median of the
    reads taken with the radio off; with none in the 100 ms, the median of the
    radio-on reads less the learned offset; with neither, no sample.
  - `GET /api/status` reports under `mag.radio` how many samples came each way
    and the learned offset.
- Found on 2026-09-19: in BLE mode the controller kept the radio powered the
  whole time (Bluetooth modem sleep was off), so no read was ever taken with it
  off and every sample was dropped: no field, no heading, no compass course in
  the NMEA. Bluetooth modem sleep is now on, with the main crystal as its clock
  (the board has no 32 kHz one). The controller powers the radio down between
  radio events through the same `esp_phy_disable` call, and while advertising
  every read is a radio-off one.
- Measured on 2026-09-19, reading every read with its radio state and the LED
  state over USB: in BLE mode with a client connected, and in WiFi mode idle,
  the samples hold to about 0.3-0.4 degrees of heading, with no trace of the
  LED, of BLE events, or of the time since the radio last switched. With the
  page polling, a sample corrected from radio-on reads (1.4-3.9 uT of scatter)
  was a spike of up to 5 degrees, about one in twelve. A corrected sample is
  now used only after half a second with no clean read, which took the heading
  from 0.65 to 0.54 degrees standard deviation on the same recording.
- Smoothing at rest (added 2026-09-19). Asked for: reduce the jitter while the
  compass isn't moving. Each sample also updates a smoothed field, which the
  heading is taken from everywhere - the page, `$HCHDG`/`$HCHDT` and the
  compass course; the calibration plot keeps the raw samples. At rest it moves
  1/20 of the way to each sample (a 2 s time constant). Two samples in a row
  more than 2 uT (about 4 degrees) from it horizontally, and agreeing with each
  other, are a move: it jumps to the reading and follows at half-steps for a
  second. On the recordings the noise never reached 2 uT; live, with the page
  polling, the heading went from 0.91 to 0.18 degrees standard deviation (5.9
  to 0.9 peak to peak).

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
| `gnss_receiver.h`| What differs between receivers: rate, configuration, UBX |
| `gnss_ws181.c`   | WS-M181 profile: 115200 baud, nothing sent to it         |
| `gnss_f9p.c`     | ZED-F9P profile: rate, high-precision NMEA, sentence set |
| `magnetometer.c` | Reads the QMC5883P/L or IST8310 on I2C                  |
| `mag_cal.c`      | Calibration sessions, direction cells, criteria, NVS storage |
| `mag_fit.c`      | The fit itself: robust 9-parameter ellipsoid, no RTOS or storage |
| `bearing.c`      | Learns the bearing offset while driving; NVS, trace log  |
| `bearing_cal.c`  | The estimator: straight-run windows, sectors, deviation coefficients |
| `wmm.c`          | World magnetic model: declination and field strength    |
| `ota.c`          | Firmware updates over WiFi                              |

minmea (https://github.com/kosma/minmea) is vendored in `components/minmea`.

## Stage 1 scope

Everything above: WiFi (station + soft AP), mDNS, web server, and the two-tab
page with text readings.

## Open questions

- `gnss.haydamaka.local`: checked on hardware 2026-09-11. ESP-IDF's mDNS
  component (espressif/mdns 1.12.0) answers it with the labels
  `gnss.haydamaka` + `local`, one label containing a literal dot, which
  resolvers asking for `gnss` + `haydamaka` + `local` reject. Used
  `gnss-haydamaka.local` instead; the two-label name would need a patched
  mDNS component. Renamed to `haydamaka-puck-gnss` on 2026-09-17, for mDNS and
  Bluetooth alike, with the page title **Haydamaka Puck GNSS**.
- The orientation of the magnetometer's axes relative to the module isn't
  established yet. The horizontal plane looks like XY, from a flat spin.
