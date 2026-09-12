# External GNSS interfaces — plan

Make the device usable as a wireless GNSS receiver for a PC or a phone: the
NMEA stream that `gnss.c` already reads goes out over the network and over
Bluetooth, so off-the-shelf software (gpsd, mapping and survey apps, Android's
mock location provider) sees it as an ordinary GPS.

Status, 2026-09-12: **TCP done and verified on hardware; BLE working, but the
WiFi station drops while it runs.** Phases 1 and 6 are implemented
(`nmea_hub.c`, `nmea_tcp.c`, `nmea_ble.c`, `tools/nmea_client.py`) along with
the sentence filter that phase 5 was meant to make unnecessary. Still open: the
coexistence problem in §3, the Outputs tab (phase 4), UDP (phase 3), UBX rate
control (phase 5) and everything Android (phase 2).

## 1. Decisions and constraints

**The ESP32-C3 has no Bluetooth Classic.** Its radio is WiFi 4 + Bluetooth 5
**LE only** — no BR/EDR, so no RFCOMM and no SPP. This matters more than
anything else on this page: most Android apps advertised as "Bluetooth GPS"
expect an SPP receiver, and they will never see this device. Bluetooth here
means **BLE with the Nordic UART Service (NUS)**, the de-facto serial-over-BLE
profile that RTK receivers use, and the client has to support it explicitly.
Nothing in firmware can work around this; only a different chip (the original
ESP32) could.

**Consequence: WiFi/TCP is the primary interface, BLE the secondary one.** TCP
is supported by far more client software, needs no new ESP-IDF components, has
no flash cost, and gives higher throughput. BLE's advantages are that the phone
keeps its own WiFi and mobile data, lower power, and pairing that survives
walking out of AP range.

**RAM is the binding constraint, not flash.** The prediction here was that
NimBLE's ~200 KB of flash would be the problem; with the two-slot partition
table the image is 1.34 MB in a 1.875 MB slot, 29 % free, and flash never
mattered. What actually broke was heap: the BLE controller failed to start with
`BLE_INIT: Malloc failed`, and once it did start there was not enough left to
allocate a 2 KB buffer, so the BLE output silently received nothing. It needed
one connection, no pairing, smaller WiFi buffers, the WiFi IRAM optimisations
off, and the hub's buffers allocated before the controller starts. Free heap
after all of it is about 81 KB.

**NMEA goes out raw, as the receiver produced it.** Not re-generated from the
parsed `gnss_data_t`. Clients want GSA/GSV/VTG/GLL too (satellite counts,
DOP, constellation), and re-encoding would lose precision and add bugs. The
parsing in `gnss.c` stays exactly as it is and keeps feeding the web page.

**Ports.** TCP and UDP on **10110**, the IANA-registered port for NMEA-0183
navigational data. Every client below can be told an arbitrary port, so there
is no reason to pick something else.

### 1.1 Why not switch to a full ESP32

The original ESP32 is the only chip in the family with Bluetooth Classic — the
S3, C3, C6 and C5 are all BLE-only — so it is the one way to get SPP, and with
it the whole ecosystem of Android apps that pair with a receiver the way they
pair with a GPS dongle, plus `/dev/rfcomm0` on Linux. Not recommended, for now:

- **It is not needed to have Bluetooth at all.** BLE NUS is supported by GPS
  Connector, GNSS Master and SW Maps (§4.2). Classic SPP widens the list of
  apps that work; it doesn't decide whether Bluetooth works.
- **It is not needed to have wireless GNSS.** TCP over the device's own soft AP
  already reaches both a PC and a phone with no infrastructure, and that is
  phases 1–2 — the bulk of the value, on the board already wired up.
- **The costs are real.** A new board, re-wiring the WS-M181, losing the
  SuperMini's size; Bluedroid with Classic BT costs roughly half a megabyte of
  flash (several times NimBLE) and a large slice of RAM; and Classic BT sharing
  one radio with a WiFi AP coexists worse than BLE does. The ESP32 is also
  older, hungrier silicon.
- **Deferring is cheap.** The `nmea_hub` fan-out (§2) makes a transport a leaf
  module. Porting to an ESP32 would be pin numbers, a partition table, and one
  new sink alongside `nmea_ble.c` — days, not a rewrite.

What the ESP32 would genuinely buy, beyond SPP, is GPIOs: the C3 SuperMini has
few, and several are spoken for (SPEC.md "Hardware"). If the device grows a
display, buttons or SD logging, that argues for the switch more than Bluetooth
does.

**Decide it after phase 2, with data:** if real Android testing shows the BLE
apps are unusable *and* the phone must keep its own WiFi, the switch becomes a
considered decision instead of a guess. Until then, phase 6 is the cheap
experiment that answers the question.

## 2. Architecture

One new module fans the sentence stream out to any number of sinks:

| Module       | Responsibility                                              |
|--------------|-------------------------------------------------------------|
| `nmea_hub.c` | Subscriber registry; `nmea_hub_publish()` from the GNSS task |
| `nmea_tcp.c` | TCP listener on 10110, one task per client                   |
| `nmea_udp.c` | Broadcast datagrams, coalesced                               |
| `nmea_ble.c` | NimBLE GAP/GATT, Nordic UART Service                         |

```
                       ┌──────────────┐   ring   ┌────────────┐
 UART1 ── gnss.c ─────►│              ├─────────►│ TCP client │──► :10110
          (parse,      │   nmea_hub   ├─────────►│ TCP client │
           unchanged)  │              ├─────────►│ UDP sender │──► broadcast
 mag_cal ─────────────►│  (publish)   ├─────────►│ BLE notify │──► NUS TX
   (HDT/HDG, phase 7)  └──────────────┘          └────────────┘
```

Rules for the hub:

- **`nmea_hub_publish()` never blocks.** It is called from the GNSS reader
  task, which must keep draining the UART. Each subscriber owns a FreeRTOS
  stream buffer (2–4 KB); the publisher writes with a zero timeout and, on a
  full buffer, drops the sentence and bumps that subscriber's `dropped`
  counter. A phone that walks out of range must not stall the receiver.
- **Whole sentences only.** A subscriber never gets half a line, so a client
  that reconnects mid-stream starts at a sentence boundary.
- **Publish before parsing**, in `gnss.c`'s line splitter, so sentence types
  minmea doesn't know about still go out. Binary UBX frames are dropped (only
  lines starting with `$` or `!` are published).
- **Counters per subscriber** — bytes, sentences, drops, last activity — for
  the web page.

Sentence filtering (an allow-list of talker/type, e.g. `RMC,GGA,GSA,VTG`) lives
in the hub so the BLE sink can be given a narrower list than TCP.

## 3. Bandwidth, and why the receiver's own settings matter

The module streams at 10 Hz. With all constellations that is roughly 3–6 KB/s
(the GSV sentences dominate — one per 4 satellites per constellation).

- **TCP/UDP over WiFi:** irrelevant, orders of magnitude of headroom.
- **BLE NUS:** with a negotiated ATT MTU of 247 (244 B payload) and a 15–30 ms
  connection interval, throughput is roughly 7–15 KB/s in the best case and
  much less when WiFi is busy — the C3 has one radio and the two protocols
  take turns (software coexistence). 10 Hz with full GSV will stutter.

So BLE needs either a sentence filter or a lower receiver rate. The filter came
first, in the hub (`CONFIG_RGNSS_NMEA_TYPES`), because it needed no work on the
receiver: leaving GSV out drops the stream from 112 sentences/s and 6 KB/s to
32 sentences/s and 1.9 KB/s. BLE then carries the whole of it — 31.6
sentences/s measured, no losses. Phase 5 is still worth doing, to get GSV back
at a sane rate.

**Measured, not predicted:** two TCP clients at the unfiltered 6 KB/s each were
enough to exhaust lwip's send buffers and stall every socket at once, including
the web server's. The first version treated a stalled send as a dead client and
disconnected it, which was the wrong response to a busy link; it now loses the
batch and keeps the connection.

**Coexistence: mostly solved, and the rest is a deliberate trade-off.** The C3
has one radio, and NimBLE's defaults assume it has nothing else to do with it.
Advertising every 30–60 ms and a connection event every 15 ms starved the
station so thoroughly that it could not finish a WPA2 handshake with a −77 dBm
access point. Advertising at 500 ms–1 s, a connection interval of 60–120 ms,
WiFi power save off and the coexistence preference set to WiFi — all only when
BLE is in use — let the station associate and the web page answer, and cost BLE
nothing: 34 sentences/s with no errors, against 4.7 before.

What remains: association takes about 25 s and several retries, and an active
BLE connection still displaces the station. **That last part is accepted** — a
client that wants BLE gives up WiFi for as long as it holds the link. What is
not yet good enough is that the station should be dependable whenever BLE is
*idle*, so the device stays configurable; today it comes back but flakily.

The design that would settle it is to make BLE opt-in at run time rather than
always advertising: a switch on the Outputs tab (phase 4), saved in NVS, so
WiFi is untouched until someone deliberately chooses BLE. The signal itself is
the other half — −77 dBm leaves coexistence no margin, and this is worth
re-measuring next to the access point before tuning anything further.

## 4. Client software

### 4.1 Linux — all ready-made, nothing to write

| Purpose                  | Tool                                                      |
|--------------------------|-----------------------------------------------------------|
| Is anything coming out?  | `nc gnss-haydamaka.local 10110`                           |
| Feed the whole desktop   | `gpsd -N -n tcp://gnss-haydamaka.local:10110`             |
| Look at the fix          | `cgps`, `gpsmon`, `xgps` (once gpsd has it)               |
| Fake a serial GPS        | `socat pty,link=/dev/ttyGNSS,raw tcp:gnss-haydamaka.local:10110` |
| Log / replay             | `gpspipe -r`, or RTKLIB's `str2str`                       |
| Maps, QGIS, Chromium     | via gpsd → geoclue                                        |
| BLE → virtual serial     | `pip install ble-serial`; `ble-serial -d <MAC>` gives a `/dev/pts/N` that gpsd or socat can then read |

`gpsd` takes a `tcp://host:port` device directly, which is the whole
integration: after that, everything on the desktop that already speaks gpsd
works with no further configuration. `ble-serial` does the same job for BLE
NUS — it is written for exactly the Nordic UART profile phase 6 implements.

**Installed on the development machine** (Ubuntu 26.04) on 2026-09-12:

```sh
sudo apt install gpsd gpsd-clients pipx adb   # gpsd 3.27.5, adb 1.0.41
pipx install ble-serial                        # 3.0.0 -> ~/.local/bin
sudo systemctl disable --now gpsd.socket       # see below
```

`socat`, `nc` and `bluetoothctl` were already there, as was a working BLE
adapter (`hci0`).

**gpsd's socket activation is switched off deliberately.** The packaged
`gpsd.socket` starts the daemon on demand with the devices from
`/etc/default/gpsd` and holds port 2947, which collides with running
`gpsd -N -n tcp://…` by hand. Manual runs are what testing wants. For a
permanent desktop setup later, put the device back in `/etc/default/gpsd`
(`DEVICES="tcp://gnss-haydamaka.local:10110"`, `GPSD_OPTIONS="-n"`) and
re-enable the socket.

**Verified end to end on 2026-09-12** against a synthetic 1 Hz NMEA server on
port 10110, before any firmware exists: `nc` showed the sentences, `gpsd`
attached to the TCP URL with the `NMEA0183` driver, and `gpspipe -w` reported
`mode:3` at the expected coordinates with the right HDOP and satellite count.
`ble-scan` found nearby BLE devices through `hci0`. So the whole Linux client
chain is known-good; when phase 1 lands, anything that fails is the
firmware.

### 4.2 Android — ready-made apps, one of them covers both interfaces

The pattern on Android is a bridge app that reads an external receiver and
publishes it as the system **mock location provider** (Developer options →
"Select mock location app"); every other app — Google Maps, OsmAnd, a survey
app — then sees the external position as if it were the built-in GPS.

| App                       | TCP | BLE NUS | Notes                                    |
|---------------------------|-----|---------|------------------------------------------|
| **GPS Connector**         | yes | yes     | Bluetooth Classic, BLE, USB and TCP/IP in one app; the best single test client for us |
| **GNSS Master** (ArduSimple) | yes | yes  | USB/Bluetooth/BLE/WiFi-TCP bridge, mock location, free |
| **SW Maps**               | yes | yes     | Survey app, connects BLE receivers directly; useful as a "real app" test |
| **Lefebure NTRIP Client** | yes | no      | TCP/IP receiver source + mock location; long-standing, widely used |
| **nRF Connect**           | —   | yes     | Not a GPS app: a raw GATT browser, for proving the NUS service works at all before blaming the app |

All of these are checks to run, not guarantees — app capabilities change, and
the BLE column in particular should be confirmed against the current version.
The first thing to verify on any of them is that it accepts a **plain NMEA TCP
stream** (some expect an NTRIP caster instead).

Two Android connection topologies:

- **Phone joins the device's soft AP** (`remote-gnss`, 192.168.4.1). Works
  anywhere, no infrastructure. Android usually keeps mobile data alive on a
  WiFi network with no internet, but this is worth checking on the actual
  phone.
- **Both on the same WiFi network** — better when the phone needs its normal
  network; use `gnss-haydamaka.local` or the IP.

### 4.3 What I would write

Almost nothing — that is the point of choosing NMEA/TCP and NUS. Two small
things are worth having in `tools/`, alongside the existing calibration
scripts:

- `tools/nmea_client.py` — standard library only, connects to the TCP port and
  prints a live summary: sentence rate, checksum errors, gaps, fix state,
  reconnects. It is the regression test I can run unattended for an hour, and
  it needs no gpsd installed.
- `tools/ble_nmea.py` — only if `ble-serial` disappoints; a 40-line `bleak`
  script that subscribes to the NUS TX characteristic and writes to a pty.

## 5. How testing works, and what I can do

**Linux: fully automatable by me.** I can run the firmware's client side from
this machine end to end — connect to the TCP port, soak-test it, kill clients
mid-stream to check slot reuse, run `gpsd`+`cgps`, drive BLE through the local
`hci0` adapter with `ble-serial` or `bleak`, and compare what comes out against
the device's `/api/status`. You flash and power the board; everything after
that I can do and report on.

**Android: only with `adb`.** There is no `adb` on this machine. If you install
it (`apt install android-tools-adb`) and enable USB debugging on the phone,
I can install the bridge app, start it, read `logcat` for its connection
errors, check `dumpsys location` to confirm the mock provider is actually
feeding the system, drive the UI with `adb shell input`, and take screenshots
— i.e. the same end-to-end verification as on Linux. Without adb I can only
write you a step-by-step checklist and interpret what you report back. **This
is worth setting up**; the Android side is where integration problems will
actually be, and blind iteration over screenshots is slow.

## 6. Phases

Ordered so that each phase ends with something demonstrably working, cheapest
and least risky first.

### Phase 1 — NMEA hub + TCP server ← the phase that delivers most of the value

`nmea_hub.c` and `nmea_tcp.c`; publish from `gnss.c`'s line splitter; fixed
port 10110, max 4 clients; `SO_KEEPALIVE` and a send timeout so a vanished
phone frees its slot; mDNS advertises `_nmea-0183._tcp`. Client counters shown
on the existing GNSS tab.

*Done when:* `nc` shows sentences; `gpsd -N tcp://…:10110` + `cgps` shows the
fix; `socat` pty works; two clients at once; pulling a client's plug for a
minute doesn't disturb the other or the web page; an hour of
`tools/nmea_client.py` with no drops.

### Phase 2 — Android over TCP

No firmware work, ideally. Install GPS Connector (and GNSS Master as a
cross-check), connect over both the soft AP and a shared network, enable mock
location, confirm Google Maps follows the device. Any firmware fix needed here
(keepalive timing, a sentence some app insists on) lands as a phase 1 patch.

*Done when:* an unrelated app on the phone shows the device's position.

### Phase 3 — UDP broadcast

~60 lines. Coalesce sentences into datagrams (flush on RMC, or every 200 ms,
or at 1 KB) instead of one datagram per sentence. Unlocks marine/aviation
apps and multiple listeners with no connection state. Optional but cheap.

### Phase 4 — Outputs tab on the web page

Enable/disable each interface, port, per-sink sentence filter, connected
clients with IP/bytes/last-seen, all saved to NVS. Until now everything is
compile-time; this is what makes it usable without a rebuild.

### Phase 5 — Configure the receiver (UBX-CFG)

Set the navigation rate and the enabled sentence set on the M10 over UART TX
(GPIO 3 is already wired), saved to NVS, exposed on the Outputs tab. A
prerequisite for BLE being pleasant, and useful on its own (10 Hz GSV is noise
for most clients).

### Phase 6 — BLE, Nordic UART Service

The large one:

- custom `partitions.csv` (2 MB app) — the current build would not link
  otherwise;
- NimBLE (smaller than Bluedroid), `CONFIG_BT_NIMBLE_ENABLED`, software
  coexistence with WiFi;
- advertise as the mDNS hostname, service `6E400001-B5A3-F393-E0A9-E50E24DCCA9E`,
  TX notify `…0003`, RX write `…0002` (unused at first; later a path for UBX
  commands from the client);
- request an ATT MTU of 247 and a short connection interval; apply a sentence
  filter and honour the phase 5 rate setting.

*Done when:* `nRF Connect` sees the service and notifications; `ble-serial` on
this machine gives a pty that `gpsd` accepts; GPS Connector or SW Maps on the
phone connects and mock location works; and BLE plus WiFi run at the same time
without either collapsing.

### Phase 7 — Heading sentences (optional)

Once calibration is finished, inject `$HCHDG`/`$HCHDM` (magnetic heading) and
`$HCHDT` (true, once the WMM declination exists) into the stream at 1–2 Hz,
with correct checksums and a toggle. Marine and mapping apps display these, and
it is the one thing this device offers that a plain GPS dongle does not.

## 7. Open questions

- **A full ESP32 for Bluetooth Classic?** Not now, and revisit after
  phase 2 — see §1.1.
- **adb for Android testing** — install it, or work from screenshots? (§5)
- **Does the phone keep mobile data** while joined to the device's soft AP?
  Decides whether the soft AP or a shared network is the recommended setup.
- **Which Android app to standardise on.** GPS Connector looks like the best
  single candidate because it covers TCP *and* BLE, so the same app tests both
  phases; to be confirmed on the actual phone.
- **Security.** Both interfaces are wide open to anyone on the network, as the
  web page already is. Fine on the soft AP, questionable on a home network.
  Leaving it open for now; a per-interface enable in phase 4 is the mitigation.
- **Do we need a TCP *client* mode too** (device dials out to a fixed host),
  for use through a phone hotspot where nothing can reach the device? Not in
  this plan; easy to add to `nmea_tcp.c` later.
