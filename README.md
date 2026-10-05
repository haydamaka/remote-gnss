# remote-gnss

ESP-IDF v5.5 firmware for an ESP32-C3 SuperMini or an ESP32-WROOM DevKit, with a
Walksnail WS-M181 or a u-blox ZED-F9P. See [SPEC.md](SPEC.md) for what it does and
[CALIBRATION.md](CALIBRATION.md) for the magnetometer calibration and
[EXTERNAL-GNSS.md](EXTERNAL-GNSS.md) for the planned TCP/BLE GNSS output.

`tools/` holds the offline calibration work: `mag_record.py` records the
device's magnetometer readings to CSV (standard library only), and
`mag_calibrate.py` is the reference implementation of the algorithm and needs
numpy (plus matplotlib for `--plot`).

## WiFi credentials

The built-in station network lives in `main/wifi_credentials.h`, which git ignores:

```sh
cp main/wifi_credentials.h.example main/wifi_credentials.h   # then fill in the SSID and password
```

Without it the device starts an open access point, `haydamaka-puck-gnss`; its page is at
http://192.168.4.1/. Other settings (access point name, mDNS hostname, pins) are
under **Remote GNSS** in `idf.py menuconfig`.

## Command line

```sh
. ~/esp/esp-idf/export.sh
idf.py build
idf.py -p /dev/ttyACM0 flash monitor
```

## Build parameters

Two choices are made at compile time, independently of each other. With
neither given, the build is for the C3 SuperMini with the WS-M181.

| Parameter | Values | How to set it | What it decides |
|---|---|---|---|
| Board | `esp32c3` (default): ESP32-C3 SuperMini<br>`esp32`: ESP32-WROOM-32 DevKit | `-D IDF_TARGET=esp32` | Chip, and the defaults for every pin, the LED's polarity, the radio switch's sense and the BLE power ceiling (`main/Kconfig.projbuild`, `sdkconfig.defaults.esp32`) |
| Receiver | WS-M181 (default)<br>ZED-F9P | `-D SDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.f9p"`, or menuconfig: Remote GNSS → GNSS → Receiver | `gnss_ws181.c` or `gnss_f9p.c` compiled in, the hostname and AP name, and the navigation rate setting (F9P only) |

What the board parameter sets:

| | SuperMini (`esp32c3`) | WROOM DevKit (`esp32`) |
|---|---|---|
| GNSS UART | UART1, RX GPIO 4 / TX GPIO 3 | UART2, RX GPIO 16 / TX GPIO 17 |
| Magnetometer I2C | SDA GPIO 1 / SCL GPIO 0 | SDA GPIO 21 / SCL GPIO 22 |
| Status LED | GPIO 8, lit low | GPIO 2, lit high |
| WiFi/BLE switch | GPIO 10; open is BLE | GPIO 27; open (none fitted) is WiFi |
| BLE transmit power | 0-20 dBm | 0-9 dBm |
| Serial port | `/dev/ttyACM0` (USB on chip) | `/dev/ttyUSB0` (CH340) |

What the receiver parameter sets:

| | WS-M181 | ZED-F9P |
|---|---|---|
| Hostname and AP | `haydamaka-puck-gnss` | `haydamaka-f9p-gnss` |
| Configuration sent | none: it arrives set up | NMEA on, rate (default 5 Hz), high-precision NMEA, sentence set, link rate; RAM only |
| Magnetometer | on the module | a separate board, or none |

Every pin and name stays a menuconfig setting under **Remote GNSS**; the
parameters only change their defaults. Each combination gets its own build
directory and sdkconfig, so they never overwrite each other's settings:

```sh
idf.py build                                                       # C3 + WS-M181
idf.py -B build-c3-f9p -D SDKCONFIG=sdkconfig.c3-f9p \
       -D SDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.f9p" build  # C3 + F9P
idf.py -B build-wroom-ws181 -D SDKCONFIG=sdkconfig.wroom-ws181 \
       -D IDF_TARGET=esp32 build                                   # WROOM + WS-M181
idf.py -B build-wroom-f9p -D SDKCONFIG=sdkconfig.wroom-f9p -D IDF_TARGET=esp32 \
       -D SDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.f9p" build  # WROOM + F9P
idf.py -B build-wroom-f9p -p /dev/ttyUSB0 flash
```

The defaults only apply when an sdkconfig is first made: to switch an existing
build directory to another combination, delete its sdkconfig first.

## Updating over WiFi

After the first USB flash, firmware goes over the network:

```sh
idf.py build
curl -X POST --data-binary @build/remote-gnss.bin http://gnss-haydamaka.local/api/ota
```

The device checks the image before it takes it, restarts into it and keeps its
settings; a bad image leaves the running firmware alone. `curl -X POST
http://gnss-haydamaka.local/api/restart` just restarts it.

The partition table (`partitions.csv`) holds two app slots for this, so the
first flash after changing it must go over USB.

## CLion

1. Settings → Build, Execution, Deployment → Toolchains: add a toolchain named `ESP-IDF`,
   **Add environment → From file** → `~/esp/esp-idf/export.sh`.
2. Settings → CMake: the `Debug` profile (from `.idea/cmake.xml`) already passes
   `-G Ninja -DIDF_TARGET=esp32c3`; select the `ESP-IDF` toolchain for it.
3. Build the `app` target; flash with the `flash` target (a CMake Application run
   configuration with target `flash`, no executable).
