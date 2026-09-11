# remote-gnss

ESP-IDF v5.5 firmware for ESP32-C3. See [SPEC.md](SPEC.md) for what it does.

## WiFi credentials

The built-in station network lives in `main/wifi_credentials.h`, which git ignores:

```sh
cp main/wifi_credentials.h.example main/wifi_credentials.h   # then fill in the SSID and password
```

Without it the device starts an open access point, `remote-gnss`; its page is at
http://192.168.4.1/. Other settings (access point name, mDNS hostname, pins) are
under **Remote GNSS** in `idf.py menuconfig`.

## Command line

```sh
. ~/esp/esp-idf/export.sh
idf.py build
idf.py -p /dev/ttyACM0 flash monitor
```

## CLion

1. Settings → Build, Execution, Deployment → Toolchains: add a toolchain named `ESP-IDF`,
   **Add environment → From file** → `~/esp/esp-idf/export.sh`.
2. Settings → CMake: the `Debug` profile (from `.idea/cmake.xml`) already passes
   `-G Ninja -DIDF_TARGET=esp32c3`; select the `ESP-IDF` toolchain for it.
3. Build the `app` target; flash with the `flash` target (a CMake Application run
   configuration with target `flash`, no executable).
