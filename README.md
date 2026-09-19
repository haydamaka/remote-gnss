# remote-gnss

ESP-IDF v5.5 firmware for ESP32-C3. See [SPEC.md](SPEC.md) for what it does and
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
