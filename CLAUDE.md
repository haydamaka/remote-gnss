# remote-gnss

ESP-IDF v5.5 firmware for an ESP32-C3 SuperMini with a GNSS receiver and a
magnetometer. [SPEC.md](SPEC.md) holds the requirements,
[CALIBRATION.md](CALIBRATION.md) the calibration algorithm.

## Commits

Commit messages and pull request descriptions carry **no AI metadata**: no
`Co-Authored-By`, no `Claude-Session`, no "Generated with" line, no emoji
signature. Write the message as the author would.

## Build

`~/venv-ardupilot` sits on PATH and its python3 shadows the system one, which
breaks ESP-IDF's export script. Before building:

```sh
unset VIRTUAL_ENV
PATH=$(echo "$PATH" | tr ':' '\n' | grep -v venv-ardupilot | paste -sd:)
. ~/esp/esp-idf/export.sh
idf.py build
idf.py -p /dev/ttyACM0 flash
```

The board enumerates as `/dev/ttyACM0`. Opening that port resets it, so wait
for it to rejoin WiFi before driving it over the network.

## Secrets

`main/wifi_credentials.h` holds the WiFi password and is kept out of git. Never
commit it, and never print its contents.
