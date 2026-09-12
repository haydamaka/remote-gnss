#!/usr/bin/env python3
"""What the firmware holds in static RAM, from the linker map.

    python3 tools/ram_report.py [build/remote-gnss.map]

Reports .bss/.data per object file and the individual buffers, largest first.
This is the number that matters on an ESP32-C3: static RAM is taken off the
top, and whatever is left is the heap that WiFi, Bluetooth and the web server
have to share. See MEMORY-OPTIMIZATIONS.md.
"""

import collections
import re
import sys

SYMBOL = re.compile(r"^ (\.(?:bss|data|noinit|dram)\S*)\s+0x[0-9a-f]+\s+0x([0-9a-f]+)\s*(\S*)$")
WRAPPED = re.compile(r"^\s+0x[0-9a-f]+\s+0x([0-9a-f]+)\s+(\S+)$")


def parse(path):
    """Yields (size, symbol, object file). The map wraps long symbol names onto
    the following line, which is why this is not a one-liner."""
    lines = open(path, errors="replace").read().split("\n")
    i = 0
    while i < len(lines):
        m = SYMBOL.match(lines[i])
        if m:
            name, size, obj = m.group(1), int(m.group(2), 16), m.group(3)
            if not obj and i + 1 < len(lines):
                w = WRAPPED.match(lines[i + 1])
                if w:
                    size, obj = int(w.group(1), 16), w.group(2)
                    i += 1
            if size:
                yield size, name, obj.split("/")[-1]
        i += 1


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else "build/remote-gnss.map"
    try:
        entries = list(parse(path))
    except OSError as e:
        sys.exit(f"{e}\n\nBuild first, or pass the path to the .map file.")

    ours = collections.Counter()
    theirs = collections.Counter()
    for size, _, obj in entries:
        (ours if "libmain" in obj else theirs)[obj] += size

    print("Our own code:")
    for obj, size in ours.most_common():
        print(f"  {size:7d}  {obj}")
    print(f"  {sum(ours.values()):7d}  total\n")

    print("Everything else, over 500 bytes:")
    for obj, size in theirs.most_common():
        if size >= 500:
            print(f"  {size:7d}  {obj}")
    print(f"  {sum(theirs.values()):7d}  total\n")

    print("Individual buffers over 900 bytes:")
    for size, name, obj in sorted(entries, reverse=True):
        if size >= 900:
            print(f"  {size:7d}  {name[:38]:40s} {obj}")


if __name__ == "__main__":
    main()
