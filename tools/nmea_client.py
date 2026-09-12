#!/usr/bin/env python3
"""Check the device's NMEA output, over TCP or through a serial port.

Standard library only, so it runs anywhere without gpsd installed. It is the
soak test for the TCP server and the BLE link: it reports the sentence rate,
the fix, checksum errors, gaps in the stream and reconnects, and exits non-zero
if anything went wrong.

    tools/nmea_client.py                              gnss-haydamaka.local:10110
    tools/nmea_client.py --host 192.168.4.1           the soft access point
    tools/nmea_client.py --serial /dev/pts/5          BLE, behind ble-serial
    tools/nmea_client.py --duration 3600 --quiet      an hour, summary only
    tools/nmea_client.py --raw                        print the sentences too

Over BLE, first bind the device to a pty:

    ble-scan                       find the address
    ble-serial -d <MAC>            prints the /dev/pts it created
"""

import argparse
import errno
import os
import re
import select
import socket
import sys
import time
from collections import Counter

# "$GPGGA,...*4A" — the talker and type are the 5 characters after the "$".
SENTENCE = re.compile(rb"^\$([A-Z]{2})([A-Z]{3}),(.*?)(?:\*([0-9A-Fa-f]{2}))?$")

# A stream that goes quiet for longer than this has a gap worth reporting.
GAP_S = 3.0


def checksum(body: bytes) -> int:
    c = 0
    for b in body:
        c ^= b
    return c


class Stats:
    def __init__(self):
        self.sentences = 0
        self.bytes = 0
        self.bad_checksum = 0
        self.malformed = 0
        self.types = Counter()
        self.gaps = []
        self.reconnects = 0
        self.fix = None
        self.satellites = None
        self.position = None
        self.utc = None
        self.started = time.monotonic()
        self.last_sentence = None

    @property
    def elapsed(self):
        return time.monotonic() - self.started


def parse(line: bytes, st: Stats, raw: bool, first: bool = False):
    # Joining a byte stream lands mid-sentence as often as not, so the first
    # line of a connection is a fragment rather than a fault.
    if first and not line.startswith(b"$"):
        return
    now = time.monotonic()
    if st.last_sentence is not None and now - st.last_sentence > GAP_S:
        st.gaps.append((st.last_sentence - st.started, now - st.last_sentence))
        print(f"  ! {now - st.last_sentence:.1f} s gap in the stream", flush=True)
    st.last_sentence = now

    st.bytes += len(line) + 2
    m = SENTENCE.match(line)
    if not m:
        st.malformed += 1
        return
    talker, kind, fields, given = m.groups()

    if given is not None:
        want = checksum(line[1 : line.index(b"*")])
        if want != int(given, 16):
            st.bad_checksum += 1
            return

    st.sentences += 1
    st.types[(talker + kind).decode()] += 1
    if raw:
        print("  " + line.decode(errors="replace"), flush=True)

    f = fields.decode(errors="replace").split(",")
    if kind == b"GGA" and len(f) >= 7:
        st.utc = f[0] or st.utc
        quality = f[5]
        st.fix = int(quality) if quality.isdigit() else 0
        st.satellites = int(f[6]) if f[6].isdigit() else None
        if f[1] and f[3] and st.fix:
            st.position = (to_degrees(f[1], f[2]), to_degrees(f[3], f[4]))


def to_degrees(value: str, hemisphere: str) -> float:
    """NMEA ddmm.mmmm to signed decimal degrees."""
    point = value.index(".")
    degrees = int(value[: point - 2] or 0)
    minutes = float(value[point - 2 :])
    result = degrees + minutes / 60.0
    return -result if hemisphere in ("S", "W") else result


def status_line(st: Stats) -> str:
    rate = st.sentences / st.elapsed if st.elapsed else 0
    fix = {None: "?", 0: "no fix", 1: "GPS", 2: "DGPS", 4: "RTK fixed", 5: "RTK float", 6: "dead reckoning"}.get(
        st.fix, str(st.fix)
    )
    where = f"{st.position[0]:.6f},{st.position[1]:.6f}" if st.position else "-"
    sats = st.satellites if st.satellites is not None else "-"
    bad = st.bad_checksum + st.malformed
    return (
        f"\r{st.elapsed:6.0f}s  {st.sentences:7d} sentences  {rate:5.1f}/s  "
        f"{st.bytes / max(st.elapsed, 1e-9):6.0f} B/s  {fix:>13}  {sats:>2} sat  {where:>22}  "
        f"{bad} bad"
    )


def read_source(source, st: Stats, args, deadline):
    """Feeds lines from an open socket or file object until it ends."""
    buf = b""
    first = True
    fd = source.fileno()
    last_status = 0.0
    while time.monotonic() < deadline:
        ready, _, _ = select.select([fd], [], [], 0.5)
        if ready:
            chunk = source.recv(4096) if isinstance(source, socket.socket) else os.read(fd, 4096)
            if not chunk:
                return "closed by the other end"
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                parse(line.strip(b"\r"), st, args.raw, first=first)
                first = False
            if len(buf) > 4096:
                buf = b""  # nothing resembling a line; don't grow without bound

        now = time.monotonic()
        if not args.quiet and not args.raw and now - last_status > 0.5:
            sys.stdout.write(status_line(st))
            sys.stdout.flush()
            last_status = now
    return None


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="gnss-haydamaka.local")
    ap.add_argument("--port", type=int, default=10110)
    ap.add_argument("--serial", metavar="DEV", help="read a serial port or pty instead of TCP")
    ap.add_argument("--duration", type=float, default=30, help="seconds to run, 0 for no limit (default 30)")
    ap.add_argument("--raw", action="store_true", help="print every sentence")
    ap.add_argument("--quiet", action="store_true", help="summary only")
    ap.add_argument("--no-reconnect", action="store_true", help="stop at the first disconnection")
    ap.add_argument("--expect-fix", action="store_true", help="fail unless the receiver reports a fix")
    args = ap.parse_args()

    st = Stats()
    deadline = st.started + args.duration if args.duration else float("inf")
    where = args.serial if args.serial else f"{args.host}:{args.port}"
    print(f"reading {where}", flush=True)

    while time.monotonic() < deadline:
        try:
            if args.serial:
                fd = os.open(args.serial, os.O_RDONLY | os.O_NOCTTY)
                source = os.fdopen(fd, "rb", buffering=0)
            else:
                source = socket.create_connection((args.host, args.port), timeout=10)
                source.settimeout(None)
        except (OSError, socket.gaierror) as e:
            print(f"\n  ! cannot connect: {e}", flush=True)
            if args.no_reconnect or st.sentences == 0:
                return 2
            st.reconnects += 1
            time.sleep(2)
            continue

        try:
            reason = read_source(source, st, args, deadline)
        except OSError as e:
            reason = f"{errno.errorcode.get(e.errno, e.errno)}"
        finally:
            source.close()

        if reason is None:
            break  # the run finished on time
        print(f"\n  ! disconnected: {reason}", flush=True)
        if args.no_reconnect:
            break
        st.reconnects += 1
        time.sleep(1)

    if not args.quiet and not args.raw:
        sys.stdout.write(status_line(st) + "\n")

    print(f"\n{st.elapsed:.0f} s, {st.sentences} sentences, {st.bytes} bytes")
    if st.types:
        print("  types:      " + ", ".join(f"{k} {v}" for k, v in sorted(st.types.items())))
    print(f"  rate:       {st.sentences / max(st.elapsed, 1e-9):.2f} sentences/s")
    print(f"  bad:        {st.bad_checksum} checksum, {st.malformed} malformed")
    print(f"  gaps:       {len(st.gaps)}" + (f" (worst {max(g[1] for g in st.gaps):.1f} s)" if st.gaps else ""))
    print(f"  reconnects: {st.reconnects}")
    if st.position:
        print(f"  position:   {st.position[0]:.7f}, {st.position[1]:.7f}  ({st.satellites} satellites)")

    failed = []
    if st.sentences == 0:
        failed.append("no sentences arrived")
    if st.bad_checksum or st.malformed:
        failed.append(f"{st.bad_checksum + st.malformed} bad sentences")
    if st.gaps:
        failed.append(f"{len(st.gaps)} gaps")
    if st.reconnects:
        failed.append(f"{st.reconnects} reconnects")
    if args.expect_fix and not st.fix:
        failed.append("no fix")
    if failed:
        print("FAILED: " + "; ".join(failed))
        return 1
    print("OK")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        sys.exit(130)
