#!/usr/bin/env python3
"""Records raw magnetometer samples from the device to CSV, for offline calibration work.

    tools/mag_record.py [--host HOST] [--out FILE] [--seconds N]

Writes seq, t_ms, x_ut, y_ut, z_ut, from the moment it starts until Ctrl-C (or N
seconds). Uses the device's /api/mag/samples stream; firmware without it is polled
through /api/status instead, which is coarser (the time comes from this computer and
readings between polls can be missed). If the device restarts mid-recording, the
sequence and time columns keep rising so the file stays one ordered stream.
Standard library only.
"""
import argparse
import csv
import datetime
import http.client
import json
import math
import pathlib
import sys
import time
import urllib.error
import urllib.request

# A reply that is cut short or malformed raises these; none should end a recording.
NET_ERRORS = (OSError, ValueError, http.client.HTTPException)


def fetch(url):
    try:
        with urllib.request.urlopen(url, timeout=3) as r:
            return json.load(r)
    except NET_ERRORS as e:
        print(f'  request failed: {e}', file=sys.stderr)
        time.sleep(1)
        return None


def has_stream(host, deadline):
    """Waits for the device to answer, then says whether it serves /api/mag/samples."""
    while time.monotonic() < deadline:
        try:
            urllib.request.urlopen(f'http://{host}/api/mag/samples?since=0', timeout=3).close()
            return True
        except urllib.error.HTTPError:
            return False
        except NET_ERRORS as e:
            print(f'  waiting for the device: {e}', file=sys.stderr)
            time.sleep(1)
    raise SystemExit(f'{host} did not answer')


def stream_samples(host, stats):
    """Yields (seq, t_ms, x, y, z) from /api/mag/samples, or None when nothing arrived."""
    since, skip = 0, True
    seq_base, t_base, last = 0, 0.0, None  # restarts continue where the file left off
    while True:
        reply = fetch(f'http://{host}/api/mag/samples?since={since}')
        if reply is None:
            yield None
            continue
        if not reply.get('chip'):
            print('  no magnetometer detected on the device', file=sys.stderr)
            time.sleep(2)
            yield None
            continue
        seq, samples, k = reply.get('seq', 0), reply.get('samples', []), reply.get('ut_per_count', 0)
        if not k:
            print('  device reports no scale factor', file=sys.stderr)
            time.sleep(2)
            yield None
            continue
        if seq < since:
            print('  device restarted, continuing', file=sys.stderr)
            if last:
                seq_base, t_base = last[0], last[1]
            since, skip = 0, True
        if skip:  # drop the backlog the device still holds
            since, skip = seq, False
            yield None
            continue

        for s, t_ms, rx, ry, rz in samples:
            stats['lost'] += s - since - 1
            since = s
            last = (seq_base + s, t_base + t_ms)
            yield last[0], round(last[1]), rx * k, ry * k, rz * k
        if not reply.get('more'):
            time.sleep(0.5)
            yield None


def status_samples(host, stats):
    """Yields readings by polling /api/status, for firmware without /api/mag/samples."""
    seq, last_t, start = 0, None, time.monotonic()
    while True:
        status = fetch(f'http://{host}/api/status')
        mag = status.get('mag') if status else None
        if mag and mag.get('age_ms') is not None and mag.get('x') is not None:
            t_ms = max(0.0, (time.monotonic() - start) * 1000 - mag['age_ms'])  # when the device took it
            if last_t is None or t_ms - last_t > 50:  # the device samples every 100 ms
                if last_t is not None:
                    stats['lost'] += max(0, round((t_ms - last_t) / 100) - 1)
                seq, last_t = seq + 1, t_ms
                yield seq, round(t_ms), mag['x'], mag['y'], mag['z']
                continue
        yield None
        time.sleep(0.03)


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--host', default='gnss-haydamaka.local', help='device address (default: %(default)s)')
    parser.add_argument('--out', help='CSV file (default: tools/recordings/mag-<time>.csv in the repo)')
    parser.add_argument('--seconds', type=float, help='stop after this long')
    args = parser.parse_args()

    repo = pathlib.Path(__file__).resolve().parent.parent
    out = pathlib.Path(args.out) if args.out else repo / f'tools/recordings/mag-{datetime.datetime.now():%Y%m%d-%H%M%S}.csv'
    out.parent.mkdir(parents=True, exist_ok=True)
    start = time.monotonic()
    deadline = start + args.seconds if args.seconds is not None else math.inf
    stream = has_stream(args.host, deadline)
    stats = {'lost': 0}
    source = (stream_samples if stream else status_samples)(args.host, stats)
    total, last_report, field = 0, start, float('nan')

    with out.open('w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['seq', 't_ms', 'x_ut', 'y_ut', 'z_ut'])
        print(f'recording to {out} from {"the sample stream" if stream else "status polling"}, Ctrl-C to stop')
        try:
            for sample in source:
                if sample is not None:
                    seq, t_ms, x, y, z = sample
                    writer.writerow([seq, t_ms, f'{x:.4f}', f'{y:.4f}', f'{z:.4f}'])
                    total += 1
                    field = math.hypot(x, y, z)
                now = time.monotonic()
                if now - last_report >= 1:
                    f.flush()
                    print(f'  {now - start:4.0f} s: {total} samples, {stats["lost"]} lost, |B| {field:.1f} µT',
                          flush=True)
                    last_report = now
                if now >= deadline:
                    break
        except KeyboardInterrupt:
            pass
    print(f'saved {total} samples ({stats["lost"]} lost) to {out}')


if __name__ == '__main__':
    main()
