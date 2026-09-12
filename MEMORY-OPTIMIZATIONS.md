# Memory

The ESP32-C3 has 400 kB of SRAM, but after the ROM, the WiFi and Bluetooth
stacks and the linker have taken theirs, **166 kB is left for everything
else** — and static buffers come off the top of that, leaving the rest as the
heap that WiFi, NimBLE, lwIP and the web server all draw on.

Measure with:

```sh
idf.py build && python3 tools/ram_report.py
```

## What went wrong (2026-09-12)

The device booted, joined WiFi, and could not be reached at all:

```
E (730) httpd: httpd_server_init: error in socket (105)     ← ENOBUFS
E (730) web: webserver_start(): start
```

`nmea_hub` logged 81 kB of heap free, NimBLE initialised, and 40 ms later the
web server could not allocate a socket. Nothing had crashed. The device ran
happily with no API, no page and no OTA — recoverable only with a USB cable.

Two things were wrong, and both are worth keeping in mind:

1. **The web server started last**, so it was the component that lost the race
   for what was left. It now starts before the wireless NMEA outputs: the page
   is how the device is controlled and updated, so whatever else runs short,
   there is still a way in. That does not fix a shortage, it makes it
   survivable.
2. **Our own code held 74 kB of static RAM** — against 16 kB for every ESP-IDF
   library put together. The shortage was ours.

## Where the 74 kB went, and where it is now

| | before | after | |
|---|---|---|---|
| `magnetometer.c` | 16,424 | 4,136 | history 1024 → 256 samples |
| `webserver.c` | 28,032 | 12,480 | `/api/cal` no longer carries the readings |
| `mag_cal.c` | 19,472 | 11,872 | fit scratch made dynamic; one NVS staging buffer, not two |
| `mag_fit.c` | 7,220 | **0** | its working arrays belong to the caller now |
| `bearing.c` | 3,240 | 3,036 | |
| everything else | 1,015 | 1,015 | |
| **total** | **74,388** | **32,523** | |

### 1. The magnetometer history: 16 kB → 4 kB

A 1024-entry ring of raw samples, so a recorder could ride out a WiFi dropout —
100 seconds of tolerance that nothing had ever needed. At 256 it is still 25
seconds, and it was the single largest buffer in the firmware.

### 2. `/api/cal` stopped sending the readings: 17 kB

The page polled `/api/cal` every second while calibrating and **every ten
seconds for ever afterwards**, and every reply carried the whole point cloud:
about 8 kB of JSON built from 1,200 floats, for readings that had not changed —
to draw a view that is usually not even on screen, because the default is the
compass and it never plots them.

That needed a 12,288-byte response buffer and a 4,800-byte copy of the points.
Both are gone. `/api/cal` is about 600 bytes and carries a `points_seq`
counter; `GET /api/cal/points` streams the readings one at a time, so no copy
of the buffer exists anywhere; and the page asks only when the sphere view is
showing *and* the counter says they moved. While a session runs it refetches
every poll regardless, because they genuinely change and the counter is then
redundant.

### 3. The fit's working memory: 17 kB, only while calibrating

The residual and sort arrays, the solver matrix, the direction-cell tables, the
snapshot of the readings and the tidy-up buffer came to about 17 kB that the
firmware carried whether or not anything was calibrating. They now live in one
block that `mag_cal.c` owns and that exists only while a session does.

`tools/fit_test.c` gives byte-identical output on both recordings before and
after the change, which is the only reason to believe the move was faithful.

## The one allocation, and why it cannot leak

This is the **only** `malloc` in the firmware. Everything else, in every
steady-state path — `gnss.c`, `magnetometer.c`, `bearing.c`, `nmea_*`,
`wifi.c`, every web request — is static. So "is there a leak?" stays answerable
with `grep -rn 'alloc\|free(' main/`.

It cannot leak, for two reasons that hold by construction rather than by care:

**The count is bounded at one.** `scratch_acquire()` returns the existing block
if there is one and only allocates when there is none. However many paths call
it, however many times, there is at most one block in existence.

**The free is driven by an invariant, not paired with an exit.** A calibration
session can leave the running state in four places, only one of which goes
through `finish()`. Pairing a `free()` with each is exactly how leaks get
written, and a fifth exit added later would leak silently. Instead the
calibration task checks, every cycle:

> nothing is calibrating and no loaded readings are waiting to be fitted,
> therefore nothing owns the scratch — release it.

That does not know or care which path ran, so it cannot be missed.

**The worst case equals the old behaviour.** If the calibration task ever
wedged, one block would stay held — which is precisely the static buffer that
used to be there permanently. The design cannot be worse than what it replaced.

**It is checked, not asserted.** The free heap is recorded when the block is
taken and compared when it is released; a shortfall is logged. A mistake in the
reasoning above shows up as a line in the log rather than as a slow death.

**Failure is visible.** If the allocation fails, `mag_cal_begin()` returns
`ESP_ERR_NO_MEM` immediately and the page says so, instead of the session
quietly never fitting. This is the real gain from making it dynamic: a
permanent cost became an occasional, visible, recoverable failure.

## What is left, and why

| | bytes | |
|---|---|---|
| `mag_cal.c` `points[400]` | 8,000 | **staying.** These are the calibration readings — the state the device exists to hold. They are in NVS too, so they could be dropped when idle and streamed back from flash, but every path that wants them would then need a buffer to decode into, which is most of what was saved. 8 kB of certainty is worth more. |
| `webserver.c` `pts[400][3]` | 4,800 | worth removing: the CSV upload could parse and feed readings incrementally rather than collecting them all first. Needs a small `begin`/`add`/`end` API on `mag_cal`. |
| `webserver.c` `buf[4096]`, `chunk[2048]`, `buf[1536]` | 7,680 | one handler runs at a time, so these could share a single arena — about 3.5 kB back. Cheap, but each handler must be checked for using two of them at once. |
| `magnetometer.c` `history[256]` | 4,096 | lower it again if the recorder's dropout tolerance is not wanted. |
| `bearing.c` `st` | 3,036 | the learning state and the drive trace. Persistent by design; the trace is 1,920 of it and halves to one hour if needed. |
| `mag_cal.c` `packed[400][3]` | 2,400 | the NVS staging buffer, shared between save and load. Needed whenever the readings are written or read. |

## Two things deliberately not done

**Calibration and the web server must not share memory.** They look mutually
exclusive and are not: the page can fetch `/api/cal/points` *while* the
calibration task is fitting. Sharing a buffer between them would corrupt a fit
at exactly the moment someone opened the page.

**No allocation in the request path.** The web server's buffers were removed by
not needing them — streaming — rather than by moving them to the heap. A
request that can fail for want of memory is a page that disappears when the
device is busiest.

## Rules

- Regular operation allocates nothing. Static buffers, or stream it.
- One `malloc` in the firmware, bounded at one block, released by an invariant.
- A buffer sized for the worst case and alive for ever is a bug in waiting; a
  response that can be streamed should be.
- Start the control surface before the optional services.
- Measure with `tools/ram_report.py` before and after. Every figure in this
  document came from it.
