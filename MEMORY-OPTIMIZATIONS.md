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
2. **Our own code held 82 kB of static RAM** — against about 16 kB for every
   ESP-IDF library put together. The shortage was ours.

## Where the 82 kB went, and where it is now

| | before | after | |
|---|---|---|---|
| `webserver.c` | 29,312 | 5,376 | `/api/cal` stopped carrying the readings; one arena for the rest |
| `mag_cal.c` | 24,668 | 12,268 | fit scratch made dynamic; one NVS staging buffer, not two |
| `magnetometer.c` | 16,424 | 4,136 | history 1024 → 256 samples |
| `mag_fit.c` | 7,220 | **0** | its working arrays belong to the caller now |
| `bearing.c` | 3,240 | 3,240 | |
| everything else | 891 | 891 | |
| **total** | **81,755** | **25,911** | |

The first version of `tools/ram_report.py` reported 74 kB rather than 82: the
linker map puts a long symbol name on a line of its own, with its size on the
next, and the parser only understood the single-line form. It silently missed
every large buffer with a long name. Both figures above come from the fixed
one, on clean builds of the same two commits.

### 1. The magnetometer history: 16 kB → 4 kB

A 1024-entry ring of raw samples, so a recorder could ride out a WiFi dropout —
100 seconds of tolerance that nothing had ever needed. At 256 it is still 25
seconds, and it was the single largest buffer in the firmware.

### 2. `/api/cal` stopped sending the readings, and the handlers share an arena: 24 kB

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

What was left — a 4 kB response buffer, a 2 kB OTA buffer and a 1.5 kB one —
now share a single 4 kB arena. The HTTP server dispatches every handler from
one task, so no two are ever live at once. The upload buffer went a different
way: see below.

### 3. The fit's working memory: 22 kB, only while calibrating

The residual and sort arrays, the solver matrix, the direction-cell tables, the
snapshot of the readings and the tidy-up buffer came to about 22 kB that the
firmware carried whether or not anything was calibrating. They now live in one
block that `mag_cal.c` owns and that exists only while a session does.

An uploaded CSV is parsed into the same block — into the fit's own snapshot
array, which is idle whenever nothing is fitting — so the web server needs no
4.8 kB buffer of its own for an occasional upload. A `fitting` flag makes sure
it is never handed out while the calibration task is using it.

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
`ESP_ERR_NO_MEM` immediately and the page says "not enough memory free to
calibrate; try switching Bluetooth off", instead of the session quietly never
fitting. This is the real gain from making it dynamic: a permanent cost became
an occasional, visible, recoverable failure.

### The block has three owners, not one

The first version got this wrong, and review caught it before it reached the
board. "One owner at a time" sounded true but there are **three**: the
calibration task fitting, the calibration task releasing, and the **web server
task parsing an upload into it**. Only the fit marked itself, so:

- an upload took the pointer and started reading from the socket; within 100 ms
  the calibration task saw nothing calibrating and freed the block underneath
  it, and the handler then wrote 4.8 kB into freed heap;
- and an upload during a running session was overwritten by the next fit's
  snapshot, halfway through.

Both are now closed by a `loading` flag that the upload holds: the release
skips it, a fit refuses to start while it is set, and every exit from the
upload handler gives it back.

The first attempt at that also added a 30-second timeout, so a vanished client
could not pin the block for ever — and that was the same bug again. A timeout
cannot take memory back from a task that still holds the pointer: on a slow
upload it cleared the flag and freed the block in the same lock hold while the
web server was still writing into it. **Memory is released when its owner gives
it back, never on a clock.** What bounds the hold instead is the request: the
body is capped at 64 kB (400 readings are about 10 kB of CSV) and the stall
counter ends a connection that stops making progress.

## Measured on the device (2026-09-12)

| | free heap |
|---|---|
| idle | 118,616 |
| calibrating | 99,364 |
| after stopping | 115,420 |
| settled, repeatedly | 118,616 |

The 19.5 kB is the block plus what the session itself is doing. Free heap
returns to the same figure every time — the dips in between are the in-flight
request's own buffers, not a leak. `heap_min` over the whole test was 80,980;
before this work the device booted to a minimum of **300 bytes** and the web
server could not start.

`/api/cal` went from about 8 kB per poll to **381 bytes**. The 355 stored
readings still stream in full from `/api/cal/points` (6,305 bytes), and a
round trip through Save and Load restored all 355.

## What is left, and why

| | bytes | |
|---|---|---|
| `mag_cal.c` `points[400]` | 8,000 | **staying.** These are the calibration readings — the state the device exists to hold. They are in NVS too, so they could be dropped when idle and streamed back from flash, but every path that wants them would then need a buffer to decode into, which is most of what was saved. 8 kB of certainty is worth more. |
| `webserver.c` `req_scratch[4096]` | 4,096 | the shared request arena, sized by the largest user. |
| `webserver.c` `samples[…]` | 1,280 | the magnetometer sample batch for `/api/mag/samples`. |
| `magnetometer.c` `history[256]` | 4,096 | lower it again if the recorder's dropout tolerance is not wanted. |
| `bearing.c` `st` | 3,068 | the learning state and the drive trace. Persistent by design; the trace is 1,920 of it and halves to one hour if needed. |
| `bearing.c` `runs[256]` | 4,096 | the straight runs the estimate is rebuilt from after a recalibration (BEARING-CALIBRATION.md §3.8). Could live only in NVS and be streamed during a replay, at the cost of reading eight keys every time; the ring halves to 128 runs if needed. |
| `mag_cal.c` `packed[400][3]` | 2,400 | the NVS staging buffer, shared between save and load. Needed whenever the readings are written or read. |
| `mag_cal.c` `by_age[400]` | 800 | ordering scratch for the same save. |

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

- Regular operation allocates nothing from our code. Static buffers, or stream
  it. (FreeRTOS objects — task stacks, mutexes, the NMEA message buffers — do
  take heap, but only at start-up, so `grep` for `alloc` is a necessary check
  and not a sufficient one.)
- One `malloc` in the firmware, bounded at one block, released by an invariant.
- A buffer sized for the worst case and alive for ever is a bug in waiting; a
  response that can be streamed should be.
- Start the control surface before the optional services.
- Measure with `tools/ram_report.py` before and after. Every figure in this
  document came from it.
