# Magnetometer calibration — proposal

Status:

- **Offline reference:** `tools/mag_calibrate.py`, tested (§9).
- **Firmware:** `main/mag_fit.c` is the 9-parameter fit, ported from that
  reference and checked against it on the host (`tools/fit_test.c`), and
  `main/mag_cal.c` runs it: the session, the direction cells, coverage
  progress, completion or Stop, NVS save/load, and the criteria of §2.
- **Quality tiers:** every session ends with something saved when the numbers
  are physical — good, rough or poor (§2.1) — all of them applied, so the
  compass works whatever the quality, with the page saying which it got.
- **Limits are settings:** the good and rough fit-error limits and the coverage
  target live in `menuconfig` under Remote GNSS → Magnetometer calibration.
- **Still missing, from §3.1:** the intake filters (validity and data-ready,
  the glitch test, the motion gate) and the per-cell agreement test. The fit's
  own MAD-gated reweighting (§3.1e) is in. Until the rest lands, stray readings
  still enter the buffer and show on the plot; the fit judges them, the buffer
  doesn't.
- **Also missing:** the noise estimate, so the fit-error limit is loosened (§9).

The steps are:

1. record real readings (`tools/mag_record.py`);
2. try the algorithm offline;
3. port it to the firmware.

Thresholds marked *(tune)* are starting values, to be checked on real IST8310
data.

Decisions so far (2026-09-11):

- **Our own design, not based on ArduPilot.** The goals are easy re-calibration
  and letting the user always see how the calibration is going.
- **Started from a Calibrate button** on the GNSS page. It completes on its own
  when good enough, or you press Stop.
- **While calibrating,** the page shows a small 3D plot of the points (the dotted
  sphere) with the progress.
- **Fit 9 parameters** (3 offsets + a symmetric 3×3 matrix), dropping to fewer
  automatically while coverage is still poor.
- **World Magnetic Model: a later phase.** Until then, the only strength check
  is 20–70 µT.

## 1. What is being calibrated

Raw readings `m` (µT, sensor axes) are distorted by the module itself and by
anything mounted rigidly with it:

- **hard iron**: a constant offset `o` (magnetised parts, DC currents);
- **soft iron and gain mismatch**: a linear distortion that turns the sphere
  into an ellipsoid, possibly tilted between the axes.

Calibrated field:

```
b = W · (m − o)          o: 3 offsets, W: symmetric 3×3 (6 values)
goal: |b| = R            R: local field magnitude
```

Why 9 parameters, and not 6:

- **6 parameters** (offsets plus a gain per axis) can stretch or squash the blob
  only along the X, Y and Z axes.
- **9 parameters** add the 3 cross-axis terms of `W`, which handle an ellipsoid
  tilted between the axes. Nearby steel or a battery often tilts it.
- The 3 extra terms cost nothing at run time. They only need decent coverage to
  estimate, so with too few directions the fit falls back to 6, or to a plain
  sphere.

Constraints of this device:

- **Magnetometer only, no accelerometer or gyro.** Samples can only be sorted by
  the direction of the magnetic vector about the current centre estimate.
  Heading needs the module held level.
- **The ESP32-C3 has no FPU**, so the buffer stays ≤ 400 samples and fits run in
  a low-priority task.
- **Calibration is only valid for one mounting.** Do it with the module in its
  final enclosure, away from the laptop. The ~76 µT seen on the bench is mostly
  hard iron.

## 2. What is a "good" sphere?

Five independent properties must all hold:

| # | Property | Measured as | Proposed pass *(tune)* |
|---|----------|-------------|------------------------|
| 1 | **Coverage**: samples reach all directions | direction cells (§3.2) holding ≥ 2 readings that agree with each other | ≥ 85 of the 100 cells, each ±X/±Y/±Z cap (45°) has ≥ 5 readings, and the smallest eigenvalue of the direction scatter `mean(u·uᵀ)` ≥ 0.2 (a uniform sphere gives 1/3) |
| 2 | **Fit error**: points lie on the sphere | RMS of `|b|/R − 1` over inliers | ≤ 1.5 %, and no tighter than 3 × measured noise / R |
| 3 | **Wobble**: no one-sided bias | length of the mean of per-cell average residual vectors, / R | ≤ 2 % |
| 4 | **Plausibility**: parameters are physical | R; eigenvalues of W normalised to det W = 1; \|o\| | R 20–70 µT; W eigenvalues 0.67–1.5; \|o\| ≤ 500 µT |
| 5 | **Stability**: the answer stopped moving | change between consecutive fits | 3 fits in a row with \|Δo\| ≤ 0.5 µT, max \|ΔWᵢⱼ\| ≤ 0.01, \|ΔR\|/R ≤ 0.5 % |

Plus: inliers make up ≥ 90 % of the buffer.

### 2.1 When it isn't perfect

Refusing to save unless everything passes throws away work and leaves the user
with no compass at all, which is worse than an imperfect one. So a session ends
in one of four ways, and says which:

| Outcome | When | What happens |
|---|---|---|
| **good** | every criterion above | saved; the session completes on its own |
| **rough** | fit error within the rough limit (6 % by default) and ≥ 70 % inliers, everything else as above | saved and labelled rough; the page warns. **Save anyway** offers this before the timeout |
| **poor** | the numbers are still physical: radius 20–70 µT, sane shape, at least half the sphere seen | saved and labelled poor, with advice to redo it standing still |
| **nothing** | not even that | refused, with the reason; the previous calibration stays in use |

All three saved tiers are applied to the compass. Quality is stored with the
calibration, so a later session and the page both know what they are holding.

Why each is needed:

- **Coverage.** A low fit error on a small arc of the sphere proves nothing:
  many ellipsoids pass through the same arc. The eigenvalue test catches
  rotating in one plane only.
- **Wobble.** Catches a fit dragged toward the densely sampled side.
- **Stability.** Guards against stopping on a lucky fit.

## 3. The algorithm

### 3.1 Outliers

Bad readings are not one thing, and the first design treated them as one. Three
kinds occur, with different causes and different cures:

| Kind | Cause | Looks like | Cure |
|---|---|---|---|
| **Glitch** | I2C noise, a torn read (the chip's axes are measured in sequence, and the driver doesn't check the data-ready flag) | one reading far off, neighbours fine | intake tests (a, b) and the data-ready check |
| **Smear** | the module turned fast: the axes are sampled at slightly different times, so the vector is measured mid-turn | runs of readings off the sphere, all during fast motion | the motion gate (c), and telling the user to slow down |
| **Distortion** | the field itself differs where the module is: metal nearby, a magnet, the desk | a whole region off the sphere, stable while the module is still | per-cell agreement (§3.2) and the monitor (§4) |

Measured on the two recordings in §9: glitches are rare (0.2 % of readings),
while fast rotation moves the vector by up to 36 µT between readings, and 8–20 %
of readings sit more than 3 MAD off the fitted sphere. A single fixed
threshold therefore cannot work: at 3 µT it would throw away 42 % of a normal
hand rotation, and at 20 µT it catches almost nothing.

**(a) Validity, per reading.** Discard read errors, saturation (IST8310:
±1600 µT on X/Y, ±2500 µT on Z), and readings whose data-ready flag was not
set. The last one needs a driver change and removes torn reads at the source.

**(b) Glitch test, against the reading's own neighbours.** Compare each reading
with the straight-line extrapolation of the two before it, and drop it when it
is further than `max(3 σ_noise, 0.5 · recent median |Δm|)` — the gate grows with
how fast the vector is currently moving, so it stays sensitive when the module
is still and doesn't eat data while it turns. σ_noise comes from the quietest
window (§9.4). A glitch is also asymmetric: it jumps away and comes straight
back, so a reading is only dropped when the next one returns near the line.

**(c) Motion gate.** Drop readings taken while the vector is moving faster than
about 15 µT per sample (roughly 90°/s at R = 50 µT): during those, the axes no
longer describe one instant. Count them, and when they exceed 20 % of recent
readings, the page says "rotate more slowly" — a user-visible cause, not a
silent drop.

**(d) Agreement within a direction cell** (§3.2) — the strongest test, because
it compares like with like. Readings pointing the same way should agree to
within noise, whatever the model is. A reading joins its cell only if it agrees
with the readings already there; otherwise it replaces the cell's
least-agreeing reading. Fitting then uses one point per cell, the
component-wise median of that cell's readings, so a wrong reading must be
repeated in the same direction before it can move the fit. This is what keeps
single junk readings out of both the fit and the plot.

**(d2) Tidy-up before saving.** When a session ends, the fit is run once more
on just the readings it kept, so the strays stop pulling on the answer. It is
taken only if it keeps ≥ 70 % of the readings, improves the error by at least a
tenth, and moves the centre by less than 1 µT — otherwise it is refused as
flattery rather than cleaning. Measured on a real session: 355 readings became
313 and the error fell from 3.67 % to 3.06 %, with the centre unmoved.

Why the guards matter: trimming a set that is really two sets (readings from two
places) does move the centre, and then the "cleanup" is inventing an answer. On
one such recording the centre wandered 4.6 µT under trimming, and the tidy-up
refuses it; on a sound one it moved less than 1 µT (§9).

**(e) Robust fitting, with the threshold taken from the data.** Everything
above is model-free; this last layer needs only a rough model:
- **RANSAC start** when the plain start fails: fit a sphere through 4 random
  cell points, count how many agree, repeat 50–200 times, keep the best. It
  needs no prior and survives about 50 % bad data.
- **Iteratively reweighted fitting**: compute residuals, take their spread
  `s = 1.4826 · MAD`, weight 0 beyond 3·s, refit, repeat until the inlier set
  stops changing, usually 3–5 passes.
- This breaks the chicken-and-egg loop: the median and MAD stay correct as long
  as more than half the points are good, however rough the current fit, and each
  refit sharpens the gate.

**Nothing is deleted for being an outlier.** Flags are recomputed on every fit,
so a reading wrongly rejected under an early model comes back when the model
improves. Readings leave the buffer only through the buffer policy (§3.2).
Rejected readings are shown on the plot in red, with counts per reason, so a
user can see whether they are rotating too fast or sitting next to a magnet.

**Distortion is not an outlier problem.** If most new readings (> 50 % of the
last 300) disagree with an accepted calibration, the field where the module sits
has changed. Re-calibrate (§4) rather than rejecting them.

### 3.2 Buffer: 100 direction cells

- **Equal-area cells:** bands of 1, 15, 34, 34, 15 and 1 cells, split at
  z = 0.98, 0.68, 0, −0.68, −0.98. A sphere's area is proportional to z, so every
  cell has the same area. Within a band, longitude is split evenly. This table is
  the normative one: `main/mag_cal.c` and `tools/mag_calibrate.py` each hold a
  copy and cite this section. The web page has none — if it ever needs cells,
  `/api/cal` should send each reading's cell index rather than a third copy.
- A sample's cell comes from its direction `u = (m − ô) / |m − ô|` about the
  current centre estimate `ô`.
- **Up to K = 4 samples per cell,** oldest first out: at most 400 samples,
  2.4 KB as int16. A module lying still can't pile hundreds of identical samples
  into one spot and bias the fit, and the cell map doubles as the coverage
  display.
- **A cell is a small committee.** A new reading joins its cell only if it
  agrees with the readings already there, to within
  `max(3 σ_noise, 2 % of R)`; otherwise it replaces the cell's least-agreeing
  reading, so a direction that has genuinely changed still turns over. The fit
  uses one point per cell, the component-wise median of the cell's readings,
  which makes the fit both robust (§3.1d) and cheap: at most 100 points instead
  of 400.
- Before the first fit, `ô` is the midpoint of the 2nd–98th percentile on each
  axis.
- When `ô` moves by more than 5 % of R, all samples are re-binned.
- **Once a fit can be trusted, the cell's worst reading makes way, not its
  oldest.** A new reading arriving in a full cell displaces whichever member
  disagrees most with the current fit. A long session then cleans itself: bad
  readings are pushed out as better ones arrive, so it converges instead of
  stalling, and nothing has to be thrown away wholesale.

  This mechanism can confirm its own mistakes, so it is fenced in:
  - only while the fit is plausible (§3.3) and at least 70 % of readings are
    inliers; otherwise eviction stays oldest-first;
  - never below two readings per cell, so no direction can be emptied to please
    the model;
  - a newcomer is only dropped in favour of a stored reading when it is clearly
    worse (say 1.5x), so a real change of environment still gets in;
  - the report keeps showing the error over *all* buffered readings and the
    share evicted, so a self-cleaned buffer cannot flatter itself.

  Without those guards the model eats its own tail: discard everything that
  disagrees and any fit becomes perfect, which is how a calibration can
  "succeed" and still be wrong. MotionCal does the same thing once coverage is
  good, and rate-limits it for the same reason (§8).

### 3.3 Fit pipeline

It runs every ~2 s or 20 new samples, in a low-priority task:

1. With fewer than 40 samples, skip.
2. **Initial sphere (o, R):** a linear least-squares (Kåsa) fit, or RANSAC if
   the last fit failed or had fewer than 70 % inliers.
3. **Robust sphere:** Gauss-Newton on the geometric residual
   `rᵢ = |mᵢ − o| − R`, reweighted as in §3.1.
4. **Model escalation by coverage:**
   - sphere (4 parameters);
   - then offsets + diagonal W (7 parameters), once the ±X/±Y/±Z caps are filled;
   - then the full symmetric W (10 parameters), at ≥ 150 inliers and full
     coverage.

   Each step is Levenberg-Marquardt on the geometric residual
   `|W(mᵢ − o)| − R`, started from the previous model, with the same
   reweighting.
5. **Normalise** W to det W = 1, so R carries the sensor gain.
6. **Evaluate §2** and report the state to the page.

### 3.4 States

```
IDLE ──Calibrate──▶ COLLECTING ──▶ FITTING ──§2 on 3 fits in a row──▶ DONE ──▶ saved
  ▲                     │  ▲           │                                        │
  │                     │  └──more─────┘                                        ▼
  └───── Stop ◀─────────┴──── (kept: previous calibration stays active) ◀── MONITORING
```

- **Stop** interrupts at any point. The previous calibration stays in use, and
  the collected points are kept, so a later run resumes from them.
- **Failure reasons** shown to the user: not enough coverage (naming the
  missing directions), readings too scattered, field strength implausible,
  offset too large, not settling.

## 4. Easy re-calibration

The main thing to do better than a one-shot calibration:

1. **Store the points, not just the parameters** (see §6). A re-calibration
   starts from the stored cells instead of from zero.
2. **Touch-up instead of redo.**
   - New samples replace the old ones cell by cell.
   - Each cell is marked fresh (confirmed by a new sample), stale (its old
     samples disagree with new data nearby), or untouched.
   - The plot colours cells accordingly, so after moving the module to a new
     mounting you only sweep the directions that are stale.
   - When the environment hasn't changed, re-calibration completes as soon as
     the session has taken in its minimum of fresh readings (about 10 s), so it
     never just re-blesses the old data.
3. **Background monitoring.**
   - Once calibrated, every sample is checked against the stored calibration,
     which costs one vector operation per sample.
   - Sustained disagreement (median `|b|/R − 1` > 3 % over the last 100 samples,
     or > 30 % outliers) raises a "calibration looks off" notice on the page with
     a one-tap re-calibrate.
4. **Never worse than before.**
   - A new calibration replaces the stored one only if it passes §2 and fits the
     current buffer better.
   - The stored calibration's fit error slowly ages, so an old good calibration
     can't block a newer one forever.
   - The previous calibration is kept for one-tap revert.

## 5. Progress the user can see

While the Calibrate button is active, the GNSS page shows:

- **A 3D point sphere,** small, auto-rotating and draggable. Its colours will
  show what matters:
  - points: inlier, or outlier in red;
  - cells: covered, missing, or stale;
  - the latest reading, highlighted.
- **A progress bar** built from the criteria:
  - coverage takes the bar to 70 %;
  - fit quality and stability fill the rest;
  - the state is written next to it: "Collecting — keep rotating", "Refining —
    hold still for a moment", "Done".
- **A checklist of §2,** each item with its value against the target, e.g.
  "Fit error 2.3 % (target ≤ 1.5 %)".
- **Hints:** the directions not reached yet (±X/±Y/±Z), or a nearby disturbance
  (a burst of outliers).
- **Why readings were dropped,** counted by reason: glitches, too-fast rotation,
  disagreement with their direction cell. "Rotate more slowly" appears when the
  motion gate (§3.1c) is dropping more than 20 % of readings.

**First version, already on the page for recording:** the point sphere, a
coverage bar, the missing-direction hint, Calibrate/Stop, and auto-complete
once coverage passes. The fit, checklist, stale cells and saving come with the
firmware port.

## 6. Storage: points or parameters?

**Both.**

| Stored | Size | Why |
|--------|------|-----|
| Parameters `{version, chip, o[3], W[6], R, fit error, time}` | ~60 B | applied at boot, before any new samples |
| The cell buffer, ≤ 400 raw samples + cell ages | ~3 KB | re-calibration resumes (§4); the page can show the stored sphere at any time; after an algorithm update the firmware can refit without anyone rotating the module again |

Written to NVS only when a calibration completes. Wear is negligible, and the
24 KB NVS partition has room.

Parameters alone would be enough to apply a calibration, but every
re-calibration would start from nothing.

## 7. Compute budget (rough, 160 MHz, software floating point)

| Step | Estimate |
|------|----------|
| One Levenberg-Marquardt iteration, 400 samples × 10 parameters | about 50 k float operations, ≈ 20–30 ms |
| Full fit (≈ 10 iterations) | ≈ 0.3 s |
| RANSAC, 100 hypotheses | ≈ 0.2 s |

Fine every few seconds in a low-priority task.

## 8. Existing implementations (survey)

Surveyed for your earlier question. None combines continuous refinement,
per-sample robust outlier handling and a formal stop.

| Implementation | Mode | Outlier handling |
|---|---|---|
| [ArduPilot `CompassCalibrator`](https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Compass/CompassCalibrator.cpp) | one-shot | none by residual; only minimum spacing between samples |
| [PX4 `mag_calibration`](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/commander/mag_calibration.cpp) | one-shot, 6 sides | none found; minimum spacing |
| [MotionCal / NXP](https://github.com/PaulStoffregen/MotionCal) | continuous | once coverage is good, periodically drops the sample furthest from the fitted field |
| [Android CHRE `mag_cal`](https://android.googlesource.com/device/google/contexthub/+/refs/heads/main/firmware/os/algos/calibration/magnetometer/mag_cal/mag_cal.c) | continuous, hard iron only | whole batches accepted or dropped |
| [ysoldak/magcal](https://github.com/ysoldak/magcal) | continuous, gradient descent | a refit when a vector's length is off |

The design above is our own. ArduPilot was not used as a basis.

## 9. Offline results (2026-09-11)

Run `tools/mag_calibrate.py REC.csv | --synthetic all [--method ours|magcal|both] [--plot OUT.png]`
with a Python that has numpy.

**Synthetic streams:** 10 Hz, R = 50 µT, noise 0.4 µT, 1 % spikes. Each starts
with 10 s still, then 30 s of yaw only, then free tumbling. Results over 4 seeds:

| Scenario | Ours | magcal (ported for comparison) |
|---|---|---|
| clean | converges at 70–87 s; offset ≤ 0.11 µT, shape ≤ 0.003, R ≤ 0.05 % | never converges; offset 5–120 µT off, R 11–450 % off |
| 10 % / 30 % outliers | converges at 72–95 s / 95–155 s, same accuracy | fails |
| partial coverage only | correctly never converges | – |
| environment change at 170 s | detected at 186 s; re-converges at 226–239 s | – |
| offset 82 µT | converges at 69–87 s; offset ≤ 0.09 µT | 84 % of samples in one sign-octant; offset 15–20 µT off |

Why magcal fails:

- Its early searches run on arc-only data, so its matrix hits its bound and R
  inflates.
- It has no spike filter.
- Its octant bins collapse when the offset is comparable to the field.
- Its unsymmetric matrix rotates the axes 1–11°, which is a heading error.

**Real bench recording** (`tools/recordings/mag-bench-1.csv`, IST8310 on a
desk with metal nearby). It correctly does not converge:

- It reached 81 of 100 cells and all six axis caps, with a 2.6 % fit error
  against the 1.5 % limit.
- The residual depends on time, not direction: +7 % in the first 80 s, −2 %
  later, including while the module was still. The field changed as the module
  moved over the desk, so the data isn't one sphere.
- With relaxed limits (80 cells, 3 %) it gives o = (16.2, 7.4, −9.5) µT and
  R = 57.9 µT.
- The stationary tail doesn't bias it.

**What the bad readings actually are** (measured 2026-09-12 on
`mag-bench-1.csv`, 2042 readings of fast hand rotation, and on 1092 readings of
a slow flat spin):

| Measure | Fast hand rotation | Slow flat spin |
|---|---|---|
| Change between consecutive readings, median | 0.67 µT | 0.45 µT |
| … 95th percentile | 36 µT | 2.3 µT |
| Isolated one-reading glitches | rare; the excursions run 1–8 readings long | 0.18 % |
| Readings > 3 MAD off the fitted sphere | 19.5 % | 8.5 % |

So most off-sphere readings are not glitches: they come from the axes being
sampled at slightly different times during fast rotation, and from the field
changing as the module moves near metal. §3.1 is built around that split.

**Interim firmware values.** `main/mag_cal.c` departs from the plan in a few
places, all in the `#define` block at the top of that file:

| Setting | Plan (§2, §3) | Firmware today |
|---|---|---|
| Coverage | ≥ 85 of 100 cells holding ≥ 2 readings that agree | 90 % of cells with ≥ 2 readings; agreement not checked |
| Fit error | ≤ 1.5 %, or 3 × noise / R | ≤ 2.5 % flat: there is no noise estimate yet |
| Saturation | the IST8310's per-axis limits (±1600 / ±2500 µT) | a flat 1000 µT for every chip |
| Completion | the criteria of §2 | those, plus ≥ 100 new readings in the session, so a continued session always brings in fresh data |

Measured on the device (ESP32-C3 at 160 MHz, no FPU, 268 readings): a fit from
cold takes 2.1 s and one continued from the previous fit 0.36 s, so `mag_cal.c`
refits at most every 3 s, always passing the previous fit, and always with the
lock released — otherwise every HTTP request would wait behind it. The fit uses
2.7 KB of stack, which is why the calibration task has 6 KB.

**Changes adopted from testing:**

1. **Only a plausible fit may define the cells**: R 10–140 µT, \|o\| ≤ 500 µT,
   W eigenvalues 0.5–2. A re-bin that would evict more than half the buffer is
   refused.
   - Without this, an arc-only phase fits a huge sphere (R 200–560 µT was
     seen), every sample bins to one or two cells, and it never recovers.
   - This rule is also in the firmware.
2. **Before the first fit,** samples near the min/max midpoint are binned by
   their raw direction.
3. **Each refit tries several starts:** continue from the previous fit, Kåsa,
   and RANSAC when inliers fall below 70 %. It keeps the one with the most
   inliers.
4. **Noise is estimated from second differences** over the quietest 20-sample
   window, which cancels steady rotation.
5. **Monitoring window 300 samples, restart at 50 % outliers** (was 100 and
   30 %). Short disturbance bursts were restarting good calibrations.
6. **Spike-filter floor 2 µT** (was 0.5). Fast hand rotation was being filtered.
7. **Once coverage passes,** a new sample further than max(2 × gate, 3 %) from
   the current fit isn't inserted.
8. **Model levels:**
   - diagonal needs ≥ 35 cells, scatter ≥ 0.08 and ≥ 100 samples;
   - full needs ≥ 60 cells, scatter ≥ 0.15, every cap ≥ 2 and ≥ 150 samples.
9. **A newer fit replaces the stored one** only if it's ≥ 10 % better on the
   current buffer. After a restart, the old calibration stays marked stale
   until the new one converges.
10. **Recording tips:**
    - hold still for about 3 s at the start (for the noise estimate);
    - rotate in the air, away from metal;
    - fill the missing directions.

    The 1.5 % fit limit is only reachable away from the desk.
