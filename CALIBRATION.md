# Magnetometer calibration — proposal

Status: **proposal, not implemented.** Thresholds marked *(tune)* are starting
values, to be checked against real IST8310 data before implementation.

## 1. What is being calibrated

Raw readings `m` (µT, sensor axes) are distorted by the module itself and by
anything mounted rigidly with it:

- **hard iron**: a constant offset `o` (magnetised parts, DC currents);
- **soft iron and gain mismatch**: a linear distortion that turns the sphere into
  an ellipsoid.

Calibrated field:

```
b = W · (m − o)          o: 3 offsets, W: symmetric 3×3 (6 values)
goal: |b| = R            R: local field magnitude
```

That's 9 parameters plus `R`. An undistorted sensor rotated through every
orientation traces a sphere of radius `R` centred on the origin. A distorted one
traces an off-centre ellipsoid. Calibration finds the `o` and `W` that map that
ellipsoid back onto the sphere. The model is the same one ArduPilot uses
(offset, diagonal, off-diagonal, radius).

Constraints of this device:

- **Magnetometer only, no accelerometer or gyro.** Samples can't be sorted by
  device orientation, only by the direction of the magnetic vector itself,
  relative to the current centre estimate. Heading needs the module held level.
- **GNSS is available.** With a fix, the World Magnetic Model gives the expected
  local field strength (about 50 µT in Europe). That's an absolute check on `R`,
  or a way to scale the output to true µT.
- **The ESP32-C3 has no FPU.** All maths is software floating point, so the
  sample buffer is kept small (≤ 400 samples) and fits run in a low-priority task.
- **Calibration is only valid for one mounting.** Do it with the module in its
  final enclosure, away from the laptop. The current raw reading of about 76 µT
  on the bench is mostly hard iron.

## 2. Answers to your questions

### 2.1 What makes a "good" sphere?

No single number captures it; five independent properties must all hold:

| # | Property | Measured as | Proposed pass *(tune)* |
|---|----------|-------------|------------------------|
| 1 | **Coverage**: samples reach all directions | inlier samples in 100 equal-area cells (MotionCal's partition) | ≥ 85 cells occupied, each ±X/±Y/±Z cap (45°) has ≥ 5 samples, and the smallest eigenvalue of the direction scatter `mean(u·uᵀ)` ≥ 0.2 (a uniform sphere gives 1/3) |
| 2 | **Fit error**: points lie on the sphere | RMS of `|b|/R − 1` over inliers | ≤ 1.5 %, and no tighter than 3 × measured noise / R |
| 3 | **Wobble**: no one-sided bias | length of the mean of per-cell average residual vectors, / R (MotionCal's "wobble") | ≤ 2 % |
| 4 | **Plausibility**: parameters are physical | R; eigenvalues of W normalised to det W = 1; \|o\| | R 20–70 µT (as in CHRE); with a GNSS fix, R within ±15 % of WMM; W eigenvalues within 0.67–1.5; \|o\| ≤ 500 µT |
| 5 | **Stability**: the answer stopped moving | change between consecutive fits | 3 fits in a row with \|Δo\| ≤ 0.5 µT, max \|ΔWᵢⱼ\| ≤ 0.01, \|ΔR\|/R ≤ 0.5 % |

Plus: inliers make up ≥ 90 % of the buffer.

Why each is needed:

- **Coverage.** A low fit error on a small arc of the sphere proves nothing:
  many ellipsoids pass through the same arc. The eigenvalue test catches the
  common failure of rotating in one plane only.
- **Wobble.** Catches a fit dragged toward the densely sampled side.
- **Stability.** Guards against stopping on a lucky fit.

### 2.2 How can outliers be excluded before the parameters are known?

An outlier test doesn't need the final parameters. It needs a model good enough
that most samples agree with it, plus a threshold taken from the data itself.
Three layers, from no knowledge to full knowledge:

1. **Rejection that needs no calibration at all, at intake:**
   - read errors and saturation (the IST8310's range is ±1600 µT on X/Y and
     ±2500 µT on Z);
   - spikes: a Hampel filter per axis over a 5-sample window. A sample more than
     5 × the MAD from the window median is dropped. Genuine rotation moves the
     vector smoothly, while electrical glitches jump and come back.
2. **Robust fitting, where the threshold comes from the data:**
   - **RANSAC start** (optional, used when the plain start fails). Fit a sphere
     through 4 random samples, count the samples within tolerance of it, repeat
     50–200 times and keep the best. This needs no prior knowledge and survives
     up to about 50 % bad data.
   - **Iteratively reweighted fitting.** Compute each sample's residual under the
     current fit and its spread `s = 1.4826 · MAD(residuals)`. Give weight 0
     beyond 3·s (or use Tukey weights), refit, and repeat until the inlier set
     stops changing, usually within 3–5 passes.
   - This breaks the chicken-and-egg loop because the median and MAD stay correct
     as long as more than half the samples are good, however rough the current
     parameters are. Each refit sharpens the residuals, which sharpens the gate.
3. **Nothing is deleted for being an outlier while calibration is running.**
   - The outlier flag is recomputed on every fit. A sample wrongly rejected under
     an early, poor model comes back once the model improves.
   - Samples leave the buffer only through the buffer policy (§3.2).
     Worst-residual-first eviction switches on only once coverage is good, as
     MotionCal does.
   - A changed environment is not an outlier problem. If most new samples
     (> 30 % of the last 100) are outliers under an accepted calibration, the
     environment has changed (a new mounting, or a battery nearby). Restart the
     calibration instead of rejecting the samples.

### 2.3 Continuous or in iterations?

Both, as two modes sharing the same code. Within each mode, fits run as
**repeated batch refits over a rolling buffer**, not per-sample recursive
updates:

| Mode | Started by | Behaviour |
|------|------------|-----------|
| **Guided calibration** | a button on the web page | You rotate the module. Every ~2 s, or every 20 new samples, the firmware refits and reports coverage, fit error and status to the page. It stops when §2.1 holds on 3 consecutive fits, then saves to NVS. It fails with a reason after 3 minutes. |
| **Background monitoring** | always on once a calibration exists | Each new sample is checked against the stored calibration, which is cheap. The buffer keeps refreshing. A refit runs in the background when drift shows (median `|b|/R − 1` of the last 100 samples > 3 %) or every few minutes. The new result replaces the stored one only if it passes §2.1 and beats the old calibration on the same current buffer. |

The adoption rule borrows NXP's idea (the algorithm MotionCal uses): the stored
calibration's error slowly ages, so an old good calibration can't lock out a
newer one forever.

Why batches rather than per-sample (recursive least squares, gradient descent):

- outlier gating and coverage checks both need a set of samples;
- a batch lets a wrongly rejected sample come back later;
- a recursive filter absorbs a bad sample into the estimate for good.

A 400-sample batch is cheap enough (§3.6).

## 3. The algorithm

### 3.1 Intake (10 Hz from `magnetometer.c`)

1. Reject read errors and saturated samples.
2. Run the Hampel spike filter (§2.2, layer 1).
3. Estimate sensor noise as the MAD of first differences while the module is
   still. This sets the noise floor for the fit-error threshold.
4. Pass the sample to the buffer.

### 3.2 Buffer

- **100 equal-area cells**, MotionCal's partition: 1 polar cap, 15, 34, 34, 15,
  1 polar cap.
- A sample's cell comes from its direction `u = (m − ô) / |m − ô|` about the
  current centre estimate `ô`.
- Up to **K = 4 samples per cell**, oldest first out. At most 400 samples × 3 ×
  int16 = 2.4 KB.
- This cap stops a module lying still from piling hundreds of identical
  samples into one spot and biasing the fit. ArduPilot and PX4 get the same
  effect with a minimum distance between samples; this is cheaper and gives the
  coverage map for free.
- Before the first fit, `ô` is the midpoint of the per-axis min/max of the
  filtered samples.
- When a fit moves `ô` by more than 5 % of R, all samples are re-binned.
- Once coverage passes, eviction also drops the worst-residual sample, rate-
  limited as in MotionCal.

### 3.3 Fit pipeline (on each trigger)

1. With fewer than 40 samples, skip. NXP uses 40 / 100 / 150 samples as the
   minimum for 4 / 7 / 10 parameters.
2. **Initial sphere (4 parameters: o, R).** A Kåsa linear least-squares fit,
   one small linear solve. Fall back to RANSAC if the previous result failed or
   had fewer than 70 % inliers.
3. **Robust sphere.** Gauss-Newton on the geometric residual
   `rᵢ = |mᵢ − o| − R`, reweighted as in §2.2.
4. **Ellipsoid (o, 3 diagonal + 3 off-diagonal values of W, R).** Only once
   coverage passes and there are ≥ 150 inliers. Levenberg-Marquardt starting
   from the sphere, with the same reweighting. This is ArduPilot's
   parameterisation, which avoids the pitfalls of converting a general quadric.
   With weaker coverage, fall back to a diagonal-only W (7 parameters, NXP's
   middle solver) or stay with the sphere.
5. **Normalise.** Scale W to det W = 1, so R carries the sensor gain and can be
   compared with the WMM. For output in true µT, scale by `B_WMM / R` when a
   GNSS fix exists.
6. **Evaluate §2.1** and set the status.

### 3.4 States

```
IDLE ──start──▶ COLLECTING ──≥40 samples──▶ FITTING_SPHERE ──coverage──▶ FITTING_ELLIPSOID
                    ▲                              │                            │
                    └──────── more samples ◀───────┴──── criteria not met ◀──────┤
                                                                                │ §2.1 × 3 fits
FAILED(reason) ◀── timeout / implausible                            CONVERGED ──▶ save to NVS
                                                                        │
                                                                        ▼
                                                              MONITORING (background)
```

Failure reasons, like ArduPilot's: `COVERAGE`, `RESIDUALS_HIGH`, `RADIUS`,
`OFFSETS`, `SCALING`, `UNSTABLE`, `TIMEOUT`.

### 3.5 Output and storage

- **NVS record:** `{version, chip, o[3], W[6], R, fit_error, coverage, time}`.
- **`magnetometer.c`** applies `b = W(m − o)` and reports raw and calibrated values.
- **Status JSON:** gains a `cal` object with state, coverage, fit error, inlier
  count and the 100-cell occupancy map for the page.

### 3.6 Compute budget (rough, 160 MHz, software floating point)

| Step | Estimate |
|------|----------|
| One LM iteration, 400 samples × 10 parameters | about 50 k float operations, ≈ 20–30 ms |
| Full fit (≈ 10 iterations) | ≈ 0.3 s |
| RANSAC, 100 hypotheses × 400 samples | ≈ 0.2 s |

This is fine in a low-priority task refitting every few seconds.

## 4. Existing implementations

Is there already an iterative, continuous calibration with outlier exclusion?
**Not as one package.** Each open implementation covers part of it:

| Implementation | Mode | Fit | Coverage handling | Outlier handling | Accept test |
|---|---|---|---|---|---|
| [ArduPilot `CompassCalibrator`](https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Compass/CompassCalibrator.cpp) (GPL-3.0) | one-shot, guided | LM: sphere (10 steps), then sphere → ellipsoid (up to 35 steps) | a sample is accepted only if far enough from all others (minimum spacing derived from polyhedron geometry); the buffer is thinned between steps | **none by residual** | RMS residual ≤ tolerance, radius 150–950 mGauss, offsets < limit, diagonal 0.2–5, off-diagonal < 1 |
| [PX4 `mag_calibration`](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/commander/mag_calibration.cpp) (BSD-3) | one-shot, guided through 6 sides | LM sphere/ellipsoid (`lm_fit.hpp`) | minimum distance between samples | none found | fit success |
| [NXP sensor fusion, as used in MotionCal](https://github.com/PaulStoffregen/MotionCal) | **continuous** | eigen solvers with 4 / 7 / 10 parameters (min 40 / 100 / 150 samples) | 650-sample buffer; while coverage is poor, one of the two closest samples is dropped | **once coverage is good, periodically drops the sample furthest from the fitted field strength** | fit error %, aged ×1.02 per run so newer fits can win; quality metrics gaps / variance / wobble / fit error |
| [Android CHRE `mag_cal`](https://android.googlesource.com/device/google/contexthub/+/refs/heads/main/firmware/os/algos/calibration/magnetometer/mag_cal/mag_cal.c) (Apache-2.0) | **continuous**, in batches | Kåsa sphere, **hard iron only** | diversity checker: minimum spacing, max-distance violations | **per batch**: a whole batch is dropped if the sample covariance fails (eigenvalue ratio < 15, magnitude 20–70 µT) or the spread of the corrected norms is too large | radius 20–70 µT, norm spread and variance |
| [ysoldak/magcal](https://github.com/ysoldak/magcal) (TinyGo) | **continuous** | gradient descent | buffer kept diverse by quadrant | a refit is triggered when a vector's length deviates from the target | best error so far |

None of them uses robust statistics (RANSAC, or MAD-gated reweighting), and
none combines a formal multi-criteria stop with continuous refinement. The
proposal takes:

- from ArduPilot: the model, the sphere → ellipsoid LM staging and the
  plausibility bounds;
- from MotionCal/NXP: the equal-area cells, the quality metrics, worst-first
  eviction and the aged fit error;
- from CHRE: the covariance eigenvalue test;
- robust statistics: new.

Licences: ArduPilot is GPL-3.0, so copying its code would make this firmware
GPL. PX4 (BSD-3) and CHRE (Apache-2.0) are permissive. The plan here is a
fresh implementation of the ideas.

## 5. Decisions needed

1. **Trigger and scope.** A guided-calibration button on the WiFi/GNSS page? Is
   background monitoring on by default?
2. **Tuning data first?** Proposed: before implementing, add a raw-sample
   stream (for example `/api/mag/raw`), record a few rotations of the real
   module, and tune §2.1 offline in Python. The same script becomes the
   reference implementation to test the C code against.
3. **WMM on the device** (a few KB of coefficients plus code) for the absolute
   R check and true-µT scaling, or only the 20–70 µT plausibility range?
4. **Parameters:** full 9 (symmetric W), or start with offsets plus diagonal
   gains (6) and add the off-diagonal terms only if the data needs them?
