#!/usr/bin/env python3
"""Offline reference implementation of the magnetometer calibration proposed in
CALIBRATION.md: tune it on recorded data here, then port it to C.

Usage:
  mag_calibrate.py RECORDING.csv [--method ours|magcal|both] [--plot OUT.png]
                                 [--progress all|changes|none]
  mag_calibrate.py --synthetic NAME|all [--method ours|magcal|both] [--seed N]
                                 [--plot OUT.png] [--save-csv OUT.csv]

RECORDING.csv has the header seq,t_ms,x_ut,y_ut,z_ut (device clock in ms, raw
field in µT, sensor axes), as written by tools/mag_record.py. The stream is
replayed at its recorded timestamps, without waiting.

--synthetic generates 10 Hz streams with a known truth and checks the result.
Scenarios: clean, outliers10, outliers30, partial, envchange, bigoffset.

--method magcal runs a port of ysoldak/magcal (BSD-3, Go) as a baseline on the
same stream; "both" compares the two.

Model
  b = W · (m − o)
  m: raw reading; o: hard-iron offset (3); W: symmetric soft-iron matrix (6);
  R = |b| on a perfect sphere. The fit uses a unit-scale W, |W (m − o)| ≈ 1;
  the reported W is scaled to det W = 1, so R carries the sensor gain.
  Model levels, picked by coverage: sphere (o + 1 scale), diagonal (o + 3
  scales), full (o + symmetric W): 4 / 6 / 9 free parameters, or 4 / 7 / 10
  counting R separately as CALIBRATION.md does.

Direction cells (the web page uses the same partition)
  100 cells of equal area on the unit sphere of directions
  u = W (m − o) / |W (m − o)| under the current fit (before the first fit:
  m − the midpoint of the per-axis min/max). Archimedes' hat-box theorem makes
  a band's area proportional to its height in z, so bands giving each cell
  Δz = 0.02 are equal-area:

    band  z range              cells  first cell
    0      0.98 <= z             1      0
    1      0.68 <= z <  0.98    15      1
    2      0    <= z <  0.68    34     16
    3     -0.68 <= z <  0       34     50
    4     -0.98 <= z < -0.68    15     84
    5             z < -0.98      1     99

  Within a band, cells split the longitude evenly, starting at +X and turning
  towards +Y: cell = first + floor(lon / 2π · cells), lon = atan2(y, x) in [0, 2π).

The code keeps to small, stage-by-stage functions over fixed-size arrays (buffer
≤ 400 samples, at most 9×9 linear solves) so it maps onto C. numpy only.
"""

import argparse
import csv
import math
import sys
from collections import deque
from pathlib import Path

import numpy as np

# ---------------------------------------------------------------------------
# Tunables. Thresholds from CALIBRATION.md unless noted.

# Direction cells
CELLS_PER_BAND = (1, 15, 34, 34, 15, 1)
BAND_FLOOR_Z = (0.98, 0.68, 0.0, -0.68, -0.98)  # lowest z of bands 0-4; band 5 takes the rest
BAND_FIRST_CELL = (0, 1, 16, 50, 84, 99)
N_CELLS = 100
CAPS = (("+X", (1, 0, 0)), ("-X", (-1, 0, 0)), ("+Y", (0, 1, 0)),
        ("-Y", (0, -1, 0)), ("+Z", (0, 0, 1)), ("-Z", (0, 0, -1)))
CAP_COS = math.cos(math.radians(45))

# Intake
SATURATION_UT = np.array([1590.0, 1590.0, 2490.0])  # IST8310 range, less a margin
HAMPEL_HALF = 2                # 5-sample window
HAMPEL_SIGMAS = 5.0
HAMPEL_FLOOR_UT = 2.0          # MAD floor. 0.5 µT (still-signal noise) dropped genuine samples at
                               # the turning points of fast hand rotation in the IST8310 recording
NOISE_WINDOW = 20
NOISE_DEFAULT_UT = 0.5         # until a quiet window has been seen
NOISE_FLOOR_UT = 0.1
PRE_FIT_STILL_UT = 5.0         # before the first fit, samples this close to the min/max
                               # midpoint are binned by their raw direction instead

# Buffer
K_PER_CELL = 4
BUFFER_SIZE = N_CELLS * K_PER_CELL

# Fitting
REFIT_EVERY = 20               # accepted samples between fits
MIN_SAMPLES = 40
MIN_FIT_SAMPLES = 20
IRLS_PASSES = 6
GATE_SIGMAS = 3.0              # inlier gate: 3 robust sigmas of the relative residual
LM_ITERATIONS = 20
RANSAC_TRIALS = 100
RANSAC_TOLERANCE = 0.1         # relative; wide because a sphere can't follow soft iron
REINIT_INLIER_FRACTION = 0.7
INSERT_GATE_FACTOR = 2.0       # once coverage passes, new samples beyond 2 gates are refused
INSERT_GATE_MIN = 0.03
LEVEL_DIAGONAL = dict(cells=35, scatter=0.08, samples=100)
LEVEL_FULL = dict(cells=60, scatter=0.15, cap=2, samples=150)

# "Good sphere" criteria
COVER_CELLS = 85
COVER_CAP = 5
COVER_SCATTER = 0.2
FIT_ERROR_MAX = 0.015
FIT_NOISE_SIGMAS = 3.0
WOBBLE_MAX = 0.02
R_MIN, R_MAX = 20.0, 70.0
W_EIG_MIN, W_EIG_MAX = 0.67, 1.5
OFFSET_MAX = 500.0
STABLE_FITS = 3
STABLE_OFFSET_UT = 0.5
STABLE_W = 0.01
STABLE_R = 0.005
INLIER_MIN = 0.90

# Monitoring after convergence
# Changed from CALIBRATION.md (100 samples, 30 %): disturbance bursts of a few
# seconds (a hand near a magnet) filled a 10 s window past 30 % and restarted a
# good calibration. A real change puts nearly every sample off, so 30 s and a
# majority are both safe.
MONITOR_WINDOW = 300
MONITOR_RESTART = 0.50         # fraction of outliers in the window that means "environment changed"
MONITOR_OUTLIER_MIN = 0.05     # relative residual counted as an outlier, at least
MONITOR_DRIFT = 0.03
ADOPT_RATIO = 0.9              # a newer converged fit replaces the stored one if 10 % better

SPHERE, DIAGONAL, FULL = 0, 1, 2
LEVEL_NAMES = ("sphere", "diagonal", "full")


# ---------------------------------------------------------------------------
# Direction cells

def cell_of(u):
    """Cell index 0-99 of a unit direction; see the module docstring."""
    band = 5
    for b, floor in enumerate(BAND_FLOOR_Z):
        if u[2] >= floor:
            band = b
            break
    lon = math.atan2(u[1], u[0])
    if lon < 0:
        lon += 2 * math.pi
    n = CELLS_PER_BAND[band]
    return BAND_FIRST_CELL[band] + min(int(lon / (2 * math.pi) * n), n - 1)


def cells_of(u):
    return np.array([cell_of(x) for x in u], dtype=int)


# ---------------------------------------------------------------------------
# Model

class Model:
    """o and a unit-scale symmetric W: |W (m − o)| ≈ 1 on the sphere."""

    __slots__ = ("o", "W", "level")

    def __init__(self, o, W, level):
        self.o = np.asarray(o, dtype=float)
        self.W = np.asarray(W, dtype=float)
        self.level = level

    def radius(self):
        return float(np.linalg.det(self.W)) ** (-1.0 / 3.0)

    def w_norm(self):
        """W scaled to det 1; b = w_norm · (m − o) has |b| ≈ R."""
        return self.W * self.radius()

    def unit_norms(self, m):
        return np.linalg.norm((m - self.o) @ self.W, axis=1)  # W is symmetric

    def residuals(self, m):
        """Relative residual |b| / R − 1."""
        return self.unit_norms(m) - 1.0

    def directions(self, m):
        v = (m - self.o) @ self.W
        n = np.linalg.norm(v, axis=1)
        n[n == 0] = 1.0
        return v / n[:, None]

    def calibrate(self, m):
        return (m - self.o) @ self.w_norm()


def positive_definite(W):
    return bool(np.all(np.isfinite(W))) and np.linalg.eigvalsh(W)[0] > 0


def pack(model, level):
    o, W = model.o, model.W
    if level == SPHERE:
        return np.r_[o, abs(np.linalg.det(W)) ** (1.0 / 3.0)]
    if level == DIAGONAL:
        return np.r_[o, np.diag(W)]
    return np.r_[o, W[0, 0], W[1, 1], W[2, 2], W[0, 1], W[0, 2], W[1, 2]]


def unpack(p, level):
    if level == SPHERE:
        W = np.eye(3) * p[3]
    elif level == DIAGONAL:
        W = np.diag(p[3:6])
    else:
        W = np.array([[p[3], p[6], p[7]], [p[6], p[4], p[8]], [p[7], p[8], p[5]]])
    return Model(p[:3].copy(), W, level)


def residuals_jacobian(p, level, m):
    """Residuals |W (m − o)| − 1 and their Jacobian for the packed parameters."""
    model = unpack(p, level)
    d = m - model.o
    v = d @ model.W
    n = np.maximum(np.linalg.norm(v, axis=1), 1e-9)
    J = np.empty((len(m), len(p)))
    J[:, :3] = -(v @ model.W) / n[:, None]          # dn/do = −W v / n
    if level == SPHERE:
        J[:, 3] = n / p[3]                          # W = s·I: dn/ds = |d|
    else:
        J[:, 3:6] = v * d / n[:, None]              # dn/dW_jj = v_j d_j / n
        if level == FULL:                           # dn/dW_jk = (v_j d_k + v_k d_j) / n
            J[:, 6] = (v[:, 0] * d[:, 1] + v[:, 1] * d[:, 0]) / n
            J[:, 7] = (v[:, 0] * d[:, 2] + v[:, 2] * d[:, 0]) / n
            J[:, 8] = (v[:, 1] * d[:, 2] + v[:, 2] * d[:, 1]) / n
    return n - 1.0, J


def levenberg_marquardt(model, m, level):
    """Geometric fit of |W (m − o)| = 1; None if it ends up not positive definite."""
    p = pack(model, level)
    r, J = residuals_jacobian(p, level, m)
    cost = r @ r
    lam = 1e-3
    for _ in range(LM_ITERATIONS):
        A = J.T @ J
        g = J.T @ r
        try:
            step = np.linalg.solve(A + lam * np.diag(np.diag(A) + 1e-12), -g)
        except np.linalg.LinAlgError:
            lam *= 10
            continue
        p_try = p + step
        r_try, J_try = residuals_jacobian(p_try, level, m)
        cost_try = r_try @ r_try
        if np.isfinite(cost_try) and cost_try < cost:
            converged = cost - cost_try < 1e-7 * cost
            p, r, J, cost = p_try, r_try, J_try, cost_try
            lam = max(lam / 3, 1e-9)
            if converged:
                break
        else:
            lam *= 4
            if lam > 1e6:
                break
    fitted = unpack(p, level)
    return fitted if positive_definite(fitted.W) else None


def kasa_sphere(m):
    """Linear least-squares sphere |m − c|² = r², on centred data for float32 safety."""
    if len(m) < 4:
        return None
    mean = m.mean(axis=0)
    d = m - mean
    A = np.column_stack([2 * d, np.ones(len(d))])
    y = (d * d).sum(axis=1)
    try:
        sol = np.linalg.solve(A.T @ A, A.T @ y)
    except np.linalg.LinAlgError:
        return None
    c = sol[:3]
    r2 = sol[3] + c @ c
    if not np.isfinite(r2) or r2 <= 0:
        return None
    return Model(c + mean, np.eye(3) / math.sqrt(r2), SPHERE)


def ransac_sphere(m, rng):
    """Sphere through 4 random samples, best of RANSAC_TRIALS by inlier count."""
    best, best_count = None, -1
    for _ in range(RANSAC_TRIALS):
        cand = kasa_sphere(m[rng.choice(len(m), 4, replace=False)])
        if cand is None or not (R_MIN / 2 <= cand.radius() <= R_MAX * 2):
            continue
        count = int((np.abs(cand.residuals(m)) < RANSAC_TOLERANCE).sum())
        if count > best_count:
            best, best_count = cand, count
    if best is None:
        return None
    return kasa_sphere(m[np.abs(best.residuals(m)) < RANSAC_TOLERANCE]) or best


def gate_inliers(r, noise_rel):
    """Inliers within GATE_SIGMAS robust sigmas (MAD) of the median residual."""
    med = np.median(r)
    spread = 1.4826 * np.median(np.abs(r - med))
    gate = GATE_SIGMAS * max(spread, noise_rel)
    return np.abs(r - med) <= gate, gate


class Fit:
    __slots__ = ("model", "mask", "gate")

    def __init__(self, model, mask, gate):
        self.model, self.mask, self.gate = model, mask, gate

    def rms(self, m):
        r = self.model.residuals(m[self.mask])
        return float(np.sqrt(np.mean(r * r))) if len(r) else float("inf")


def robust_fit(model, m, level, noise_ut):
    """Refit on the inliers until the inlier set stops changing. Samples are
    re-gated on every pass, so one rejected under a poor early model comes back."""
    mask = None
    for _ in range(IRLS_PASSES):
        new_mask, _ = gate_inliers(model.residuals(m), noise_ut / model.radius())
        if mask is not None and np.array_equal(new_mask, mask):
            break
        mask = new_mask
        if mask.sum() < MIN_FIT_SAMPLES:
            return None
        fitted = levenberg_marquardt(model, m[mask], level)
        if fitted is None:
            return None
        model = fitted
    mask, gate = gate_inliers(model.residuals(m), noise_ut / model.radius())
    return Fit(model, mask, gate)


def coverage(model, m):
    """Occupied cells, per-cap counts and the smallest eigenvalue of mean(u·uᵀ)."""
    if len(m) == 0:
        return 0, [0] * 6, 0.0
    u = model.directions(m)
    occupied = len(set(cells_of(u)))
    caps = [int((u @ np.array(axis) >= CAP_COS).sum()) for _, axis in CAPS]
    scatter = float(np.linalg.eigvalsh(u.T @ u / len(u))[0])
    return occupied, caps, scatter


def choose_level(occupied, caps, scatter, samples):
    if (occupied >= LEVEL_FULL["cells"] and scatter >= LEVEL_FULL["scatter"]
            and min(caps) >= LEVEL_FULL["cap"] and samples >= LEVEL_FULL["samples"]):
        return FULL
    if (occupied >= LEVEL_DIAGONAL["cells"] and scatter >= LEVEL_DIAGONAL["scatter"]
            and samples >= LEVEL_DIAGONAL["samples"]):
        return DIAGONAL
    return SPHERE


def wobble(model, m):
    """|mean over occupied cells of the cell's mean radial residual vector r·u|."""
    if len(m) == 0:
        return float("inf")
    u = model.directions(m)
    r = model.residuals(m)
    cells = cells_of(u)
    total = np.zeros(3)
    occupied = 0
    for c in np.unique(cells):
        sel = cells == c
        total += (r[sel, None] * u[sel]).mean(axis=0)
        occupied += 1
    return float(np.linalg.norm(total / occupied))


def plausible_for_binning(model):
    """Loose sanity check before a fit may define the direction cells. An arc of
    data fits a huge sphere well; binning by such a fit sends every sample to the
    same cell, the 4-per-cell cap then empties the buffer, and it never recovers."""
    R = model.radius()
    eig = np.linalg.eigvalsh(model.w_norm())
    return (R_MIN / 2 <= R <= R_MAX * 2 and np.linalg.norm(model.o) <= OFFSET_MAX
            and eig[0] >= 0.5 and eig[-1] <= 2.0)


def fit_limit(noise_ut, radius):
    return max(FIT_ERROR_MAX, FIT_NOISE_SIGMAS * noise_ut / radius)


def evaluate(fit, m, noise_ut):
    """The CALIBRATION.md criteria except stability: [(name, ok, value, limit)], metrics."""
    model = fit.model
    inl = m[fit.mask]
    R = model.radius()
    occupied, caps, scatter = coverage(model, inl)
    err = fit.rms(m)
    limit = fit_limit(noise_ut, R)
    eig = np.linalg.eigvalsh(model.w_norm())
    offset = float(np.linalg.norm(model.o))
    frac = float(fit.mask.mean())
    wob = wobble(model, inl)
    missing = [name for (name, _), n in zip(CAPS, caps) if n < COVER_CAP]
    crit = [
        ("model", model.level == FULL, LEVEL_NAMES[model.level], "full"),
        ("coverage", occupied >= COVER_CELLS and not missing and scatter >= COVER_SCATTER,
         f"{occupied} cells, caps min {min(caps)}, scatter {scatter:.2f}",
         f">= {COVER_CELLS}, >= {COVER_CAP}, >= {COVER_SCATTER}"),
        ("fit", err <= limit, f"{err * 100:.2f} %", f"<= {limit * 100:.2f} %"),
        ("wobble", wob <= WOBBLE_MAX, f"{wob * 100:.2f} %", f"<= {WOBBLE_MAX * 100:.0f} %"),
        ("radius", R_MIN <= R <= R_MAX, f"{R:.2f} µT", f"{R_MIN:.0f}-{R_MAX:.0f} µT"),
        ("shape", W_EIG_MIN <= eig[0] and eig[-1] <= W_EIG_MAX, f"eig {eig[0]:.3f}-{eig[-1]:.3f}",
         f"{W_EIG_MIN}-{W_EIG_MAX}"),
        ("offset", offset <= OFFSET_MAX, f"{offset:.1f} µT", f"<= {OFFSET_MAX:.0f} µT"),
        ("inliers", frac >= INLIER_MIN, f"{frac * 100:.1f} %", f">= {INLIER_MIN * 100:.0f} %"),
    ]
    metrics = dict(cells=occupied, missing=missing, scatter=scatter, fit=err, limit=limit,
                   inliers=frac, level=model.level, radius=R)
    return crit, metrics


# ---------------------------------------------------------------------------
# Our calibrator: streaming intake, cell buffer, periodic robust refits,
# criteria, and monitoring after convergence.

class Calibrator:
    name = "ours"

    def __init__(self, seed=1, progress="all", out=print):
        self.rng = np.random.default_rng(seed)
        self.progress, self.out = progress, out
        self.buf_m = np.zeros((BUFFER_SIZE, 3))
        self.buf_t = np.zeros(BUFFER_SIZE)
        self.buf_cell = np.full(BUFFER_SIZE, -1)
        self.buf_used = np.zeros(BUFFER_SIZE, dtype=bool)
        self.window = deque(maxlen=2 * HAMPEL_HALF + 1)
        self.noise_recent = deque(maxlen=NOISE_WINDOW)
        self.noise_ut = None
        self.monitor_flags = deque(maxlen=MONITOR_WINDOW)
        self.monitor_res = deque(maxlen=MONITOR_WINDOW)
        self.t0 = None
        self.seen = self.spikes = self.refused = self.restarts = 0
        self.stream = []        # (t, m, dropped) for the report and plot
        self.timeline = []      # (t, cells, fit, inliers, level, state)
        self.events = []        # (t, text)
        self.accepted = None    # stored calibration: Model
        self.accepted_t = None
        self._reset_run()

    def _reset_run(self):
        self.buf_used[:] = False
        self.lo = self.hi = None
        self.fit = None          # latest fit, reported even when implausible
        self.bin_model = None    # the fit the direction cells are based on
        self.cover_ok = False
        self.history = deque(maxlen=STABLE_FITS)
        self.since_refit = 0
        self.last_crit = None
        self.last_metrics = None
        self.accepted_stale = self.accepted is not None
        self.drift_reported = False
        self.monitor_flags.clear()
        self.monitor_res.clear()

    # ----- intake
    def feed(self, t_ms, m):
        if self.t0 is None:
            self.t0 = t_ms
        t = (t_ms - self.t0) / 1000.0
        m = np.asarray(m, dtype=float)
        self.seen += 1
        if np.any(np.abs(m) >= SATURATION_UT):
            self.spikes += 1
            self.stream.append((t, m, True))
            return
        self.window.append((t, m))
        if len(self.window) < self.window.maxlen:
            return
        t_mid, m_mid = self.window[HAMPEL_HALF]
        block = np.array([x for _, x in self.window])
        med = np.median(block, axis=0)
        mad = 1.4826 * np.median(np.abs(block - med), axis=0)
        if np.any(np.abs(m_mid - med) > HAMPEL_SIGMAS * np.maximum(mad, HAMPEL_FLOOR_UT)):
            self.spikes += 1
            self.stream.append((t_mid, m_mid, True))
            return
        self.stream.append((t_mid, m_mid, False))
        self._take(t_mid, m_mid)

    def finish(self):
        pass

    def _update_noise(self, m):
        """Noise from second differences (they cancel steady rotation), quietest window wins."""
        self.noise_recent.append(m)
        if len(self.noise_recent) == NOISE_WINDOW:
            x = np.array(self.noise_recent)
            d2 = x[2:] - 2 * x[1:-1] + x[:-2]
            sigma = float(np.max(1.4826 * np.median(np.abs(d2), axis=0) / math.sqrt(6)))
            sigma = max(sigma, NOISE_FLOOR_UT)
            self.noise_ut = sigma if self.noise_ut is None else min(self.noise_ut, sigma)

    def noise(self):
        return self.noise_ut if self.noise_ut is not None else NOISE_DEFAULT_UT

    def _take(self, t, m):
        self._update_noise(m)
        ref = self.accepted if (self.accepted is not None and not self.accepted_stale) else (
            self.fit.model if self.cover_ok else None)
        if ref is not None:
            self._monitor(t, m, ref)  # may restart, which clears the extents below
        self.lo = m.copy() if self.lo is None else np.minimum(self.lo, m)
        self.hi = m.copy() if self.hi is None else np.maximum(self.hi, m)

        if self.cover_ok:
            r = abs(self.fit.model.residuals(m[None])[0])
            if r > max(INSERT_GATE_FACTOR * self.fit.gate, INSERT_GATE_MIN):
                self.refused += 1
                return
        self._insert(t, m)
        self.since_refit += 1
        if self.since_refit >= REFIT_EVERY and self.buf_used.sum() >= MIN_SAMPLES:
            self._refit(t)

    def _monitor(self, t, m, ref):
        r = abs(ref.residuals(m[None])[0])
        limit = max(MONITOR_OUTLIER_MIN, 2 * fit_limit(self.noise(), ref.radius()))
        self.monitor_flags.append(r > limit)
        self.monitor_res.append(r)
        if len(self.monitor_flags) < MONITOR_WINDOW:
            return
        frac = float(np.mean(self.monitor_flags))
        if frac > MONITOR_RESTART:
            self.restarts += 1
            self._event(t, f"environment changed ({frac * 100:.0f} % of the last {MONITOR_WINDOW} "
                           f"samples off the calibration): restarting")
            self._reset_run()
        elif np.median(self.monitor_res) > MONITOR_DRIFT and not self.drift_reported:
            self.drift_reported = True
            self._event(t, f"calibration drifting (median residual {np.median(self.monitor_res) * 100:.1f} %)")

    # ----- buffer
    def _direction(self, m):
        if self.bin_model is not None:
            return self.bin_model.directions(m[None])[0]
        v = m - (self.lo + self.hi) / 2
        if np.linalg.norm(v) < PRE_FIT_STILL_UT:
            v = m  # module not rotated yet: keep the pile-up in one cell
        n = np.linalg.norm(v)
        return v / n if n > 0 else np.array([0.0, 0.0, 1.0])

    def _evictee(self, slots):
        """Worst sample of a full cell if it's an outlier (once coverage passes), else the oldest."""
        if self.cover_ok:
            r = np.abs(self.fit.model.residuals(self.buf_m[slots]))
            if r.max() > self.fit.gate:
                return int(slots[np.argmax(r)])
        return int(slots[np.argmin(self.buf_t[slots])])

    def _insert(self, t, m):
        cell = cell_of(self._direction(m))
        same = np.flatnonzero(self.buf_used & (self.buf_cell == cell))
        slot = int(np.flatnonzero(~self.buf_used)[0]) if len(same) < K_PER_CELL else self._evictee(same)
        self.buf_m[slot], self.buf_t[slot], self.buf_cell[slot], self.buf_used[slot] = m, t, cell, True

    def _survivors(self, model):
        """How many buffered samples would stay if binned by this model."""
        idx = np.flatnonzero(self.buf_used)
        cells = cells_of(model.directions(self.buf_m[idx]))
        return int(np.minimum(np.bincount(cells, minlength=N_CELLS), K_PER_CELL).sum()), len(idx)

    def _rebin(self):
        idx = np.flatnonzero(self.buf_used)
        self.buf_cell[idx] = cells_of(self.bin_model.directions(self.buf_m[idx]))
        for c in np.unique(self.buf_cell[idx]):
            while True:
                same = np.flatnonzero(self.buf_used & (self.buf_cell == c))
                if len(same) <= K_PER_CELL:
                    break
                self.buf_used[self._evictee(same)] = False

    def buffer(self):
        return self.buf_m[self.buf_used]

    # ----- fitting
    def _fit_from(self, start, m):
        noise = self.noise()
        mask, _ = gate_inliers(start.residuals(m), noise / start.radius())
        level = choose_level(*coverage(start, m[mask]), int(mask.sum()))
        fit = None
        while fit is None and level >= SPHERE:
            fit = robust_fit(start, m, level, noise)
            level -= 1
        if fit is None:
            return None
        higher = choose_level(*coverage(fit.model, m[fit.mask]), int(fit.mask.sum()))
        if higher > fit.model.level:
            better = robust_fit(fit.model, m, higher, noise)
            if better is not None and better.mask.sum() >= 0.98 * fit.mask.sum():
                fit = better
        return fit

    @staticmethod
    def _better(a, b, m):
        if a is None:
            return b
        if b is None:
            return a
        ka, kb = int(a.mask.sum()), int(b.mask.sum())
        if ka != kb:
            return a if ka > kb else b
        return a if a.rms(m) <= b.rms(m) else b

    def _fit(self, m):
        tried_kasa = False
        best = None
        prev = self.fit.model if self.fit is not None else None
        if prev is not None and R_MIN / 2 <= prev.radius() <= R_MAX * 2:
            best = self._fit_from(prev, m)
        else:
            tried_kasa = True
            start = kasa_sphere(m)
            best = self._fit_from(start, m) if start is not None else None
        if best is None or best.mask.mean() < REINIT_INLIER_FRACTION:
            starts = [] if tried_kasa else [kasa_sphere(m)]
            starts.append(ransac_sphere(m, self.rng))
            for start in starts:
                if start is not None:
                    best = self._better(best, self._fit_from(start, m), m)
        return best

    def _refit(self, t):
        self.since_refit = 0
        fit = self._fit(self.buffer())
        if fit is None:
            self._record(t, None, "no fit")
            return
        self.fit = fit
        if plausible_for_binning(fit.model):
            kept, total = self._survivors(fit.model)
            if kept >= total / 2:  # a model that would evict half the buffer is not trusted
                self.bin_model = fit.model
                self._rebin()
        m = self.buffer()
        mask, gate = gate_inliers(fit.model.residuals(m), self.noise() / fit.model.radius())
        self.fit = Fit(fit.model, mask, gate)
        crit, metrics = evaluate(self.fit, m, self.noise())
        self.last_crit, self.last_metrics = crit, metrics
        self.cover_ok = next(ok for name, ok, _, _ in crit if name == "coverage")

        static_ok = all(ok for _, ok, _, _ in crit)
        model = self.fit.model
        self.history.append((model.o.copy(), model.w_norm(), model.radius(), static_ok))
        stable = len(self.history) == STABLE_FITS and all(h[3] for h in self.history) and all(
            np.linalg.norm(a[0] - b[0]) <= STABLE_OFFSET_UT
            and np.max(np.abs(a[1] - b[1])) <= STABLE_W
            and abs(a[2] / b[2] - 1) <= STABLE_R
            for a, b in zip(list(self.history)[:-1], list(self.history)[1:]))
        metrics["stable"] = stable

        if static_ok and stable:
            if self.accepted is None or self.accepted_stale:
                what = "converged" if self.accepted is None else "re-converged"
                self._accept(t, model, what)
            else:
                stored = float(np.sqrt(np.mean(self.accepted.residuals(m[mask]) ** 2)))
                if metrics["fit"] <= ADOPT_RATIO * stored:
                    self._accept(t, model, f"improved (fit {metrics['fit'] * 100:.2f} % "
                                           f"vs stored {stored * 100:.2f} %)")
        fails = [name for name, ok, _, _ in crit if not ok]
        if not fails and not stable:
            fails = ["stability"]
        self._record(t, metrics, ",".join(fails) or "-")

    def _accept(self, t, model, what):
        self.accepted = Model(model.o.copy(), model.W.copy(), model.level)
        self.accepted_stale = False
        self.accepted_t = t
        self.monitor_flags.clear()
        self.monitor_res.clear()
        self.drift_reported = False
        self._event(t, f"{what}: o = {fmt_vec(model.o)} µT, R = {model.radius():.2f} µT")

    def state(self):
        if self.accepted is not None and not self.accepted_stale:
            return "converged"
        return "fitting" if self.buf_used.sum() >= MIN_SAMPLES and self.fit is not None else "collecting"

    def _record(self, t, metrics, fails):
        state = self.state()
        if metrics is None:
            self.timeline.append((t, 0, float("nan"), float("nan"), -1, state))
            line = f"t={t:7.1f}s seen {self.seen:5d} buf {self.buf_used.sum():3d}  no fit yet"
        else:
            self.timeline.append((t, metrics["cells"], metrics["fit"], metrics["inliers"],
                                  metrics["level"], state))
            caps = ",".join(metrics["missing"]) or "-"
            line = (f"t={t:7.1f}s seen {self.seen:5d} buf {self.buf_used.sum():3d} "
                    f"cover {metrics['cells']:3d}% caps missing {caps:<17} "
                    f"fit {metrics['fit'] * 100:5.2f}% inl {metrics['inliers'] * 100:5.1f}% "
                    f"{LEVEL_NAMES[metrics['level']]:<8} {state:<10} fails: {fails}")
        if self.progress == "all":
            self.out(line)

    def _event(self, t, text):
        self.events.append((t, text))
        if self.progress in ("all", "changes"):
            self.out(f"t={t:7.1f}s ** {text}")

    # ----- results
    def estimate(self):
        """Current estimate (the latest fit) as {o, S, R, rot}, or None."""
        if self.fit is None:
            return None
        model = self.fit.model
        return dict(o=model.o.copy(), S=model.w_norm(), R=model.radius(), rot=0.0)

    def accepted_estimate(self):
        if self.accepted is None:
            return None
        return dict(o=self.accepted.o.copy(), S=self.accepted.w_norm(), R=self.accepted.radius(), rot=0.0)

    def converged_at(self):
        return [t for t, text in self.events if "converged" in text]


# ---------------------------------------------------------------------------
# Baseline: port of ysoldak/magcal (BSD-3, Go).

class Magcal:
    """Offset o + full unsymmetric M, b = M (m − o), pattern search on
    Σ |‖b‖² − T²| over a 128-vector buffer binned by the raw vector's octant."""

    SIZE = 128
    TOLERANCE = 0.01
    STEP = 0.01
    MAX_PASSES = 500

    def __init__(self, target="50", progress="all", out=print):
        self.name = f"magcal(T={target})"
        self.target_mode = target
        self.T = 50.0 if target == "50" else None
        self.o = np.zeros(3)
        self.M = np.eye(3)
        self.buf = []            # [t, m, octant]
        self.progress, self.out = progress, out
        self.t0 = None
        self.seen = self.searches = self.evals = 0
        self.raw = []            # all samples so far, for the median target
        self.next_report = 10.0

    def feed(self, t_ms, m):
        if self.t0 is None:
            self.t0 = t_ms
        t = (t_ms - self.t0) / 1000.0
        m = np.asarray(m, dtype=float)
        self.seen += 1
        if self.target_mode != "50":
            self.raw.append(m)
        if self.T is not None:
            b = self.M @ (m - self.o)
            if abs(b @ b - self.T ** 2) <= self.TOLERANCE * self.T ** 2:
                self._report(t)
                return
        octant = int(m[0] >= 0) | int(m[1] >= 0) << 1 | int(m[2] >= 0) << 2
        if len(self.buf) < self.SIZE:
            self.buf.append([t, m, octant])
        else:
            counts = np.bincount([e[2] for e in self.buf], minlength=8)
            crowded = int(np.argmax(counts))
            members = [i for i, e in enumerate(self.buf) if e[2] == crowded]
            oldest = min(members, key=lambda i: self.buf[i][0])
            self.buf[oldest] = [t, m, octant]
        if len(self.buf) == self.SIZE:
            if self.target_mode != "50":
                # Rough centre from the 1st-99th percentile range: min/max would be
                # thrown by spikes, which magcal doesn't filter.
                raw = np.array(self.raw)
                centre = (np.percentile(raw, 1, axis=0) + np.percentile(raw, 99, axis=0)) / 2
                self.T = float(np.median(np.linalg.norm(raw - centre, axis=1)))
            self._search()
        self._report(t)

    def finish(self):
        pass

    def _error(self, X, o, M):
        self.evals += 1
        b = (X - o) @ M.T
        return float(np.abs((b * b).sum(axis=1) - self.T ** 2).sum())

    @staticmethod
    def _allowed(M):
        diag = np.diag(M)
        if np.any(diag < 0.5) or np.any(diag > 2.0):
            return False
        off = np.abs(M[~np.eye(3, dtype=bool)])
        return bool(np.all(off * 5 < diag.min()))

    def _search(self):
        self.searches += 1
        X = np.array([e[1] for e in self.buf])
        params = [("o", i) for i in range(3)] + [("M", (i, i)) for i in range(3)] + \
                 [("M", (i, j)) for i in range(3) for j in range(3) if i != j]
        best = self._error(X, self.o, self.M)
        for _ in range(self.MAX_PASSES):
            improved = False
            for kind, idx in params:
                step = self.STEP * self.T if kind == "o" else self.STEP
                for sign in (1, -1):
                    moved = False
                    for _ in range(10000):
                        o, M = self.o.copy(), self.M.copy()
                        if kind == "o":
                            o[idx] += sign * step
                        else:
                            M[idx] += sign * step
                            if not self._allowed(M):
                                break
                        e = self._error(X, o, M)
                        if e < best:
                            self.o, self.M, best = o, M, e
                            moved = improved = True
                        else:
                            break
                    if moved:
                        break
            if not improved:
                break

    def _report(self, t):
        if self.progress == "all" and t >= self.next_report:
            self.next_report += 10.0
            est = self.estimate()
            if est is not None:
                self.out(f"t={t:7.1f}s seen {self.seen:5d} buf {len(self.buf):3d} searches {self.searches:4d} "
                         f"o {fmt_vec(est['o'])} R {est['R']:.2f} rot {est['rot']:.1f}°")

    def estimate(self):
        if self.T is None:
            return None
        # Polar decomposition M = Q S: S is the soft-iron shape, Q a rotation of the axes.
        U, s, Vt = np.linalg.svd(self.M)
        Q = U @ Vt
        S = Vt.T @ np.diag(s) @ Vt
        scale = float(np.linalg.det(S)) ** (1.0 / 3.0)
        rot = math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(Q) - 1) / 2))))
        return dict(o=self.o.copy(), S=S / scale, R=self.T / scale, rot=rot)

    accepted_estimate = estimate

    def converged_at(self):
        return []


# ---------------------------------------------------------------------------
# Synthetic streams

FIELD_UT = 50.0
INCLINATION_DEG = 65.0
NOISE_UT = 0.4
LSB_UT = 0.3


def symmetric_det1(A):
    A = (np.asarray(A, dtype=float) + np.asarray(A, dtype=float).T) / 2
    return A / np.linalg.det(A) ** (1.0 / 3.0)


O1 = np.array([25.0, -40.0, 15.0])
W1 = symmetric_det1([[1.06, 0.05, -0.03], [0.05, 0.93, 0.04], [-0.03, 0.04, 1.01]])
O2 = O1 + np.array([20.0, -15.0, 10.0])
W2 = symmetric_det1([[1.02, -0.04, 0.02], [-0.04, 0.97, 0.05], [0.02, 0.05, 1.01]])
O_BIG = np.array([55.0, -45.0, 40.0])  # |o| ≈ 82 µT > R: raw vectors never change sign on X

SCENARIOS = {
    "clean": dict(duration=200, outliers=0.0, partial=False, change=None, o=O1,
                  expect="converge", tol=(1.0, 0.02, 0.01)),
    "outliers10": dict(duration=200, outliers=0.10, partial=False, change=None, o=O1,
                       expect="converge", tol=(1.0, 0.02, 0.01)),
    "outliers30": dict(duration=240, outliers=0.30, partial=False, change=None, o=O1,
                       expect="converge", tol=(1.5, 0.03, 0.015)),
    "partial": dict(duration=200, outliers=0.0, partial=True, change=None, o=O1,
                    expect="none", tol=None),
    "envchange": dict(duration=360, outliers=0.0, partial=False, change=170.0, o=O1,
                      expect="reconverge", tol=(1.0, 0.02, 0.01)),
    "bigoffset": dict(duration=200, outliers=0.0, partial=False, change=None, o=O_BIG,
                      expect="converge", tol=(1.0, 0.02, 0.01)),
}


def rodrigues(v):
    th = float(np.linalg.norm(v))
    if th < 1e-12:
        return np.eye(3)
    k = np.asarray(v) / th
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(th) * K + (1 - math.cos(th)) * K @ K


def truth_at(truth, t):
    current = truth[0]
    for phase in truth:
        if t >= phase[0]:
            current = phase
    return current


def synthesize(name, seed):
    """Stream at 10 Hz: 10 s still, 30 s of yaw with ±15° tilt (partial coverage),
    then random tumbling. Noise 0.4 µT, 0.3 µT LSB, 1 % single-sample spikes plus
    0.2 % two-sample spikes, and flat-topped disturbance bursts (a nearby magnet
    or current) covering the scenario's outlier fraction of the moving part."""
    spec = SCENARIOS[name]
    rng = np.random.default_rng(seed)
    dt = 0.1
    n = int(spec["duration"] / dt)
    t = np.arange(n) * dt
    inc = math.radians(INCLINATION_DEG)
    B = FIELD_UT * np.array([math.cos(inc), 0.0, -math.sin(inc)])
    truth = [(0.0, spec["o"], W1)]
    if spec["change"] is not None:
        truth.append((spec["change"], O2, W2))

    R0 = rodrigues(rng.normal(size=3))
    Rm = R0
    omega = np.zeros(3)
    clean = np.zeros((n, 3))
    for i, ti in enumerate(t):
        if ti < 10:
            Rm = R0
        elif spec["partial"] or ti < 40:
            tt = ti - 10
            Rm = rodrigues([0, 0, 1.0 * tt]) @ rodrigues([0.26 * math.sin(0.4 * tt), 0, 0]) @ R0
        else:
            omega = 0.95 * omega + rng.normal(0, 0.25, 3)
            Rm = Rm @ rodrigues(omega * dt)
        _, o, W = truth_at(truth, ti)
        clean[i] = np.linalg.solve(W, Rm.T @ B) + o

    disturb = np.zeros((n, 3))
    moving = int(10 / dt)
    target = spec["outliers"] * (n - moving)
    covered = 0
    while covered < target:
        length = int(rng.integers(10, 40))
        start = int(rng.integers(moving, n - length))
        envelope = np.ones(length)
        ramp = np.linspace(0, 1, 5)[1:-1]
        envelope[:3], envelope[-3:] = ramp, ramp[::-1]
        direction = rng.normal(size=3)
        direction /= np.linalg.norm(direction)
        disturb[start:start + length] += envelope[:, None] * rng.uniform(8, 40) * direction
        covered += length

    spikes = np.zeros((n, 3))
    for i in np.flatnonzero(rng.random(n) < 0.01):
        spikes[i, rng.integers(3)] += rng.choice([-1, 1]) * rng.uniform(50, 400)
    for i in np.flatnonzero(rng.random(n - 1) < 0.002):
        axis, size = rng.integers(3), rng.choice([-1, 1]) * rng.uniform(50, 400)
        spikes[i:i + 2, axis] += size

    m = clean + rng.normal(0, NOISE_UT, (n, 3)) + disturb + spikes
    m = np.round(m / LSB_UT) * LSB_UT
    return (t * 1000).astype(int), m, truth


def errors(est, truth_phase):
    _, o, W = truth_phase
    return dict(offset=float(np.linalg.norm(est["o"] - o)),
                shape=float(np.max(np.abs(est["S"] - W))),
                radius=abs(est["R"] / FIELD_UT - 1),
                rot=est["rot"])


def settled_time(marks, truth, tol):
    """Earliest checkpoint after which the estimate stays within tol until the
    end of its truth phase (the last phase for environment changes)."""
    last_phase_start = truth[-1][0]
    settled = None
    for t, est in marks:
        if t < last_phase_start:
            continue
        ok = est is not None and (lambda e: e["offset"] <= tol[0] and e["shape"] <= tol[1])(
            errors(est, truth_at(truth, t)))
        if ok and settled is None:
            settled = t
        elif not ok:
            settled = None
    return settled


# ---------------------------------------------------------------------------
# Running, reporting, plotting

def fmt_vec(v, digits=2):
    return "(" + ", ".join(f"{x:.{digits}f}" for x in v) + ")"


def run(estimator, t_ms, m, checkpoint_s=1.0):
    marks = []
    next_mark = 0.0
    for tm, x in zip(t_ms, m):
        estimator.feed(tm, x)
        ts = (tm - t_ms[0]) / 1000.0
        if ts >= next_mark:
            marks.append((ts, estimator.estimate()))
            next_mark += checkpoint_s
    estimator.finish()
    return marks


def report_ours(cal, out=print):
    out("")
    out(f"== ours: state {cal.state()}, {cal.seen} samples, {cal.spikes} dropped as spikes, "
        f"{cal.refused} refused by the inlier gate, {cal.restarts} restart(s)")
    for t, text in cal.events:
        out(f"   t={t:7.1f}s {text}")
    model = cal.accepted if cal.accepted is not None else (cal.fit.model if cal.fit else None)
    if model is None:
        out("   no fit")
        return
    label = "stored calibration" if cal.accepted is not None else "latest fit (not accepted)"
    out(f"   {label}: o = {fmt_vec(model.o)} µT, R = {model.radius():.2f} µT")
    for row in model.w_norm():
        out(f"      W | {fmt_vec(row, 4)}")
    if cal.last_crit:
        out("   criteria at the last fit:")
        for name, ok, value, limit in cal.last_crit:
            out(f"      {'ok  ' if ok else 'FAIL'} {name:<9} {value:<40} {limit}")
        out(f"      {'ok  ' if cal.last_metrics.get('stable') else 'FAIL'} {'stability':<9} "
            f"{STABLE_FITS} fits: |Δo| <= {STABLE_OFFSET_UT} µT, |ΔW| <= {STABLE_W}, |ΔR| <= {STABLE_R * 100} %")
    raw = np.array([x for _, x, dropped in cal.stream if not dropped])
    if len(raw):
        rawn = np.linalg.norm(raw, axis=1)
        caln = np.linalg.norm(model.calibrate(raw), axis=1)
        for name, v in (("raw |m|", rawn), ("calibrated |b|", caln)):
            dev = np.abs(v / np.median(v) - 1)
            out(f"   {name:<15} mean {v.mean():6.2f} µT, std {v.std() / v.mean() * 100:5.2f} %, "
                f"median |dev| {np.median(dev) * 100:5.2f} %, 95th pct {np.percentile(dev, 95) * 100:5.2f} %")


def report_magcal(mc, out=print):
    out("")
    out(f"== {mc.name}: {mc.seen} samples, buffer {len(mc.buf)}, {mc.searches} searches, "
        f"{mc.evals} error evaluations")
    est = mc.estimate()
    if est is None:
        out("   no estimate")
        return
    out(f"   o = {fmt_vec(est['o'])} µT, R = {est['R']:.2f} µT (T = {mc.T:.2f}), axes rotated {est['rot']:.2f}°")
    for row in mc.M:
        out(f"      M | {fmt_vec(row, 4)}")


def plot(cal, path, title):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig = plt.figure(figsize=(13, 11))
    fig.suptitle(title)
    stream = np.array([x for _, x, dropped in cal.stream if not dropped]).reshape(-1, 3)
    dropped = np.array([x for _, x, d in cal.stream if d]).reshape(-1, 3)
    buf = cal.buffer()
    model = cal.accepted if cal.accepted is not None else (cal.fit.model if cal.fit else None)
    inl = np.ones(len(buf), dtype=bool) if model is None else gate_inliers(
        model.residuals(buf), cal.noise() / model.radius())[0]
    pairs = ((0, 1, "X", "Y"), (0, 2, "X", "Z"), (1, 2, "Y", "Z"))
    for k, (a, b, na, nb) in enumerate(pairs):
        ax = fig.add_subplot(3, 3, k + 1)
        ax.scatter(stream[:, a], stream[:, b], s=1, c="0.8", label="stream")
        ax.scatter(buf[inl, a], buf[inl, b], s=4, c="tab:blue", label="buffer inliers")
        ax.scatter(buf[~inl, a], buf[~inl, b], s=12, c="tab:red", marker="x", label="buffer outliers")
        if len(dropped):
            near = np.all(np.abs(dropped - np.median(stream, axis=0)) < 300, axis=1)
            ax.scatter(dropped[near, a], dropped[near, b], s=10, c="tab:orange", marker="+", label="spikes")
        ax.set_xlabel(f"raw {na} µT"), ax.set_ylabel(f"raw {nb} µT"), ax.set_aspect("equal", "datalim")
        if k == 0:
            ax.legend(fontsize=7, loc="upper right")
    if model is not None:
        R = model.radius()
        cs = model.calibrate(stream)
        cb = model.calibrate(buf)
        ring = np.linspace(0, 2 * math.pi, 200)
        for k, (a, b, na, nb) in enumerate(pairs):
            ax = fig.add_subplot(3, 3, k + 4)
            ax.scatter(cs[:, a], cs[:, b], s=1, c="0.8")
            ax.scatter(cb[inl, a], cb[inl, b], s=4, c="tab:blue")
            ax.scatter(cb[~inl, a], cb[~inl, b], s=12, c="tab:red", marker="x")
            ax.plot(R * np.cos(ring), R * np.sin(ring), "k--", lw=0.8)
            ax.set_xlabel(f"calibrated {na} µT"), ax.set_ylabel(f"calibrated {nb} µT")
            ax.set_aspect("equal", "datalim")
    ax = fig.add_subplot(3, 1, 3)
    if cal.timeline:
        tl = np.array([(t, c, f, i) for t, c, f, i, _, _ in cal.timeline], dtype=float)
        ax.plot(tl[:, 0], tl[:, 1], label="coverage (cells, %)")
        ax.plot(tl[:, 0], tl[:, 3] * 100, label="inliers %")
        ax.axhline(COVER_CELLS, color="tab:blue", ls=":", lw=0.8)
        ax.set_ylim(0, 105)
        ax.set_xlabel("time s")
        ax2 = ax.twinx()
        ax2.plot(tl[:, 0], tl[:, 2] * 100, color="tab:red", label="fit error %")
        ax2.axhline(FIT_ERROR_MAX * 100, color="tab:red", ls=":", lw=0.8)
        ax2.set_yscale("log")
        ax2.set_ylabel("fit error %")
        for t, text in cal.events:
            ax.axvline(t, color="k" if "converged" in text else "tab:orange", lw=1)
        ax.legend(loc="upper left", fontsize=8)
        ax2.legend(loc="upper right", fontsize=8)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


def make_estimators(method, seed, progress):
    ests = []
    if method in ("ours", "both"):
        ests.append(Calibrator(seed=seed, progress=progress))
    if method in ("magcal", "both"):
        ests.append(Magcal(target="50", progress=progress))
        ests.append(Magcal(target="median", progress=progress))
    return ests


def run_synthetic(name, method, seed, progress, plot_path, save_csv):
    spec = SCENARIOS[name]
    t_ms, m, truth = synthesize(name, seed)
    extras = []
    if spec["partial"]:
        extras.append("partial coverage only")
    if spec["change"] is not None:
        extras.append(f"environment change at {spec['change']:.0f} s")
    extras.append(f"|o| = {np.linalg.norm(spec['o']):.0f} µT")
    print(f"\n######## scenario {name}: {len(m)} samples, {spec['duration']} s, "
          f"outliers {spec['outliers'] * 100:.0f} %, " + ", ".join(extras))
    if save_csv:
        write_csv(Path(save_csv), t_ms, m)  # the caller makes the name unique per scenario
    rows = []
    for est in make_estimators(method, seed, progress):
        marks = run(est, t_ms, m)
        if isinstance(est, Calibrator):
            report_ours(est)
            if plot_path:
                p = Path(plot_path)
                plot(est, p.with_name(f"{p.stem}_{name}{p.suffix}"), f"{name} (seed {seed})")
        else:
            report_magcal(est)
        final = est.accepted_estimate()
        e = errors(final, truth[-1]) if final is not None else None
        tol = spec["tol"] or (1.0, 0.02, 0.01)
        settled = settled_time(marks, truth, tol)
        conv = est.converged_at()
        after_change = [t for t in conv if spec["change"] is None or t >= spec["change"]]
        verdict = "-"
        if isinstance(est, Calibrator):
            verdict = check(spec, est, e)
        rows.append((est.name, conv, after_change, settled, e, verdict))
        if e is not None:
            print(f"   vs truth: offset error {e['offset']:.3f} µT, shape error {e['shape']:.4f}, "
                  f"radius error {e['radius'] * 100:.2f} %, axes rotated {e['rot']:.2f}°, "
                  f"settled (offset <= {tol[0]} µT, shape <= {tol[1]}) at "
                  f"{'never' if settled is None else f'{settled:.0f} s'}")
        print(f"   check: {verdict}")
    return rows


def check(spec, cal, e):
    exp = spec["expect"]
    if exp == "none":
        return "PASS (did not converge)" if cal.accepted is None else "FAIL (converged on partial coverage)"
    if cal.accepted is None:
        return "FAIL (never converged)"
    if exp == "reconverge":
        if cal.restarts < 1:
            return "FAIL (no restart after the change)"
        if not [t for t in cal.converged_at() if t >= spec["change"]]:
            return "FAIL (did not re-converge after the change)"
    tol = spec["tol"]
    bad = [f"{k} {e[k]:.4f} > {lim}" for k, lim in zip(("offset", "shape", "radius"), tol) if e[k] > lim]
    return "PASS" if not bad else "FAIL (" + ", ".join(bad) + ")"


def write_csv(path, t_ms, m):
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["seq", "t_ms", "x_ut", "y_ut", "z_ut"])
        for i, (t, x) in enumerate(zip(t_ms, m)):
            w.writerow([i + 1, int(t), f"{x[0]:.4f}", f"{x[1]:.4f}", f"{x[2]:.4f}"])


def read_csv(path):
    t, m = [], []
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        missing = {"t_ms", "x_ut", "y_ut", "z_ut"}.difference(reader.fieldnames or ())
        if missing:
            raise SystemExit(f"{path}: missing column(s) {', '.join(sorted(missing))}")
        for row in reader:
            t.append(int(row["t_ms"]))
            m.append((float(row["x_ut"]), float(row["y_ut"]), float(row["z_ut"])))
    return np.array(t), np.array(m)


def compare_on_stream(ests, m_stream):
    """Without a truth: how round each result makes the (spike-filtered) stream."""
    print("\n== comparison on the recording (spike-filtered samples):")
    for est in ests:
        e = est.accepted_estimate()
        if e is None:
            print(f"   {est.name:<18} no estimate")
            continue
        b = np.linalg.norm((m_stream - e["o"]) @ e["S"].T, axis=1)
        dev = np.abs(b / np.median(b) - 1)
        print(f"   {est.name:<18} o {fmt_vec(e['o'])} R {e['R']:6.2f}  |b| median dev {np.median(dev) * 100:5.2f} %, "
              f"RMS {np.sqrt(np.mean(dev ** 2)) * 100:5.2f} %, 95th pct {np.percentile(dev, 95) * 100:5.2f} %")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv", nargs="?", help="recording from tools/mag_record.py")
    ap.add_argument("--synthetic", metavar="NAME", help="|".join(SCENARIOS) + "|all")
    ap.add_argument("--method", choices=("ours", "magcal", "both"), default="ours")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--plot", metavar="OUT.png")
    ap.add_argument("--save-csv", metavar="OUT.csv", help="with --synthetic: also write the stream")
    ap.add_argument("--progress", choices=("all", "changes", "none"))
    args = ap.parse_args()

    if args.synthetic:
        names = list(SCENARIOS) if args.synthetic == "all" else [args.synthetic]
        if any(n not in SCENARIOS for n in names):
            ap.error(f"unknown scenario; choose from {', '.join(SCENARIOS)} or all")
        progress = args.progress or ("changes" if len(names) > 1 else "all")
        summary = {}
        for name in names:
            save_csv = args.save_csv
            if save_csv and len(names) > 1:  # several scenarios can't share one file
                p = Path(save_csv)
                save_csv = str(p.with_name(f"{p.stem}_{name}{p.suffix}"))
            summary[name] = run_synthetic(name, args.method, args.seed, progress, args.plot, save_csv)
        print("\n######## summary")
        print(f"{'scenario':<11} {'method':<18} {'converged at':<14} {'settled':<8} "
              f"{'offset µT':>9} {'shape':>7} {'R %':>6} {'rot °':>6}  check")
        failed = False
        for name, rows in summary.items():
            for est_name, conv, after, settled, e, verdict in rows:
                conv_s = ",".join(f"{t:.0f}" for t in conv[:3]) or "-"
                settled_s = "never" if settled is None else f"{settled:.0f} s"
                if e is None:
                    print(f"{name:<11} {est_name:<18} {conv_s:<14} {settled_s:<8} {'-':>9} {'-':>7} {'-':>6} {'-':>6}  {verdict}")
                else:
                    print(f"{name:<11} {est_name:<18} {conv_s:<14} {settled_s:<8} {e['offset']:9.3f} "
                          f"{e['shape']:7.4f} {e['radius'] * 100:6.2f} {e['rot']:6.2f}  {verdict}")
                failed |= verdict.startswith("FAIL")
        sys.exit(1 if failed else 0)

    if not args.csv:
        ap.error("give a recording CSV or --synthetic")
    t_ms, m = read_csv(args.csv)
    if len(m) == 0:
        ap.error(f"no samples in {args.csv}")
    print(f"{args.csv}: {len(m)} samples over {(t_ms[-1] - t_ms[0]) / 1000:.1f} s")
    progress = args.progress or "all"
    ests = make_estimators(args.method, args.seed, progress)
    for est in ests:
        print(f"\n######## {est.name}")
        run(est, t_ms, m)
        if isinstance(est, Calibrator):
            report_ours(est)
            if args.plot:
                plot(est, args.plot, Path(args.csv).name)
        else:
            report_magcal(est)
    ours = next((e for e in ests if isinstance(e, Calibrator)), None)
    if len(ests) > 1:
        stream = (np.array([x for _, x, d in ours.stream if not d]) if ours is not None else m)
        compare_on_stream(ests, stream)


if __name__ == "__main__":
    main()
