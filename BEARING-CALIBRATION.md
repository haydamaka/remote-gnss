# Learning the bearing offset from GNSS — proposal

Status: **plan only**, nothing implemented. The manual path exists today: point
the module along a known bearing, type the bearing in, press **Set bearing**,
and the firmware stores `heading_offset` in NVS (SPEC.md, "Compass bearing").

This document is about doing the same thing automatically, from the GNSS
course over ground, while the device is moving.

## 1. The idea, and the one assumption it rests on

When the device moves, GNSS knows which way it is *travelling*. The
magnetometer knows which way the module is *pointing*. The difference between
them is the mounting offset — the thing **Set bearing** asks the user for.

That only holds while **travel direction equals pointing direction**. It is
true for a module screwed into a car, a bike or a rover going forwards. It is
false when reversing (180° out), when a boat or an aircraft crabs into a
current or a crosswind (a steady few degrees out), and when the device is
carried in a hand or a pocket (out by anything at all). Everything in §4 exists
to keep those cases out of the estimate.

So: this is a refinement of a manual calibration, not a replacement for one. It
should be possible to turn it off, and it should never be able to make a
compass that was right become wrong.

## 2. What the difference actually contains

Per qualifying sample, with GNSS course `C` and measured magnetic heading `H`:

```
δ = wrap(C − D − H)
```

where `D` is the magnetic declination at the current position, from `main/wmm.c`.

`D` matters. GNSS course is referenced to **true** north; the magnetometer
measures **magnetic** north. At Kyiv they differ by 8.6° E. Leave `D` out and
the learned offset silently swallows the declination — which happens to make
the compass read true north, and looks right, until the device is used a few
hundred kilometres away and is wrong by the change in declination with no
indication why.

Recommendation: **subtract `D` explicitly.** `heading_offset` then stays what it
claims to be — a constant of how the module is screwed down — and declination
stays a separate, visible, position-dependent correction. When there is no WMM
answer (no fix, or past the model's life) the sample is not used.

## 3. The estimator

### 3.1 Not a running average of angles

Averaging 359° and 1° arithmetically gives 180°. Accumulate unit vectors
instead, weighted:

```
Sx += w·cos δ      Sy += w·sin δ      W += w
offset = atan2(Sy, Sx)
```

This answers "should we use an accumulated average" — yes, but as a vector sum,
which costs three floats and gives the confidence for free:

```
R = hypot(Sx, Sy) / W            concentration, 0..1
σ_offset ≈ sqrt(−2·ln R) / sqrt(N_eff)      radians
```

`R` near 1 means the samples agree; `R` near 0 means they are spread round the
circle and the offset is meaningless. That single number gates whether the
estimate is applied at all, and it is what the page should show.

### 3.2 The weight, and how much speed is really worth

Weight each sample by the inverse of its expected variance:

```
w = 1 / (σ_C² + σ_H² + σ_turn²)
```

- **σ_C, the course error.** GNSS course comes from the Doppler velocity, so its
  angular error is roughly `σ_v / v` with `σ_v ≈ 0.05 m/s` for a u-blox M10:

  | speed | 1 m/s | 3 m/s | 5 m/s | 10 m/s | 20 m/s |
  |---|---|---|---|---|---|
  | σ_C | 2.9° | 0.95° | 0.57° | 0.29° | 0.14° |

- **σ_H, the compass error.** A fit error of `e` (the `fit_rms` already reported)
  costs roughly `e` radians of heading. The current *rough* calibration at
  3.7 % is about **2.1°**; a *good* one at 2.5 % is about 1.4°.

- **σ_turn, the timing error.** The magnetometer sample and the GNSS fix are not
  simultaneous. With Δt up to ~150 ms, a 30°/s turn puts them 4.5° apart.
  `σ_turn = turn_rate · Δt`.

The useful consequence: **σ_H dominates above about 3 m/s.** Going from 5 m/s
to 20 m/s improves a sample's variance by under 5 %, while going from 3 m/s
down to 1 m/s makes it four times worse. So yes, weight by speed — but through
this formula, where the weight saturates on its own. Do not weight by speed
directly; that would let one motorway run outvote everything else for no real
reason.

It also says where the speed gate belongs: **3 m/s (11 km/h)** is where GNSS
course becomes better than the compass it is correcting. Below it there is
nothing to gain, and at walking pace the assumption of §1 is not true anyway.

### 3.3 One sample per straight run, not per fix

Do not feed individual fixes into the estimator. Take a **window** of `T`
seconds, test that the vehicle went straight through all of it, and emit **one**
sample: the circular mean of `δ` over the window.

This does two jobs at once. GNSS course errors are correlated over several
seconds — the same multipath, the same receiver filtering — so ten fixes a
second on a straight road are nowhere near ten independent measurements, and
feeding them in one by one inflates the confidence to nonsense. And measured
straightness is the only direct evidence that the module was pointing where the
vehicle was going.

**How straight is straight.** Testing `max − min` of the course does not work:
at 3 m/s the course noise alone is about 1°, so 80 fixes scatter over ±3° with
the vehicle on rails. Fit a line through the unwrapped course against time
instead, and test the two parts separately:

| test | bound | what it rejects |
|---|---|---|
| slope of the course | ≤ 1°/s | turning |
| total change across the window | ≤ 5° | a slow constant-radius curve |
| scatter about that line (circular SD) | ≤ 2.5° | a lane change, a skid, a roundabout, bad reception |
| the same three on the *measured heading* | same bounds | the module moving independently of the vehicle |
| the two slopes agree | within 0.5°/s | a module loose in a cup holder: it yaws when the vehicle does not |

The last row is the cheap test that catches the case that is otherwise hard to
see. A device sliding about produces headings that drift while the course does
not.

There is a pleasant accident here worth relying on: at a **constant** yaw rate,
the mean course over the window equals the course at the window's midpoint, and
the same holds for the heading — so a gentle steady curve cancels out of `δ`
entirely. What does not cancel is a *timing* difference between the two averages,
and a *changing* rate. That is why the scatter bound is the tight one and the
slope bound can be moderate.

**Weight from what was measured, not from a formula.** §3.2's variance estimate
is a prior; the window has the real thing. Use the residual scatter of `δ` about
its own mean, divided by the number of effective samples in the window, and take
`w = 1/σ²` from that. A window through a hedge-lined lane weighs itself down
without anyone having to model hedges.

### 3.3.1 How long should the window be?

**8 seconds**, with non-overlapping windows.

The instinct is that longer is better — more averaging, stronger evidence of
straightness. It is wrong here, for a reason that has nothing to do with
statistics:

> **Long windows are only accepted on long straight roads, and long straight
> roads all point the same way.**

Motorways and trunk roads run in a handful of directions. If the estimator only
ever accepts 30-second windows, every sample it gets comes from two or three
compass sectors, §3.4 never fills, and what does get through is the heading
error of those particular directions applied to all of them. A window length
that accepts a turn onto a side street, a run down it, and the run after the
next junction is worth far more than one that accepts only the bypass.

The statistical argument agrees, weakly. With a correlation time of ~5 s, a
window holds about `T/5` independent course samples, so the accuracy of one
window improves only as `sqrt(T)` — 16 s is 1.4 times better than 8 s — while
the *number* of windows a trip yields falls much faster than that, because a
run has to be entirely straight to count at all.

| T | independent samples | accuracy of one window | where it gets accepted |
|---|---|---|---|
| 4 s | ~0.8 | poor; barely evidence of anything | almost anywhere, including between two turns |
| **8 s** | ~1.6 | ~1.5-2° at 3 m/s, better when faster | most urban straight runs between junctions |
| 16 s | ~3 | ~1-1.4° | main roads |
| 30 s | ~6 | ~0.8-1° | motorways, and little else |

Since the whole trip is accumulated anyway (§3.1) the per-window accuracy hardly
matters: 40 windows at 2° beat 8 windows at 1°, and they arrive from more
directions. What matters is that every accepted window is **unbiased**, and bias
comes from turning, not from noise — which is what the table above is for.

Keep it a `menuconfig` setting, since the right answer depends on the vehicle: a
boat or a plane would want 30 s and would get it.

### 3.4 Diversity: the estimate is not allowed to come from one direction

If `δ` were really constant, one direction of travel would be enough. It is not
constant when the magnetometer calibration is imperfect: a residual soft-iron
error makes the heading error depend on heading. Learning the offset from an
hour of driving north teaches the compass north's error and applies it
everywhere.

Keep `Sx, Sy, W` **per 45° sector of measured heading** (8 sectors), and:

- require **at least 3 occupied sectors**, each above a minimum weight, before
  applying anything;
- take the offset as the mean of the occupied sectors' means, so a sector that
  was driven for an hour does not outvote one driven for a minute;
- report the **spread between sectors** — it is a direct, free measurement of
  how good the magnetometer calibration is, and worth showing next to the
  calibration quality on the page.

This is the part that answers "what if we need to update it continuously": it is
not the *time* structure that needs care, it is the *heading* structure.

### 3.5 Drift: converge, then fade slowly

A mounting offset is a constant until someone unscrews the module. Two things
can be wanted from it — settle down and stop moving, or follow a remount — and
a plain accumulating average does neither well: it converges and then becomes
too heavy to ever move again.

Recommendation: **cap the accumulated weight.** Once `W` reaches the equivalent
of ~200 effective samples, scale `Sx, Sy, W` down before adding each new one, so
the estimator becomes a fading average with a fixed memory. It converges in a
few minutes of driving, holds steady, and still follows a genuine remount within
a few more minutes. No half-life against wall-clock time — memory should be
measured in qualifying motion, not in how long the device sat on a shelf.

### 3.6 The first estimate must survive reversing

A weighted mean has no defence against a reversing car: those samples are a
clean 180° out and there may be a lot of them in a car park. Once there *is* an
estimate, §4's gate rejects them. Getting the first one needs something robust:
take the **circular median** of the first 20 qualifying samples, then switch to
the weighted mean from §3.1 seeded with it.

## 4. When a window counts

Every one of these must hold for the **whole** window, or it is dropped —
nothing is averaged across a gate change:

| Gate | Value | Why |
|---|---|---|
| Not spoofed | `spoof ≤ NONE` and `!implausible` | a faked course teaches a faked offset |
| Fix quality | `fix`, `satellites ≥ 5`, `hdop < 2` | course from a poor fix is noise |
| Speed | `> 3 m/s` throughout | §3.2 |
| Speed steady | within ±20 % across the window | braking and accelerating pitch the vehicle, and the module with it |
| Straightness | the five tests of §3.3 | the whole point |
| Field magnitude | within 20 % of the WMM horizontal field, every sample | a magnet, a motor or a steel bridge went past |
| No gap | no missing fixes or magnetometer samples | a window with a hole in it is two short windows |
| Agrees with the current estimate | `\|δ − offset\| < 3σ + 15°`, once converged | reversing, crabbing, a transient |

A window that fails is discarded whole and the next one starts after it. No
sliding: overlapping windows would count the same correlated seconds twice, and
the confidence in §3.1 would be a fiction.

## 5. What the user sees and controls

In the bearing row under the compass:

- a checkbox, **Learn from GNSS while moving** — off by default, kept in NVS;
- the state, in one line: *Learning — 43 samples, 4 of 8 directions, ±1.9°*, or
  *Learned −11.8° ±0.7°*, or *Waiting for movement above 11 km/h*;
- **Set bearing** stays, and a manual entry always wins: it replaces the
  estimate and reseeds the accumulator with the manual value at a modest weight,
  so learning refines it instead of fighting it.

The disagreement between sectors (§3.4) belongs next to the calibration quality,
not here.

### Phase it in

1. **Shadow mode first.** Collect and display, apply nothing. The page shows
   what it *would* set the offset to, next to the current one. This is the only
   honest way to find out whether the assumption of §1 holds on the vehicle
   this ends up on, and it costs one line of UI.
2. Apply it once shadow mode agrees with a manual offset on a real trip.

## 6. Storage and API

NVS namespace `mag`, alongside `hdg_off`:

| key | what |
|---|---|
| `hdg_auto` | the checkbox |
| `hdg_acc` | the 8 sectors' `Sx, Sy, W` — 96 bytes |

Write on change only, at most once every 5 minutes, and only when the offset
moved by more than 0.5°: this runs for hours and flash wears out.

`POST /api/heading` gains `auto=0|1`. `/api/status` gains, under `mag`:
`heading_auto`, `heading_learned`, `heading_conf` (degrees), `heading_samples`,
`heading_sectors`, `heading_spread`.

## 7. What this cannot do

- **No accelerometer.** The heading is `atan2(Y, X)` and is only right with the
  module flat. On a hill, or on a motorbike leaning into a bend, the tilt error
  is large — at Kyiv's 67.7° inclination, a 10° pitch is worth several degrees
  of heading. The estimate will absorb whatever tilt is typical of the route.
  Tilt compensation needs an accelerometer the WS-M181 does not have.
- **Crabbing is indistinguishable from a mounting offset** by any means
  available here. On water or in the air this method learns the average crab
  angle as if it were the mounting.
- **It cannot fix a bad magnetometer calibration**, only measure how bad it is
  (§3.4). A compass whose error depends on heading needs recalibrating, not
  offsetting.

## 8. Open questions

- The 5 s correlation time behind §3.3.1 is a guess, and the window length rests
  on it. It is measurable from one recorded trip: the autocorrelation of the
  course residual against lag, on a road known to be straight. Worth doing
  before settling on 8 s.
- Whether a failed window should be retried from its midpoint rather than its
  end. A vehicle that turns 2 s into a window wastes the 6 s of straight running
  that follows.
- Whether to let the estimate apply at all while the magnetometer calibration is
  *poor* (§2.1 of CALIBRATION.md), or only at *rough* and better.
- Whether the sector means should be weighted by sector coverage (a sector
  driven once is noisy) or trimmed (drop the worst sector of 8).
