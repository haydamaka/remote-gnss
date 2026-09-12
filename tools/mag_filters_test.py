#!/usr/bin/env python3
"""Measures what each CALIBRATION.md section 3.1 intake filter does, one at a time
and together, on the recordings under tools/recordings and on the synthetic
scenarios. Every row is a full replay of the stream through mag_calibrate.py with
a different set of stages enabled.

Usage:
  mag_filters_test.py [--motion UT] [--quick]

Note: mag-session-good.csv and mag-session-mixed.csv are buffer snapshots, not
streams (t_ms rises by exactly 100 ms and seq by exactly 1 for every row, while
neighbouring readings are ~100 µT apart). The intake filters judge a reading
against its neighbours in time, so they mean nothing on those two; only the
fit-side stages do. The script says so rather than printing a number that looks
like a measurement.
"""

import argparse
import itertools
import sys
from pathlib import Path

import numpy as np

import mag_calibrate as M

HERE = Path(__file__).resolve().parent
RECORDINGS = ["mag-bench-1", "mag-session-good", "mag-session-mixed"]
STREAMED = {"mag-bench-1"}          # the rest are buffer snapshots
STAGES = ["glitch", "motion", "agree", "tidy", "evict"]


def is_stream(t_ms):
    """A genuine recording has jittery timestamps; a buffer dump has none."""
    dt = np.diff(t_ms)
    return not (len(dt) > 10 and np.all(dt == dt[0]))


def replay(t_ms, m, filters, seed=1):
    old = dict(M.FILTERS)
    M.FILTERS.update(filters)
    try:
        cal = M.Calibrator(seed=seed, progress="none", out=lambda _: None)
        for ti, mi in zip(t_ms, m):
            cal.feed(ti, mi)
        cal.finish()
    finally:
        M.FILTERS.clear()
        M.FILTERS.update(old)
    return cal


def summary(cal, ref):
    """ref is every reading in the file: the spread of |b| over it is the one number
    the filters cannot game, since it is the same set of readings for every row."""
    model = cal.accepted if cal.accepted is not None else (cal.fit.model if cal.fit else None)
    met = cal.last_metrics or {}
    conv = cal.converged_at()
    spread = float("nan")
    if model is not None:
        b = np.linalg.norm(model.calibrate(ref), axis=1)
        spread = float(np.median(np.abs(b / np.median(b) - 1)))
    return dict(
        state=cal.state(),
        kept=cal.seen - cal.spikes,
        glitch=cal.dropped["glitch"], motion=cal.dropped["motion"],
        disagreed=cal.disagreed, evicted=cal.evict_refused, refused=cal.refused, tidied=cal.tidied,
        sigma=cal.noise(), have_sigma=cal.noise_ut is not None,
        cells=met.get("cells", 0), fit=met.get("fit", float("nan")),
        limit=met.get("limit", float("nan")), inliers=met.get("inliers", float("nan")),
        R=met.get("radius", float("nan")),
        o=model.o.copy() if model is not None else None,
        converged=conv[0] if conv else None,
        spread=spread,
    )


def row(label, s):
    conv = f"{s['converged']:6.1f}s" if s["converged"] is not None else "   -   "
    return (f"  {label:<26} kept {s['kept']:5d}  cells {s['cells']:3d}  "
            f"fit {s['fit'] * 100:5.2f}% (limit {s['limit'] * 100:5.2f}%)  "
            f"inl {s['inliers'] * 100:5.1f}%  R {s['R']:5.2f}  "
            f"sigma {s['sigma']:5.3f}{'' if s['have_sigma'] else '*'}  "
            f"conv {conv}  spread {s['spread'] * 100:5.2f}%")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--motion", type=float, default=None, help="override MOTION_MAX_UT")
    ap.add_argument("--quick", action="store_true", help="skip the synthetic scenarios")
    args = ap.parse_args()
    if args.motion is not None:
        M.MOTION_MAX_UT = args.motion

    off = dict({k: False for k in STAGES}, medians=False)

    for name in RECORDINGS:
        t_ms, m = M.read_csv(HERE / "recordings" / f"{name}.csv")
        stream = is_stream(t_ms)
        step = np.median(np.linalg.norm(np.diff(m, axis=0), axis=1))
        print(f"\n==== {name}: {len(m)} readings, "
              f"{'stream' if stream else 'BUFFER SNAPSHOT (not a time series)'}, "
              f"median step between rows {step:.2f} µT")
        if not stream:
            print("     t_ms rises by a constant, so neighbouring rows are buffer slots, not")
            print("     moments: (b) glitch and (c) motion mean nothing here and are left off.")
        usable = STAGES if stream else ["agree", "tidy", "evict"]
        allon = dict({k: (k in usable) for k in STAGES}, medians=False)

        def show(label, filters, note=""):
            s = summary(replay(t_ms, m, filters), m)
            print(row(label, s) + note)
            return s

        show("none (baseline)", off)
        for st in usable:
            s = summary(replay(t_ms, m, dict(off, **{st: True})), m)
            print(row(f"only {st}", s) + "  " + tally(st, s))
        s = show("all of the above", allon)
        for st in usable:                      # what each one is still contributing
            show(f"all but {st}", dict(allon, **{st: False}), "  " + tally(st, s))
        show("all + per-cell medians", dict(allon, medians=True))

        if stream:
            keep = M.MOTION_MAX_UT
            for gate in (15.0, 25.0, 40.0, 1e9):
                M.MOTION_MAX_UT = gate
                tag = "off" if gate > 1e8 else f"{gate:.0f} µT"
                s = summary(replay(t_ms, m, allon), m)
                print(row(f"all, motion gate {tag}", s) + f"  [{s['motion']} dropped]")
            M.MOTION_MAX_UT = keep

    if args.quick:
        return
    print("\n==== synthetic scenarios: before (no stage) vs after (all stages)")
    on = dict({k: True for k in STAGES}, medians=False)
    for name in M.SCENARIOS:
        for label, f in (("before", off), ("after", on)):
            t_ms, m, truth = M.synthesize(name, 1)
            cal = replay(t_ms, m, f)
            est = cal.accepted_estimate() or cal.estimate()
            verdict = M.check(M.SCENARIOS[name], cal, M.errors(est, truth[-1])) if est else "FAIL (no fit)"
            conv = cal.converged_at()
            at = f"{conv[0]:6.1f}s" if conv else "   -   "
            print(f"  {name:<12} {label:<7} conv {at}  {verdict}")


def tally(st, s):
    return {"glitch": f"[{s['glitch']} dropped]", "motion": f"[{s['motion']} dropped]",
            "agree": f"[{s['disagreed']} replaced a cell member]",
            "tidy": f"[{s['tidied']} fits tidied]",
            "evict": f"[{s['evicted']} newcomers refused]"}[st]


if __name__ == "__main__":
    main()
