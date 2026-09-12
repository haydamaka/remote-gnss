#!/usr/bin/env python3
"""Runs the reference fit (tools/mag_calibrate.py) on one recording, on exactly
the readings tools/fit_test.c uses, so the C port can be compared against it.

    fit_ref.py RECORDING.csv
"""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import mag_calibrate as ref  # noqa: E402


def load(path, limit=ref.BUFFER_SIZE):
    _, m = ref.read_csv(path)
    stride = max(1, -(-len(m) // limit))
    return m[::stride][:limit]


def main():
    if len(sys.argv) != 2:
        sys.exit("usage: fit_ref.py RECORDING.csv")
    m = load(sys.argv[1])
    cal = ref.Calibrator(seed=1, progress="none", out=lambda *a, **k: None)
    fit = cal._fit(m)
    if fit is None:
        print("no fit")
        sys.exit(1)
    model = fit.model
    w = model.w_norm()
    eig = np.linalg.eigvalsh(w)
    occupied, caps, scatter = ref.coverage(model, m[fit.mask])
    print(f"points {len(m)} inliers {int(fit.mask.sum())} level {model.level}")
    print("offset %.4f %.4f %.4f" % tuple(model.o))
    print("radius %.5f" % model.radius())
    print("rms %.6f" % fit.rms(m))
    print("eig %.5f %.5f" % (eig[0], eig[-1]))
    print("cells %d caps_min %d scatter %.4f wobble %.5f"
          % (occupied, min(caps), scatter, ref.wobble(model, m[fit.mask])))
    print("matrix %.5f %.5f %.5f %.5f %.5f %.5f"
          % (w[0, 0], w[1, 1], w[2, 2], w[0, 1], w[0, 2], w[1, 2]))


if __name__ == "__main__":
    main()
