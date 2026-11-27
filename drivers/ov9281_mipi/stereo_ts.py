#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0
"""Compare per-frame timestamps of the two cameras (logs from stereo.sh).

For every left frame, find the nearest right frame and report the offset.
Two free-running sensors each have their own oscillator, so the offset
usually drifts; hardware triggering (J3) is what makes it ~0 and constant.
"""
import re
import sys
import numpy as np

TS = re.compile(r"seq:\s*(\d+).*?ts:\s*([0-9.]+)")


def load(path):
    ts = [float(m.group(2)) for m in map(TS.search, open(path, errors="ignore")) if m]
    if not ts:
        raise SystemExit(f"{path}: no 'ts:' lines (v4l2-ctl --verbose output?)")
    return np.array(ts)


L, R = load(sys.argv[1]), load(sys.argv[2])
for name, t in (("left", L), ("right", R)):
    d = np.diff(t) * 1e3
    print(f"{name:5s}: {t.size} frames, period {np.median(d):.3f} ms "
          f"({1e3 / np.median(d):.2f} fps), jitter {d.std():.3f} ms, "
          f"gaps>1.5T {int((d > 1.5 * np.median(d)).sum())}")

idx = np.clip(np.searchsorted(R, L), 1, R.size - 1)
near = np.where(np.abs(R[idx] - L) < np.abs(R[idx - 1] - L), R[idx], R[idx - 1])
off = (L - near) * 1e3                       # ms, left minus right
k = max(1, off.size // 10)
print(f"offset L-R: median {np.median(off):+.3f} ms, min {off.min():+.3f}, "
      f"max {off.max():+.3f}")
print(f"drift: first 10% {np.median(off[:k]):+.3f} ms -> last 10% "
      f"{np.median(off[-k:]):+.3f} ms over {L[-1] - L[0]:.1f} s")
