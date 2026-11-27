#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0
"""Live focus meter: read RAW10 frames from stdin, print a sharpness score.

Turn the lens until the score peaks. Score = variance of the Laplacian over
the centre of the frame (higher = sharper). Use with focus.sh.
"""
import sys
import numpy as np

W, H = 1280, 800
FRAME = W * H * 2
CROP = 200                       # half-size of the centre window (px)
best = 0.0
n = 0
buf = sys.stdin.buffer
try:
    while True:
        raw = buf.read(FRAME)
        if len(raw) < FRAME:
            break
        n += 1
        if n % 6:                    # ~10 updates/s at 60 fps
            continue
        f = np.frombuffer(raw, dtype="<u2").reshape(H, W).astype(np.float32)
        if f.max() > 1023:
            f /= 64.0
        c = f[H // 2 - CROP:H // 2 + CROP, W // 2 - CROP:W // 2 + CROP]
        lap = (c[1:-1, :-2] + c[1:-1, 2:] + c[:-2, 1:-1] + c[2:, 1:-1]
               - 4 * c[1:-1, 1:-1])
        score = float(lap.var())
        best = max(best, score)
        bar = "#" * int(40 * score / best) if best else ""
        print(f"\rsharp {score:8.1f}  best {best:8.1f}  mean {c.mean():6.1f}  {bar:<40}",
              end="", flush=True)
except KeyboardInterrupt:
    pass
print()
