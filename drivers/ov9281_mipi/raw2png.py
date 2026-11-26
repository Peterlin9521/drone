#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0
"""Convert OV9281 RAW10 frames (16-bit little-endian words) to PNG.

usage: raw2png.py frames.raw [--width 1280] [--height 800]
                             [--rotate 0|90|180|270] [--linear]

By default each frame is contrast-stretched (1st..99.5th percentile) so
dark indoor shots are visible; --linear keeps the plain 10->8 bit scale.
"""
import argparse
import numpy as np
from PIL import Image

ap = argparse.ArgumentParser()
ap.add_argument("raw")
ap.add_argument("--width", type=int, default=1280)
ap.add_argument("--height", type=int, default=800)
ap.add_argument("--rotate", type=int, default=0, choices=[0, 90, 180, 270])
ap.add_argument("--linear", action="store_true")
a = ap.parse_args()

w, h = a.width, a.height
data = np.fromfile(a.raw, dtype="<u2")
n = data.size // (w * h)
if n == 0:
    raise SystemExit(f"file too small for {w}x{h}: {data.size} pixels")

for i, f in enumerate(data[: n * w * h].reshape(n, h, w)):
    f = f.astype(np.float32)
    if f.max() > 1023:              # MSB-aligned 10-bit data
        f /= 64.0
    if a.linear:
        img = f / 4.0
    else:
        lo, hi = np.percentile(f, 1), np.percentile(f, 99.5)
        img = (f - lo) / max(hi - lo, 1.0) * 255.0
    img = np.clip(img, 0, 255).astype(np.uint8)
    if a.rotate:
        img = np.rot90(img, k=a.rotate // 90)
    out = f"{a.raw.rsplit('.', 1)[0]}_{i}.png"
    Image.fromarray(img).save(out)
    print(f"{out}  min {f.min():.0f} max {f.max():.0f} mean {f.mean():.1f}")
