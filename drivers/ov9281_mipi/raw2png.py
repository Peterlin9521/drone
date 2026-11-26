#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0
"""Convert OV9281 RAW10 frames (16-bit little-endian containers) to PNG.
usage: raw2png.py frames.raw [width height]"""
import sys
import numpy as np
from PIL import Image

path = sys.argv[1]
w = int(sys.argv[2]) if len(sys.argv) > 2 else 1280
h = int(sys.argv[3]) if len(sys.argv) > 3 else 800
data = np.fromfile(path, dtype='<u2')
n = data.size // (w * h)
if n == 0:
    sys.exit(f"file too small for {w}x{h}: {data.size} pixels")
frames = data[: n * w * h].reshape(n, h, w)
for i, f in enumerate(frames):
    # 10-bit data may be MSB- or LSB-aligned in the 16-bit word
    img = (f >> 8) if f.max() > 1023 else (f >> 2)
    out = f"{path.rsplit('.', 1)[0]}_{i}.png"
    Image.fromarray(img.astype(np.uint8)).save(out)
    print(out, "min", f.min(), "max", f.max(), "mean", round(float(f.mean()), 1))
