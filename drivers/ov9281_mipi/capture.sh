#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-2.0
# Grab RAW10 frames from the OV9281 through Allwinner VIN and convert to PNG.
#   ./capture.sh [count] [name]          e.g. ./capture.sh 5 frames
#   CAM=1 ./capture.sh 1 right           second sunxi-vin node (CAM2)
# Env: DEV=/dev/videoN overrides CAM (default CAM=0: first sunxi-vin node)
# Run inside drone-env (raw2png.py needs numpy + pillow).
set -euo pipefail
COUNT=${1:-1}
NAME=${2:-shot}
CAM=${CAM:-0}
HERE=$(cd "$(dirname "$0")" && pwd)

if [ -z "${DEV:-}" ]; then
  DEV=$(v4l2-ctl --list-devices 2>/dev/null |
        awk -v n="$CAM" '/sunxi-vin/{f=1;next} /^[^ \t]/{f=0} f&&/\/dev\/video/{if(c++==n){print $1; exit}}')
fi
[ -n "$DEV" ] || { echo "no sunxi-vin video node #$CAM; run ./test.sh first"; exit 1; }
echo "device: $DEV"

# VIN only attaches the sensor to the pipeline after VIDIOC_S_INPUT, and only
# for the open file handle, so input, format and streaming must be one call.
# With the ISP bypassed only RAW formats work (BG10 = our mono RAW10).
v4l2-ctl -d "$DEV" -i 0 \
  --set-fmt-video=width=1280,height=800,pixelformat=BG10 \
  --stream-mmap --stream-skip=10 --stream-count="$COUNT" \
  --stream-to="$NAME.raw"
# "VIDIOC_CREATE_BUFS returned -1" is harmless: v4l2-ctl falls back to REQBUFS.

python3 "$HERE/raw2png.py" "$NAME.raw"
