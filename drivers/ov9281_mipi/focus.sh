#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-2.0
# Live focus meter. Point the camera at a detailed target 1-2 m away, turn
# the lens slowly and stop where "sharp" peaks. Ctrl-C to quit.
#   ./focus.sh                CAM1      CAM=1 ./focus.sh    CAM2
#   EXP=28000 GAIN=32 ./focus.sh        brighter (dim rooms)
# Run inside drone-env (needs numpy).
set -uo pipefail
CAM=${CAM:-0}
HERE=$(cd "$(dirname "$0")" && pwd)
if [ -z "${DEV:-}" ]; then
  DEV=$(v4l2-ctl --list-devices 2>/dev/null |
        awk -v n="$CAM" '/sunxi-vin/{f=1;next} /^[^ \t]/{f=0} f&&/\/dev\/video/{if(c++==n){print $1; exit}}')
fi
[ -n "$DEV" ] || { echo "no sunxi-vin video node #$CAM; run ./test.sh first"; exit 1; }
C="exposure=${EXP:-28000},gain=${GAIN:-32}"
echo "device: $DEV  controls: $C   (Ctrl-C to stop)"
v4l2-ctl -d "$DEV" -i 0 -c "$C" \
  --set-fmt-video=width=1280,height=800,pixelformat=BG10 \
  --stream-mmap --stream-to=- 2>/dev/null | python3 "$HERE/focus.py"
