#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-2.0
# Stream both cameras at once and measure how far apart their frames are.
#   ./stereo.sh [frames] [name]           default 300 frames (~5 s), "stereo"
#   EXP=3500 GAIN=64 ./stereo.sh          same controls for both cameras
# Writes NAME_left.raw / NAME_right.raw, the first pair as PNG, and the
# per-frame driver timestamps in NAME_left.log / NAME_right.log.
# Run inside drone-env.
set -uo pipefail
COUNT=${1:-300}
NAME=${2:-stereo}
HERE=$(cd "$(dirname "$0")" && pwd)
mapfile -t DEVS < <(v4l2-ctl --list-devices 2>/dev/null |
  awk '/sunxi-vin/{f=1;next} /^[^ \t]/{f=0} f&&/\/dev\/video/{print $1}')
[ "${#DEVS[@]}" -ge 2 ] || { echo "need two sunxi-vin video nodes; run ./test.sh"; exit 1; }
C="exposure=${EXP:-10272},gain=${GAIN:-16}"
echo "left ${DEVS[0]}  right ${DEVS[1]}  controls: $C  frames: $COUNT"

grab() {   # dev side
  v4l2-ctl -d "$1" -i 0 -c "$C" \
    --set-fmt-video=width=1280,height=800,pixelformat=BG10 \
    --stream-mmap --stream-skip=10 --stream-count="$COUNT" --verbose \
    --stream-to="${NAME}_$2.raw" > "${NAME}_$2.log" 2>&1
}
grab "${DEVS[0]}" left &
grab "${DEVS[1]}" right &
wait

for s in left right; do
  python3 "$HERE/raw2png.py" "${NAME}_$s.raw" --first 1
done
python3 "$HERE/stereo_ts.py" "${NAME}_left.log" "${NAME}_right.log"
