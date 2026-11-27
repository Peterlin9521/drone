#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-2.0
# Live view in the PC's browser: http://<Pi IP>:8080   (Ctrl-C to stop)
#   ./live.sh                  CAM1        CAM=1 ./live.sh     CAM2
#   EXP=28000 GAIN=32 ./live.sh            PORT=8081 ./live.sh
#   FPS=10 ./live.sh           browser frames/s (default 5; higher = more CPU)
#   Power the board from a proper 5 V / 3 A supply, not a PC USB port.
# Run inside drone-env (needs numpy + pillow).
set -uo pipefail
CAM=${CAM:-0}
PORT=${PORT:-8080}
FPS=${FPS:-5}
HERE=$(cd "$(dirname "$0")" && pwd)
if [ -z "${DEV:-}" ]; then
  DEV=$(v4l2-ctl --list-devices 2>/dev/null |
        awk -v n="$CAM" '/sunxi-vin/{f=1;next} /^[^ \t]/{f=0} f&&/\/dev\/video/{if(c++==n){print $1; exit}}')
fi
[ -n "$DEV" ] || { echo "no sunxi-vin video node #$CAM; run ./test.sh first"; exit 1; }
C="exposure=${EXP:-28000},gain=${GAIN:-32}"
echo "device: $DEV  controls: $C"
echo "Pi IP: $(hostname -I | awk '{print $1}')  -> open http://$(hostname -I | awk '{print $1}'):$PORT"
v4l2-ctl -d "$DEV" -i 0 -c "$C" \
  --set-fmt-video=width=1280,height=800,pixelformat=BG10 \
  --stream-mmap --stream-to=- 2>/dev/null | python3 "$HERE/live.py" "$PORT" "$FPS"
