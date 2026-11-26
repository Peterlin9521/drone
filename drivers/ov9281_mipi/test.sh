#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-2.0
# Load the OV9281 driver into VIN and report what happened.
set -uo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
KO="$HERE/build/ov9281_mipi.ko"
[ -f "$KO" ] || { echo "build first: ./setup.sh"; exit 1; }

echo "== live DT sensor0 =="
N=$(find /proc/device-tree/ -type d -name 'sensor@5812000' | head -1)
for p in mname twi_addr isp_used; do
  printf '%-10s ' "$p"
  if [ "$p" = mname ]; then tr -d '\0' < "$N/sensor0_$p"; echo
  else od -An -tx1 "$N/sensor0_$p" | tr -d ' \n'; echo; fi
done

echo "== reload VIN with ov9281_mipi =="
sudo rmmod vin_v4l2 2>/dev/null
sudo rmmod ov9281_mipi 2>/dev/null
sudo modprobe vin_io
sudo insmod "$KO"
sudo dmesg -C
sudo modprobe vin_v4l2
sleep 1
sudo dmesg | grep -iE 'ov9281|sensor|vinc0|register|cci' | head -40

echo "== video devices =="
v4l2-ctl --list-devices 2>/dev/null || ls /dev/video*
