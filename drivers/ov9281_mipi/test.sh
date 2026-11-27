#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-2.0
# Load the OV9281 driver into VIN and report what happened (both CSI ports).
set -uo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
KO="$HERE/build/ov9281_mipi.ko"
[ -f "$KO" ] || { echo "build first: ./setup.sh"; exit 1; }

echo "== live DT sensors =="
for i in 0 1; do
  P=$(find /proc/device-tree/ -name "sensor${i}_mname" | head -1)
  [ -n "$P" ] || { echo "sensor$i: not in DT"; continue; }
  N=$(dirname "$P")
  printf 'sensor%s  mname=%s' "$i" "$(tr -d '\0' < "$P")"
  for p in twi_addr isp_used; do
    printf '  %s=%s' "$p" "$(od -An -tx1 "$N/sensor${i}_$p" 2>/dev/null | tr -d ' \n')"
  done
  echo
done

echo "== reload VIN with ov9281_mipi =="
sudo rmmod vin_v4l2 2>/dev/null
sudo rmmod ov9281_mipi 2>/dev/null
sudo modprobe vin_io
sudo insmod "$KO"
sudo dmesg -C
sudo modprobe vin_v4l2
sleep 1
sudo dmesg | grep -iE 'ov9281|sensor|vinc|register|cci' | head -40

echo "== video devices =="
v4l2-ctl --list-devices 2>/dev/null || ls /dev/video*
