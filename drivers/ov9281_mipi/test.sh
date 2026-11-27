#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-2.0
# Load the OV9281 driver into VIN and report what happened (both CSI ports).
set -uo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
KO="$HERE/build/ov9281_mipi.ko"
[ -f "$KO" ] || { echo "build first: ./setup.sh"; exit 1; }

echo "== live DT sensors (enabled only) =="
for N in $(find /proc/device-tree/ -type d -name 'sensor@*' | sort); do
  [ "$(tr -d '\0' < "$N/status" 2>/dev/null)" = okay ] || continue
  i=$(tr -d '\0' < "$N/device_type"); i=${i#sensor}
  printf 'sensor%s  mname=%s' "$i" "$(tr -d '\0' < "$N/sensor${i}_mname")"
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

# Let udev finish probing the new nodes, then attach each sensor once
# (S_INPUT) so a later VIDIOC_S_CTRL cannot hit VIN's NULL-sensor Oops.
sudo udevadm settle
for d in $(v4l2-ctl --list-devices 2>/dev/null |
           awk '/sunxi-vin/{f=1;next} /^[^ \t]/{f=0} f&&/\/dev\/video/{print $1}'); do
  v4l2-ctl -d "$d" -i 0 >/dev/null && echo "attached sensor on $d"
done

echo "== video devices =="
v4l2-ctl --list-devices 2>/dev/null || ls /dev/video*
