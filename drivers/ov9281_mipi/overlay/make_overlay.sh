#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-2.0
# Build one device-tree user overlay that switches both CSI sensors from
# IMX219 to OV9281 and install it for the Orange Pi boot script.
#
# Only enabled (status = okay) sensor nodes count. On the Zero 3W these are
#   CAM1 = sensor@5812000, device_type "sensor0"  -> "ov9281_mipi"
#   CAM2 = sensor@5812020, device_type "sensor2"  -> "ov9281_mipi_2"
# (sensor@5812010 / "sensor1" exists but is disabled.)
# For each: sensorN_mname, sensorN_twi_addr = 0xc0 (8-bit form of 0x60),
#           sensorN_isp_used = 0 (mono RAW: bypass the Bayer ISP)
#
# Usage: ./make_overlay.sh        both cameras
#        ./make_overlay.sh 1      CAM1 only
set -euo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
NAME=ov9281-sensors
OLD=ov9281-sensor0              # earlier single-camera overlay
NCAM=${1:-2}

printf '/dts-v1/;\n/plugin/;\n/ {\n' > "$HERE/$NAME.dts"

# Enabled sensor nodes, ordered by their sensorN number
NODES=$(for n in $(find /proc/device-tree/ -type d -name 'sensor@*'); do
          [ "$(tr -d '\0' < "$n/status" 2>/dev/null)" = okay ] || continue
          t=$(tr -d '\0' < "$n/device_type")
          echo "${t#sensor} $n"
        done | sort -n | head -"$NCAM")
[ -n "$NODES" ] || { echo "no enabled sensor nodes in /proc/device-tree"; exit 1; }

k=0
while read -r N NODE; do
  [ "$k" = 0 ] && MNAME=ov9281_mipi || MNAME=ov9281_mipi_2
  DTPATH=${NODE#/proc/device-tree}
  echo "CAM$((k+1)): $DTPATH  sensor$N  $(tr -d '\0' < "$NODE/sensor${N}_mname") -> $MNAME"
  cat >> "$HERE/$NAME.dts" <<DTS
	fragment@$k {
		target-path = "$DTPATH";
		__overlay__ {
			sensor${N}_mname = "$MNAME";
			sensor${N}_twi_addr = <0xc0>;
			sensor${N}_isp_used = <0x00>;
		};
	};
DTS
  k=$((k+1))
done <<< "$NODES"
echo "};" >> "$HERE/$NAME.dts"

dtc -@ -I dts -O dtb -o "$HERE/$NAME.dtbo" "$HERE/$NAME.dts"
sudo mkdir -p /boot/overlay-user
sudo cp "$HERE/$NAME.dtbo" /boot/overlay-user/

ENV=/boot/orangepiEnv.txt
# Drop the old single-camera overlay so two overlays don't fight over sensor0
sudo sed -i -E "/^user_overlays=/ s/ ?\b$OLD\b//; s/^user_overlays= +/user_overlays=/; /^user_overlays=$/d" "$ENV"
if grep -q '^user_overlays=' "$ENV"; then
  grep -q "\b$NAME\b" "$ENV" || sudo sed -i "s/^user_overlays=.*/& $NAME/" "$ENV"
else
  echo "user_overlays=$NAME" | sudo tee -a "$ENV" >/dev/null
fi
echo "--- $ENV ---"; grep -E '^(overlays|user_overlays)=' "$ENV"
echo "Installed. Reboot, then run ./test.sh"
