#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-2.0
# Build one device-tree user overlay that switches both CSI sensors from
# IMX219 to OV9281 and install it for the Orange Pi boot script.
#   sensorN_mname    = "ov9281_mipi" (CAM1) / "ov9281_mipi_2" (CAM2)
#   sensorN_twi_addr = 0xc0           (8-bit form of 7-bit 0x60)
#   sensorN_isp_used = 0              (mono RAW: bypass the Bayer ISP)
# Usage: ./make_overlay.sh        both cameras
#        ./make_overlay.sh 0      CAM1 only
set -euo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
NAME=ov9281-sensors
OLD=ov9281-sensor0              # earlier single-camera overlay
IDS=${1:-"0 1"}

{
  echo "/dts-v1/;"
  echo "/plugin/;"
  echo "/ {"
} > "$HERE/$NAME.dts"

for i in $IDS; do
  [ "$i" = 0 ] && MNAME=ov9281_mipi || MNAME=ov9281_mipi_2
  # The sensor node is the one that carries sensorN_mname
  PROP=$(find /proc/device-tree/ -name "sensor${i}_mname" | head -1)
  [ -n "$PROP" ] || { echo "sensor${i}_mname not found in /proc/device-tree"; exit 1; }
  NODE=$(dirname "$PROP")
  DTPATH=${NODE#/proc/device-tree}
  echo "sensor$i node: $DTPATH  (current mname: $(tr -d '\0' < "$PROP") -> $MNAME)"
  cat >> "$HERE/$NAME.dts" <<DTS
	fragment@$i {
		target-path = "$DTPATH";
		__overlay__ {
			sensor${i}_mname = "$MNAME";
			sensor${i}_twi_addr = <0xc0>;
			sensor${i}_isp_used = <0x00>;
		};
	};
DTS
done
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
