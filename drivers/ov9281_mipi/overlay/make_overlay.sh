#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-2.0
# Build a device-tree user overlay that switches CSI sensor0 from IMX219 to
# OV9281 and install it for the Orange Pi boot script.
#   sensor0_mname    = "ov9281_mipi"  (must match the driver name)
#   sensor0_twi_addr = 0xc0           (8-bit form of 7-bit 0x60)
#   sensor0_isp_used = 0              (mono RAW: bypass the Bayer ISP)
set -euo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
NAME=ov9281-sensor0

# Find the live path of sensor0 (sensor@5812000) in the running device tree
NODE=$(find /proc/device-tree/ -type d -name 'sensor@5812000' | head -1)
[ -n "$NODE" ] || { echo "sensor@5812000 not found in /proc/device-tree"; exit 1; }
DTPATH=${NODE#/proc/device-tree}
echo "sensor0 node: $DTPATH"
echo "current mname: $(tr -d '\0' < "$NODE/sensor0_mname")"

cat > "$HERE/$NAME.dts" <<DTS
/dts-v1/;
/plugin/;

/ {
	fragment@0 {
		target-path = "$DTPATH";
		__overlay__ {
			sensor0_mname = "ov9281_mipi";
			sensor0_twi_addr = <0xc0>;
			sensor0_isp_used = <0x00>;
		};
	};
};
DTS

dtc -@ -I dts -O dtb -o "$HERE/$NAME.dtbo" "$HERE/$NAME.dts"
sudo mkdir -p /boot/overlay-user
sudo cp "$HERE/$NAME.dtbo" /boot/overlay-user/

ENV=/boot/orangepiEnv.txt
if grep -q '^user_overlays=' "$ENV"; then
  grep -q "$NAME" "$ENV" || sudo sed -i "s/^user_overlays=.*/& $NAME/" "$ENV"
else
  echo "user_overlays=$NAME" | sudo tee -a "$ENV" >/dev/null
fi
echo "--- $ENV ---"; grep -E '^(overlays|user_overlays)=' "$ENV"
echo "Installed. Reboot, then run ../test.sh"
