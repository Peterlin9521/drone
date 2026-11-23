#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-2.0
# Build Allwinner VIN camera sensor drivers on an Orange Pi Zero 3W
# (Allwinner A733, vendor kernel 6.6.98-sun60iw2), directly on the board.
set -euo pipefail

KVER=$(uname -r)
SRC=${SRC:-$HOME/linux-orangepi}
BRANCH=orange-pi-6.6-sun60iw2
HERE=$(cd "$(dirname "$0")" && pwd)
SENSOR="$SRC/bsp/drivers/vin/modules/sensor"

# 1. Kernel headers (Orange Pi ships the matching .deb in /opt)
if [ ! -d "/lib/modules/$KVER/build" ]; then
  echo "[1/4] Installing kernel headers"
  sudo dpkg -i /opt/linux-headers-current-sun60iw2_*_arm64.deb
fi

# 2. Vendor source: only the VIN camera stack and Allwinner headers
if [ ! -d "$SRC/.git" ]; then
  echo "[2/4] Fetching $BRANCH (sparse)"
  git clone --depth 1 --filter=blob:none --sparse -b "$BRANCH" \
    https://github.com/orangepi-xunlong/linux-orangepi "$SRC"
fi
git -C "$SRC" sparse-checkout set bsp/drivers/vin bsp/include

# 3. Drop our Kbuild and driver sources into the vendor sensor folder
echo "[3/4] Copying Kbuild and sources"
cp "$HERE/Kbuild" "$SENSOR/Kbuild"
shopt -s nullglob
for f in "$HERE"/*.c; do cp "$f" "$SENSOR/"; done

# 4. Build against the running kernel
echo "[4/4] Building"
make -C "/lib/modules/$KVER/build" M="$SENSOR" modules
mkdir -p "$HERE/build"
cp "$SENSOR"/*.ko "$HERE/build/"
ls -l "$HERE/build"
