# OV9281 global-shutter camera driver for Orange Pi Zero 3W (Allwinner A733)

Goal: run two OV9281 mono global-shutter cameras on the Zero 3W's two MIPI CSI
ports as a stereo pair for onboard VIO/SLAM. The vendor BSP ships no OV9281
driver, so this folder ports one into Allwinner's VIN camera framework.

## Status
- [x] Vendor camera stack confirmed working (VIN loads, both CSI ports probe)
- [x] On-board driver build environment (headers + vendor source + Kbuild)
- [x] Rebuilt the vendor IMX219 driver out-of-tree and loaded it successfully
- [ ] `ov9281_mipi.c`: read chip ID 0x9281 over I2C
- [ ] First frame (Y10, 1280x800)
- [ ] Device-tree overlay for sensor0 / sensor1
- [ ] Two cameras streaming, hardware-synchronised (FSIN)

## Platform
| Item | Value |
|---|---|
| Board | Orange Pi Zero 3W, 6 GB |
| SoC | Allwinner A733 (2x Cortex-A76 + 6x Cortex-A55, 3 TOPS NPU, E902 RISC-V) |
| Kernel | `6.6.98-sun60iw2` (vendor BSP) |
| Camera framework | Allwinner VIN (`vin_io.ko`, `vin_v4l2.ko`) |
| Vendor source | [`orangepi-xunlong/linux-orangepi`](https://github.com/orangepi-xunlong/linux-orangepi), branch `orange-pi-6.6-sun60iw2` |

## Findings
- The board's DTB (`sun60i-a733-orangepi-zero3w.dtb`) already contains the full
  camera graph: `vind`, 3x `csi`, 3x `mipi`, `isp`, `tdm`, `sensor`, `vinc`.
- `sensor0` is preset to `imx219` (CCI bus 0x0b, 8-bit addr 0x20, MCLK 1,
  pwdn PE6, status okay). `sensor1` is preset to `ov13850_mipi` (disabled).
- Loading VIN probes IMX219 on both CSI ports. With nothing connected:
  ```
  sunxi:vin:[INFO]: [VIN_DEV_I2C]imx219 read retry 3, read addr is 0x0
  sunxi:vin:[ERR]: [imx219]chip found is not an target chip.
  sunxi:vin:[ERR]: registering imx219_2, No such device!
  ```
  i.e. the pipeline is alive and waiting for a sensor.
- Shipped sensor modules: only `imx219.ko` and `ov13850_mipi.ko`.
- Building out-of-tree needs Allwinner's own headers from `bsp/include`
  (e.g. `sunxi-log.h`), which the headers package does not include. The
  `Kbuild` here adds that include path and builds only the drivers we need.

## Build (on the board)
```bash
./setup.sh                 # headers, sparse vendor source, build
sudo rmmod vin_v4l2        # VIN only probes sensors when it loads
sudo insmod build/<driver>.ko
sudo modprobe vin_v4l2
sudo dmesg | grep -iE 'imx219|ov9281'
```
`loading out-of-tree module taints kernel` is expected.

## Porting plan (IMX219 -> OV9281)
| Field | IMX219 | OV9281 |
|---|---|---|
| `SENSOR_NAME` | `imx219` | `ov9281_mipi` |
| `V4L2_IDENT_SENSOR` | `0x0219` | `0x9281` |
| `I2C_ADDR` (8-bit) | `0x20` | `0xc0` (7-bit 0x60) |
| Chip-ID register | `0x0000` | `0x300A` / `0x300B` |
| Media-bus format | `SRGGB10_1X10` | `Y10_1X10` (mono) |
| Register tables | IMX219 | from mainline `drivers/media/i2c/ov9282.c` |
| DT `sensor0_isp_used` | 1 | 0 (mono RAW, bypass Bayer ISP) |

## License
Everything in this folder is GPL-2.0, because the driver is derived from the
Allwinner vendor driver (`imx219.c`, GPL) and mainline Linux `ov9282.c`
(GPL-2.0-only). Original copyright notices are kept in the source files.
