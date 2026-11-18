# Flight controller

- Board: Matek F405-WMO, flashed with the **MatekF405-TE** target (same family)
- Firmware: ArduCopter 4.7.2, custom build from custom.ardupilot.org with:
  `OPTICALFLOW`, `MSP_OPTICALFLOW`, `EKF3_OPTFLOW`, `MODE_FLOWHOLD`, `MSP_RANGEFINDER`, `MSP_SENSORS`
  (the stock F405 build omits optical flow and MSP sensor drivers)
- `drone.param`: full parameter dump from Mission Planner

## Optical flow + rangefinder (MicoAir MTF-01)
- MTF-01 speaks MSP (`MSP2_SENSOR_OPTIC_FLOW`, `MSP2_SENSOR_RANGEFINDER`), not MAVLink
- `FLOW_TYPE` = MSP and the rangefinder set to MSP; **reboot the FC** after setting `FLOW_TYPE`
- All 4 broken-out UARTs are used, so the companion computer (Orange Pi) talks to the FC
  on its UART2 via mavlink-router (see `onboard/config/pi_config.txt`)
