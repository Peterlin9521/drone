# Vision-Based Drone Follow-Me (ROS 2 + ArduPilot SITL + Gazebo)

Simulation stack for developing a person-following controller before deploying it on a
small quadcopter (Matek F405 / ArduCopter 4.7.2, Orange Pi Zero 3W with NPU).

## Stack
- Ubuntu 24.04 (WSL2), ROS 2 Jazzy, Gazebo Harmonic, ArduPilot SITL, MAVROS
- YOLOv8n person detection (GPU in sim; Allwinner A733 NPU on the real drone)

## Components
| File | Purpose |
|---|---|
| `start.sh` / `stop.sh` / `takeoff.sh` | Launch / stop the full sim in tmux, take off |
| `follow_world*.sdf` | Worlds with a scripted walking person (back-and-forth / rectangle) |
| `truth.py` | Publishes the person's ground-truth pose (`/walker/pose`) from the script + sim clock |
| `follow_node.py` | Baseline: bearing Kalman filter + feed-forward yaw, box-height range control |
| `mpc_core.py` | Translational MPC (OSQP): standoff distance, soft min-distance constraint |
| `mpc_follow.py` | ROS 2 MPC node: configurable follow angle (side / behind), yaw feed-forward |
| `cam_geom.py`, `target_cam.py` | Camera-based target localisation: foot pixel -> ground-plane ray -> KF |
| `calib.py` | Calibrates camera tilt / gimbal model against ground truth |

## Results so far (sim)
- Found and fixed: self-detections of the drone's own frame, tracker ID switches, low camera rate
- Baseline controller: drifts onto the person's walking line; box-height ranging got as close as 0.7 m
- MPC side-follow: mean 3.05 m (target 3 m), min 2.84 m, never on the walking line, ~2.8 ms per solve

## Notes
The camera model in `ardupilot_gazebo/models/gimbal_small_3d/model.sdf` was modified
(update rate 30 Hz, horizontal FOV 80 deg, ~12 deg downward tilt).
