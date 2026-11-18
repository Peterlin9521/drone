#!/bin/bash
case "${1:-rect}" in rect) F=~/sim/follow_world_rect.sdf;; line) F=~/sim/follow_world.sdf;; *) F=$1;; esac
[ -f "$F" ] || { echo "找不到世界檔 $F"; exit 1; }
~/sim/stop.sh > /dev/null 2>&1; sleep 1
S=sim; CAM=/world/iris_runway/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera
tmux new-session -d -s $S -n gz
tmux send-keys -t $S:gz "gz sim -s -r $F" Enter
tmux new-window -t $S -n sitl
tmux send-keys -t $S:sitl "source ~/venv-ardupilot/bin/activate && cd ~/ardupilot/ArduCopter && ../Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console" Enter
tmux new-window -t $S -n bridge
tmux send-keys -t $S:bridge "sleep 10; ros2 run ros_gz_bridge parameter_bridge '$CAM/image@sensor_msgs/msg/Image[gz.msgs.Image' '$CAM/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo' '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock' --ros-args -r '$CAM/image:=/camera/image' -r '$CAM/camera_info:=/camera/camera_info'" Enter
tmux new-window -t $S -n truth
tmux send-keys -t $S:truth "sleep 12; /usr/bin/python3 ~/sim/truth.py $F" Enter
tmux new-window -t $S -n mavros
tmux send-keys -t $S:mavros "sleep 30; ros2 launch mavros apm.launch fcu_url:=tcp://127.0.0.1:5762" Enter
tmux new-window -t $S -n view
tmux send-keys -t $S:view "sleep 15; ros2 run rqt_image_view rqt_image_view /follow/image" Enter
tmux new-window -t $S -n follow
tmux send-keys -t $S:follow "source ~/venv-ros/bin/activate && clear" Enter
tmux attach -t $S
