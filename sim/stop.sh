#!/bin/bash
tmux kill-session -t sim 2>/dev/null
for p in "gz sim" arducopter mavproxy sim_vehicle mavros_node parameter_bridge rqt_image_view; do pkill -9 -f "$p" 2>/dev/null; done
echo "已全部關閉"
