#!/bin/bash
ros2 service call /mavros/set_stream_rate mavros_msgs/srv/StreamRate "{stream_id: 0, message_rate: 30, on_off: true}" > /dev/null
for c in "param set BATT_CAPACITY 30000" "mode guided" "arm throttle" "takeoff 2"; do tmux send-keys -t sim:sitl "$c" Enter; sleep 2; done
echo "已送出起飛指令(若沒起飛,等 10 秒再執行一次)"
