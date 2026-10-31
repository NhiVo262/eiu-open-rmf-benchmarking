#!/usr/bin/env bash
set -e
source "$(dirname "$0")/env.sh"
exec ros2 launch turtlebot3_navigation2 navigation2.launch.py \
    use_sim_time:=False \
    map:="$MAP"
