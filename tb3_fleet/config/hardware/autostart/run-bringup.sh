#!/usr/bin/env bash
set -e
source "$(dirname "$0")/env.sh"
exec ros2 launch turtlebot3_bringup robot.launch.py
