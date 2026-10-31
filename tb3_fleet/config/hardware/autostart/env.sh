#!/usr/bin/env bash
# EDIT the values below to match your robot, then re-run install.sh.
# ─────────────────────────────────────────────────────────────────────────────

# Must match the ROS_DOMAIN_ID in the robot's ~/.bashrc.
export ROS_DOMAIN_ID=5

# TurtleBot3 and OpenCR.
export TURTLEBOT3_MODEL=burger
export OPENCR_MODEL=burger
export OPENCR_PORT=/dev/ttyACM0

# LDS-01 or LDS-02. These robots run ld08_driver, which is LDS-02.
export LDS_MODEL=LDS-02

# Map run-nav2.sh passes to Nav2.
export MAP=$HOME/turtlebot3_ws/maps/map.yaml

source /opt/ros/humble/setup.bash
source $HOME/turtlebot3_ws/install/setup.bash

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
