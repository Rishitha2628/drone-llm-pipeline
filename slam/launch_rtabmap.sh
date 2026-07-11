#!/usr/bin/env bash
# Point-cloud SLAM launcher. Uses direct node composition to avoid
# rtabmap.launch.py argument-passing quirks.
set -e
source /opt/ros/humble/setup.bash
ros2 launch $(dirname "$(readlink -f "$0")")/launch_rtabmap.py
