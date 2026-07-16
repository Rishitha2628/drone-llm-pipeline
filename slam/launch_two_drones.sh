#!/usr/bin/env bash
# Launch PX4 instance 1 (the second drone) attached to the ALREADY-RUNNING
# Gazebo world started by instance 0.
#
# Usage:
#   Terminal A (drone 0 + Gazebo):
#       cd ~/PX4-Autopilot
#       __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia \
#         QT_QPA_PLATFORM=xcb PX4_GZ_WORLD=lawn make px4_sitl gz_x500
#       (wait for "Ready for takeoff!")
#
#   Terminal B (drone 1) — THIS SCRIPT:
#       bash slam/launch_two_drones.sh
#
# Result:
#   drone 0 = model x500_0 at (0,0),  MAVLink on udp port 14540
#   drone 1 = model x500_1 at (0,5),  MAVLink on udp port 14541
#
# PX4's rc scripts detect the running Gazebo ("gazebo already running") and
# spawn a second model into it. The -i 1 instance ID shifts every port by +1
# automatically, so no manual port config is needed.

set -e

PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
WORLD="${PX4_GZ_WORLD:-lawn}"
POSE="${PX4_GZ_MODEL_POSE:-0,-5}"    # Gazebo ENU x,y -> "0,-5" = 5 m SOUTH in NED.
                                     # MUST match HOME_OFFSETS_NED in pipeline/squad_executor.py

cd "$PX4_DIR"

if ! pgrep -f "gz sim" > /dev/null; then
    echo "ERROR: no running Gazebo found. Start instance 0 first (see header)."
    exit 1
fi

echo "[launch_two_drones] attaching instance 1 to world '$WORLD' at pose ($POSE)"

PX4_GZ_WORLD="$WORLD" \
PX4_SYS_AUTOSTART=4001 \
PX4_SIM_MODEL=gz_x500 \
PX4_GZ_MODEL_POSE="$POSE" \
  ./build/px4_sitl_default/bin/px4 -i 1
