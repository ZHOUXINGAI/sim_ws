#!/usr/bin/env bash
set -euo pipefail

SIM_WS=${HOME}/sim_ws
PX4_DIR=${SIM_WS}/px4/PX4-Autopilot
WORLD_NAME=${1:-baylands_no_trees}

cd "${PX4_DIR}"

# PX4's gazebo-classic runner auto-enables ROS Gazebo plugins when ROS_VERSION=2.
# On this machine those libs are missing, which causes startup errors.
if [[ "${ROS_VERSION-}" == "2" ]]; then
  if ! ldconfig -p 2>/dev/null | grep -Eq "libgazebo_ros_init\\.so|libgazebo_ros_factory\\.so"; then
    echo "[WARN] ROS Gazebo plugins not found; launching gazebo-classic without ros gazebo plugins."
    unset ROS_VERSION
  fi
fi

# Fixed spawn order and coordinates:
# instance 1 -> iris_aircarrier_1 at (0, 0)
# instance 2 -> plane_2         at (0, 20)
Tools/simulation/gazebo-classic/sitl_multiple_run.sh \
  -s iris_aircarrier:1:0:0,plane:1:0:20 \
  -w "${WORLD_NAME}"
