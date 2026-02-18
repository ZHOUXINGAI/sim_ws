#!/usr/bin/env bash
set -euo pipefail

SIM_WS=${HOME}/sim_ws
PX4_DIR=${SIM_WS}/px4/PX4-Autopilot
WORLD_NAME=${1:-baylands}

cd "${PX4_DIR}"

# Fixed spawn order and coordinates:
# instance 1 -> iris_aircarrier_1 at (0, 0)
# instance 2 -> plane_2         at (0, 20)
Tools/simulation/gazebo-classic/sitl_multiple_run.sh \
  -s iris_aircarrier:1:0:0,plane:1:0:20 \
  -w "${WORLD_NAME}"

