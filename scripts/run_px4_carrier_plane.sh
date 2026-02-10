#!/usr/bin/env bash
set -e

SIM_WS=$HOME/sim_ws
PX4_DIR=$SIM_WS/px4/PX4-Autopilot
WORLD_NAME=baylands

cd "$PX4_DIR"

# Spawn one carrier + one plane in the same Gazebo
Tools/simulation/gazebo-classic/sitl_multiple_run.sh -s iris_aircarrier:1,plane:1 -w "$WORLD_NAME"
