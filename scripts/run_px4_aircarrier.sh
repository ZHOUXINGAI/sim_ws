#!/usr/bin/env bash
set -e

# ===== 工作区 =====
SIM_WS=$HOME/sim_ws
PX4_DIR=$SIM_WS/px4/PX4-Autopilot
WORLD_NAME=baylands.world
WORLD_DIR=$PX4_DIR/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds

# ===== Gazebo 环境 =====
export PX4_SIM_HOSTNAME=127.0.0.1
export GAZEBO_MODEL_PATH=$PX4_DIR/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models

# ===== 切换 world（gazebo-classic 必须这样）=====
echo "[INFO] Using world: $WORLD_NAME"
cp "$WORLD_DIR/$WORLD_NAME" "$WORLD_DIR/empty.world"

# ===== 启动 =====
cd "$PX4_DIR"
make px4_sitl gazebo-classic_iris_aircarrier
