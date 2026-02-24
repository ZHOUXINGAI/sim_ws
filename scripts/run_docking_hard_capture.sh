#!/usr/bin/env bash
set -euo pipefail

cd /home/hw/sim_ws
pkill -f docking_manager.py || true

python3 scripts/docking_manager.py \
  --plane-url udp:127.0.0.1:14542 \
  --carrier-url udp:127.0.0.1:14541 \
  --plane-sysid 3 \
  --carrier-sysid 2 \
  --world-name default \
  --use-gz-pose \
  --gz-pose-rate 10 \
  --gz-max-model-speed 30 \
  --gz-pose-alpha 0.4 \
  --plane-model-name plane_2 \
  --carrier-model-name iris_aircarrier_1 \
  --plane-model plane_2 \
  --carrier-model iris_aircarrier_1 \
  --rate 20 \
  --stream-rate 30 \
  --offboard-refresh-sec 0.2 \
  --yaw-mode auto \
  --yaw-auto-switch-dist 3.0 \
  --lead-time 0 \
  --intercept-time 0 \
  --track-forward 0 --track-right 0 --track-down 0.6 \
  --track-xy-thr 0.8 --track-z-thr 0.3 \
  --track-d-min -40 --track-d-max -1.5 \
  --land-forward 0 --land-right 0 --land-down 0.6 \
  --land-xy-thr 0.2 --land-z-thr 0.2 \
  --land-hold-sec 0.3 \
  --land-d-min -40 --land-d-max -1.5 \
  --hard-capture \
  --hard-capture-xy 0.2 \
  --hard-capture-down-min 0.3 \
  --hard-capture-down-max 1.2 \
  --hard-capture-hold-sec 0.5 \
  --kill-plane-px4-on-capture \
  --slave-offset-forward 0 \
  --slave-offset-right 0 \
  --slave-offset-up 0 \
  --home-from-start \
  --home-thr 0.8 \
  --arm-carrier --wait-arm 15 \
  --relax-arm-checks \
  --follow-yaw \
  --ff-plane-vel 0.0 \
  --ff-kp-pos 1.1 \
  --ff-max-xy 18 \
  --ff-max-z 3 \
  --track-vel-mode \
  --track-vel-switch-xy 6.0 \
  --no-dock-vel-match \
  --pure-pursuit \
  --pure-kp-xy 1.6 \
  --pure-kp-z 1.0 \
  --debug
