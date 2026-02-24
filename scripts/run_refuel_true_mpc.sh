#!/usr/bin/env bash
set -euo pipefail

cd /home/hw/sim_ws
pkill -f carrier_refuel_true_mpc.py || true
pkill -f carrier_refuel_global_track.py || true

python3 scripts/carrier_refuel_global_track.py \
  --plane-url udp:127.0.0.1:14542 \
  --carrier-url udp:127.0.0.1:14541 \
  --plane-sysid 3 \
  --carrier-sysid 2 \
  --target-rel-n 0 \
  --target-rel-e 0 \
  --target-rel-d -1 \
  --rate 20 \
  --stream-rate 20 \
  --lead-sec 0.45 \
  --lead-relv-gain 0.35 \
  --lag-adapt-enable \
  --lag-alpha 0.18 \
  --lag-max 2.2 \
  --lag-gain 1.0 \
  --kp-xy 0.55 \
  --kd-xy 0.70 \
  --ki-xy 0.12 \
  --kp-z 0.90 \
  --kd-z 0.55 \
  --ki-z 0.08 \
  --plane-acc-alpha 0.35 \
  --plane-acc-max 8.0 \
  --ff-acc-xy 0.85 \
  --ff-acc-z 0.85 \
  --i-zone-xy 50 \
  --i-zone-z 10 \
  --i-limit-xy 80 \
  --i-limit-z 20 \
  --close-boost 0.04 \
  --close-boost-start 6 \
  --close-boost-max 3.0 \
  --max-xy-speed 12 \
  --max-z-speed 3 \
  --cmd-acc-max-xy 10 \
  --cmd-acc-max-z 3 \
  --guard-enable \
  --guard-min-close-rate 0.8 \
  --guard-kp-xy 0.45 \
  --guard-kp-z 0.65 \
  --debug
