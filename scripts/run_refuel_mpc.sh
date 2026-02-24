#!/usr/bin/env bash
set -euo pipefail

cd /home/hw/sim_ws
pkill -f carrier_refuel_mpc.py || true

python3 scripts/carrier_refuel_mpc.py \
  --plane-url udp:127.0.0.1:14542 \
  --carrier-url udp:127.0.0.1:14541 \
  --plane-sysid 3 \
  --carrier-sysid 2 \
  --use-gz-pose \
  --world-name default \
  --plane-model plane_2 \
  --carrier-model iris_aircarrier_1 \
  --target-rel-n 0 \
  --target-rel-e 0 \
  --target-rel-d -1 \
  --rate 20 \
  --stream-rate 30 \
  --mpc-horizon 18 \
  --q-n 2.0 --q-e 2.0 --q-d 4.0 \
  --r-vn 0.12 --r-ve 0.12 --r-vd 0.25 \
  --qf-scale 6.0 \
  --max-xy-speed 18 \
  --max-z-speed 3 \
  --debug

