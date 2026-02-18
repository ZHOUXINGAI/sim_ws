# Carrier Offboard Follow (Simple)

This flow keeps `plane` on QGC mission control, and controls `carrier` via PX4 Offboard (MAVLink setpoint), without directly moving Gazebo model pose.

## 1) Start simulation (two PX4 instances)

```bash
cd /home/hw/sim_ws
./scripts/run_px4_carrier_plane_offboard.sh
```

Spawn order is fixed:
- `iris_aircarrier_1` -> PX4 instance 1 -> offboard UDP remote `14541` -> sysid usually `2`
- `plane_2` -> PX4 instance 2 -> offboard UDP remote `14542` -> sysid usually `3`

## 2) Fly plane with QGC mission

Use QGC to arm and start mission on `plane_2` (sysid 3).

## 3) Run carrier offboard follower

```bash
cd /home/hw/sim_ws
python3 scripts/carrier_offboard_follow_plane.py \
  --plane-url udp:127.0.0.1:14542 \
  --carrier-url udp:127.0.0.1:14541 \
  --plane-sysid 3 \
  --carrier-sysid 2 \
  --rate 20 \
  --offset-forward -20 \
  --offset-right 0 \
  --offset-down 0 \
  --follow-yaw \
  --arm \
  --debug
```

Notes:
- `--offset-forward -20` means carrier tracks 20m behind plane heading direction.
- `--offset-down` is in NED frame (`+` is down, `-` is up).
- If Offboard reject occurs, keep script running and switch carrier to Offboard from QGC manually once.

