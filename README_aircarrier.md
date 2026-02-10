# sim_ws Quad & Aircarrier Quick Start

This repo includes two launch scripts:
- `scripts/run_px4_quad.sh` (standard iris quad)
- `scripts/run_px4_aircarrier.sh` (iris + 2-axis gimbal + top platform)

## 1) Run the quad (iris)
```bash
cd /home/hw/sim_ws
./scripts/run_px4_quad.sh
```

## 2) Run the aircarrier
```bash
cd /home/hw/sim_ws
./scripts/run_px4_aircarrier.sh
```

You should see Gazebo open and QGC can connect on UDP 14550.

## 3) Gimbal test (simple)
There are two easy options:

### Option A: Use QGC UI
1. Start `run_px4_aircarrier.sh`
2. Open QGroundControl
3. Try gimbal controls (if shown). If no gimbal UI appears, use Option B below.

### Option B: Use the included Python script
This script listens on UDP 13030 (gimbal plugin output) and sends a roll/pitch command
back to the plugin's source port.

```bash
cd /home/hw/sim_ws
python3 scripts/gimbal_test.py --roll 10 --pitch -15 --rate 0.5
```

If `pymavlink` is missing, install it:
```bash
python3 -m pip install pymavlink
```

Notes:
- The gimbal plugin sends a heartbeat to UDP 13031 (to avoid conflict with PX4's 13030). The script waits for it.

## 4) Attitude follow (fixed-wing -> carrier platform)
This script reads the fixed-wing ATTITUDE over MAVLink and applies roll/pitch to the carrier platform joints.

Example (fixed-wing MAVLink on 14540):
```bash
cd /home/hw/sim_ws
python3 scripts/attitude_follow.py --mavlink udp:127.0.0.1:14540 --model iris_aircarrier
```
- After receiving the heartbeat, it sends `GIMBAL_DEVICE_SET_ATTITUDE`.
