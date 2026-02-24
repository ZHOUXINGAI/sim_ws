#!/usr/bin/env python3
import argparse
import time

from pymavlink import mavutil


PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6


def send_pos_sp(conn, target_sys, target_comp, n, e, d, yaw):
    type_mask = (
        (1 << 3) | (1 << 4) | (1 << 5) |
        (1 << 6) | (1 << 7) | (1 << 8) |
        (1 << 11)
    )
    conn.mav.set_position_target_local_ned_send(
        0,
        target_sys,
        target_comp,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        type_mask,
        float(n),
        float(e),
        float(d),
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        float(yaw),
        0.0,
    )


def set_mode_offboard(conn, target_sys, target_comp):
    conn.mav.command_long_send(
        target_sys,
        target_comp,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        0,
        1,
        PX4_CUSTOM_MAIN_MODE_OFFBOARD,
        0,
        0,
        0,
        0,
        0,
    )


def main():
    ap = argparse.ArgumentParser(description="Return carrier to home with Offboard position setpoint.")
    ap.add_argument("--carrier-url", default="udp:127.0.0.1:14541")
    ap.add_argument("--carrier-sysid", type=int, default=2)
    ap.add_argument("--home-n", type=float, default=0.0)
    ap.add_argument("--home-e", type=float, default=0.0)
    ap.add_argument("--home-d", type=float, default=-5.0)
    ap.add_argument("--yaw", type=float, default=0.0)
    ap.add_argument("--rate", type=float, default=20.0)
    ap.add_argument("--debug", action="store_true")
    args = ap.parse_args()

    conn = mavutil.mavlink_connection(args.carrier_url, autoreconnect=True)
    hb = conn.wait_heartbeat(timeout=10)
    if hb is None:
        raise RuntimeError("carrier heartbeat timeout")
    target_sys = args.carrier_sysid or hb.get_srcSystem()
    target_comp = hb.get_srcComponent()

    dt = 1.0 / max(args.rate, 1.0)
    t_prime = time.time() + 1.0
    while time.time() < t_prime:
        send_pos_sp(conn, target_sys, target_comp, args.home_n, args.home_e, args.home_d, args.yaw)
        time.sleep(dt)

    set_mode_offboard(conn, target_sys, target_comp)
    print(f"[INFO] return-home running to NED=({args.home_n:.1f},{args.home_e:.1f},{args.home_d:.1f})")

    while True:
        send_pos_sp(conn, target_sys, target_comp, args.home_n, args.home_e, args.home_d, args.yaw)
        if args.debug:
            print("[DBG] setpoint sent")
        time.sleep(dt)


if __name__ == "__main__":
    main()

