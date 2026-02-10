#!/usr/bin/env python3
import argparse
import math
import subprocess
import time

from pymavlink import mavutil


def clamp(v, vmin, vmax):
    return max(vmin, min(vmax, v))


def run_gz_joint(model, joint, angle_rad):
    subprocess.run(
        ["gz", "joint", "-m", model, "-j", joint, "-p", str(angle_rad)],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        check=False,
    )


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mavlink", default="udp:127.0.0.1:14540", help="MAVLink input, e.g. udp:127.0.0.1:14540")
    ap.add_argument("--sysid", type=int, default=0, help="Target sysid (0 = first ATTITUDE seen)")
    ap.add_argument("--model", default="iris_aircarrier", help="Gazebo model name")
    ap.add_argument("--pitch-joint", default="servo_pitch_joint", help="Pitch joint name")
    ap.add_argument("--roll-joint", default="servo_roll_joint", help="Roll joint name")
    ap.add_argument("--max-deg", type=float, default=40.0, help="Max abs roll/pitch deg")
    ap.add_argument("--rate", type=float, default=20.0, help="Control rate Hz")
    ap.add_argument("--alpha", type=float, default=0.2, help="Low-pass alpha (0-1)")
    args = ap.parse_args()

    mav = mavutil.mavlink_connection(args.mavlink, autoreconnect=True)
    print(f"[INFO] Waiting for ATTITUDE on {args.mavlink} ...")

    target_sysid = args.sysid
    last_roll = 0.0
    last_pitch = 0.0

    period = 1.0 / max(1.0, args.rate)
    max_rad = math.radians(args.max_deg)

    while True:
        msg = mav.recv_match(type="ATTITUDE", blocking=True, timeout=1.0)
        if msg is None:
            continue

        if target_sysid == 0:
            target_sysid = msg.get_srcSystem()
            print(f"[INFO] Using sysid {target_sysid}")
        if msg.get_srcSystem() != target_sysid:
            continue

        roll = clamp(msg.roll, -max_rad, max_rad)
        pitch = clamp(msg.pitch, -max_rad, max_rad)

        # low-pass
        roll = args.alpha * roll + (1 - args.alpha) * last_roll
        pitch = args.alpha * pitch + (1 - args.alpha) * last_pitch
        last_roll, last_pitch = roll, pitch

        run_gz_joint(args.model, args.roll_joint, roll)
        run_gz_joint(args.model, args.pitch_joint, pitch)

        time.sleep(period)


if __name__ == "__main__":
    main()
