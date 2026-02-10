#!/usr/bin/env python3
import argparse
import math
import subprocess
import time

from pymavlink import mavutil


def clamp(val, limit):
    return max(-limit, min(limit, val))


def scoped_joint_name(model, joint, use_scope):
    if not use_scope:
        return joint
    if "::" in joint:
        return joint
    return f"{model}::{joint}"


def set_joint_pos(model, joint, target, p, i, d, use_scope):
    cmd = [
        "gz",
        "joint",
        "-m",
        model,
        "-j",
        scoped_joint_name(model, joint, use_scope),
        "--pos-t",
        f"{target:.6f}",
        "--pos-p",
        str(p),
        "--pos-i",
        str(i),
        "--pos-d",
        str(d),
    ]
    subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def main():
    parser = argparse.ArgumentParser(description="Follow plane attitude and drive carrier joints via gz joint.")
    parser.add_argument("--mavlink", default="udp:127.0.0.1:14540", help="MAVLink endpoint for plane ATTITUDE")
    parser.add_argument("--sysid", type=int, default=3, help="Plane sysid (from MAVLink)")
    parser.add_argument("--model", default="iris_aircarrier_1", help="Gazebo model name")
    parser.add_argument("--pitch-joint", default="arm_pitch_joint", help="Pitch joint name")
    parser.add_argument("--roll-joint", default="arm_roll_joint", help="Roll joint name")
    parser.add_argument("--scoped-joint", action="store_true", default=True, help="Prefix joint with model name")
    parser.add_argument("--no-scoped-joint", action="store_false", dest="scoped_joint", help="Do not prefix joint")
    parser.add_argument("--rate", type=float, default=10.0, help="Command rate (Hz)")
    parser.add_argument("--max-abs", type=float, default=0.7, help="Max abs angle (rad)")
    parser.add_argument("--roll-sign", type=float, default=1.0, help="Roll sign multiplier")
    parser.add_argument("--pitch-sign", type=float, default=1.0, help="Pitch sign multiplier")
    parser.add_argument("--pos-p", type=float, default=20.0, help="Position P gain")
    parser.add_argument("--pos-i", type=float, default=0.0, help="Position I gain")
    parser.add_argument("--pos-d", type=float, default=1.0, help="Position D gain")
    parser.add_argument("--min-delta", type=float, default=0.01, help="Min delta (rad) to resend")
    args = parser.parse_args()

    mav = mavutil.mavlink_connection(args.mavlink, autoreconnect=True)
    mav.wait_heartbeat()
    print(f"[INFO] MAVLink connected on {args.mavlink}, waiting for ATTITUDE from sysid {args.sysid}...")

    last_roll = None
    last_pitch = None
    period = 1.0 / max(1e-3, args.rate)

    while True:
        msg = mav.recv_match(type="ATTITUDE", blocking=True, timeout=1)
        if msg is None:
            continue
        if msg.get_srcSystem() != args.sysid:
            continue

        roll = clamp(msg.roll * args.roll_sign, args.max_abs)
        pitch = clamp(msg.pitch * args.pitch_sign, args.max_abs)

        if last_roll is None or abs(roll - last_roll) >= args.min_delta:
            set_joint_pos(args.model, args.roll_joint, roll, args.pos_p, args.pos_i, args.pos_d, args.scoped_joint)
            last_roll = roll

        if last_pitch is None or abs(pitch - last_pitch) >= args.min_delta:
            set_joint_pos(args.model, args.pitch_joint, pitch, args.pos_p, args.pos_i, args.pos_d, args.scoped_joint)
            last_pitch = pitch

        time.sleep(period)


if __name__ == "__main__":
    main()
