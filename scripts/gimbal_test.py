#!/usr/bin/env python3
import argparse
import math
import socket
import sys
import time

try:
    from pymavlink import mavutil
    from pymavlink.dialects.v20 import common as mavlink2
except Exception as e:
    print("[ERROR] pymavlink not installed. Run: python3 -m pip install pymavlink")
    sys.exit(1)


def euler_to_quat(roll, pitch, yaw):
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return (w, x, y, z)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--roll", type=float, default=0.0, help="roll deg")
    ap.add_argument("--pitch", type=float, default=0.0, help="pitch deg")
    ap.add_argument("--yaw", type=float, default=0.0, help="yaw deg")
    ap.add_argument("--rate", type=float, default=0.2, help="send duration seconds")
    ap.add_argument("--port", type=int, default=13031, help="listen port")
    args = ap.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", args.port))
    sock.settimeout(0.5)

    print(f"[INFO] Listening for gimbal heartbeat on UDP :{args.port} ...")
    sender = None
    # Use MAVLink2 common dialect for gimbal_device_set_attitude
    mav = mavlink2.MAVLink(None)
    mav.srcSystem = 255
    mav.srcComponent = 190

    # wait for any incoming packet to learn sender addr
    start = time.time()
    while time.time() - start < 10.0:
        try:
            data, addr = sock.recvfrom(4096)
        except socket.timeout:
            continue
        if data:
            sender = addr
            print(f"[INFO] Got packet from {sender}")
            break

    if sender is None:
        print("[ERROR] No gimbal packets received. Make sure sim is running and gimbal plugin is loaded.")
        sys.exit(1)

    roll = math.radians(args.roll)
    pitch = math.radians(args.pitch)
    yaw = math.radians(args.yaw)
    quat = euler_to_quat(roll, pitch, yaw)

    msg = mavlink2.MAVLink_gimbal_device_set_attitude_message(
        target_system=0,
        target_component=0,
        flags=0,
        q=quat,
        angular_velocity_x=float("nan"),
        angular_velocity_y=float("nan"),
        angular_velocity_z=float("nan"),
    )

    duration = max(0.1, args.rate)
    end = time.time() + duration
    print(f"[INFO] Sending gimbal command to {sender} for {duration:.2f}s")
    while time.time() < end:
        buf = msg.pack(mav)
        sock.sendto(buf, sender)
        time.sleep(0.1)

    print("[INFO] Done")


if __name__ == "__main__":
    main()
