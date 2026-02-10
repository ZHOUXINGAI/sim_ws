#!/usr/bin/env python3
import argparse
import math
import re
import subprocess
import time

from pymavlink import mavutil


def clamp(v, lim):
    return max(-lim, min(lim, v))


def rpy_to_quat(roll, pitch, yaw):
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
    return x, y, z, w


def get_model_pose(model):
    # gz model -m <name> -p -> x y z roll pitch yaw
    out = subprocess.check_output(["gz", "model", "-m", model, "-p"], text=True).strip()
    vals = [float(x) for x in re.findall(r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?", out)]
    if len(vals) < 6:
        raise RuntimeError(f"Unexpected gz model output: {out}")
    return vals[0], vals[1], vals[2], vals[3], vals[4], vals[5]


def publish_pose(link_name, x, y, z, qx, qy, qz, qw):
    msg = (
        f'name: "{link_name}" '
        f'position {{ x: {x:.6f} y: {y:.6f} z: {z:.6f} }} '
        f'orientation {{ x: {qx:.8f} y: {qy:.8f} z: {qz:.8f} w: {qw:.8f} }}'
    )
    subprocess.run(
        [
            "gz",
            "topic",
            "-t",
            "/gazebo/default/pose/modify",
            "-m",
            "gazebo.msgs.Pose",
            "-p",
            msg,
        ],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        check=False,
    )


def publish_pose_with_id(link_name, link_id, x, y, z, qx, qy, qz, qw):
    msg = (
        f'name: "{link_name}" id: {int(link_id)} '
        f'position {{ x: {x:.6f} y: {y:.6f} z: {z:.6f} }} '
        f'orientation {{ x: {qx:.8f} y: {qy:.8f} z: {qz:.8f} w: {qw:.8f} }}'
    )
    subprocess.run(
        [
            "gz",
            "topic",
            "-t",
            "/gazebo/default/pose/modify",
            "-m",
            "gazebo.msgs.Pose",
            "-p",
            msg,
        ],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        check=False,
    )


def get_pose_id_by_name(name):
    try:
        txt = subprocess.check_output(
            ["gz", "topic", "-e", "/gazebo/default/pose/info", "-d", "1"],
            text=True,
            stderr=subprocess.DEVNULL,
        )
    except Exception:
        return None

    # Parse repeated pose blocks and match by exact name
    blocks = txt.split("pose {")
    for b in blocks:
        if f'name: "{name}"' not in b:
            continue
        m = re.search(r"\bid:\s*(\d+)", b)
        if m:
            return int(m.group(1))
    return None


def resolve_link_candidates(carrier_model, target_link):
    if "::" in target_link:
        return [target_link]
    base_model = re.sub(r"_[0-9]+$", "", carrier_model)
    candidates = [f"{carrier_model}::{target_link}", f"{base_model}::{target_link}"]
    # keep order, remove duplicates
    out = []
    for c in candidates:
        if c not in out:
            out.append(c)
    return out


def main():
    ap = argparse.ArgumentParser(description="Follow plane ATTITUDE and drive simplified deck link pose.")
    ap.add_argument("--mavlink", default="udp:127.0.0.1:14542")
    ap.add_argument("--sysid", type=int, default=3)
    ap.add_argument("--carrier-model", default="iris_aircarrier_1")
    ap.add_argument("--target-link", default="deck_link")
    ap.add_argument("--rate", type=float, default=20.0)
    ap.add_argument("--max-tilt", type=float, default=0.7, help="rad")
    # target link world position = base position + offset (in base yaw frame)
    ap.add_argument("--offset-x", type=float, default=0.0)
    ap.add_argument("--offset-y", type=float, default=0.0)
    ap.add_argument("--offset-z", type=float, default=0.0)
    ap.add_argument("--roll-sign", type=float, default=1.0)
    ap.add_argument("--pitch-sign", type=float, default=1.0)
    ap.add_argument("--debug", action="store_true")
    args = ap.parse_args()

    link_candidates = resolve_link_candidates(args.carrier_model, args.target_link)

    mav = mavutil.mavlink_connection(args.mavlink, autoreconnect=True)
    mav.wait_heartbeat()
    print(f"[INFO] MAVLink connected: {args.mavlink}, waiting ATTITUDE sysid={args.sysid}")
    print(f"[INFO] Target link candidates: {', '.join(link_candidates)}")
    link_id_map = {}
    for ln in link_candidates:
        lid = get_pose_id_by_name(ln)
        if lid is not None:
            link_id_map[ln] = lid
    if link_id_map:
        pretty = ", ".join([f"{k}=>{v}" for k, v in link_id_map.items()])
        print(f"[INFO] Resolved pose IDs: {pretty}")
    else:
        print("[WARN] No pose ID resolved from /pose/info, fallback to name-only publish")

    dt = 1.0 / max(1e-3, args.rate)
    last_dbg = 0.0

    while True:
        msg = mav.recv_match(type="ATTITUDE", blocking=True, timeout=1.0)
        if msg is None:
            continue
        if msg.get_srcSystem() != args.sysid:
            continue

        # plane attitude -> target roll/pitch
        roll = clamp(msg.roll * args.roll_sign, args.max_tilt)
        pitch = clamp(msg.pitch * args.pitch_sign, args.max_tilt)

        # carrier base pose (for position and yaw hold)
        try:
            bx, by, bz, _, _, byaw = get_model_pose(args.carrier_model)
        except Exception:
            time.sleep(dt)
            continue

        # offset in world using carrier yaw
        ox = math.cos(byaw) * args.offset_x - math.sin(byaw) * args.offset_y
        oy = math.sin(byaw) * args.offset_x + math.cos(byaw) * args.offset_y
        tx, ty, tz = bx + ox, by + oy, bz + args.offset_z

        # keep yaw as carrier yaw, only follow roll/pitch
        qx, qy, qz, qw = rpy_to_quat(roll, pitch, byaw)

        for link_name in link_candidates:
            lid = link_id_map.get(link_name)
            if lid is not None:
                publish_pose_with_id(link_name, lid, tx, ty, tz, qx, qy, qz, qw)
            else:
                publish_pose(link_name, tx, ty, tz, qx, qy, qz, qw)
        if args.debug:
            now = time.time()
            if now - last_dbg > 1.0:
                print(
                    f"[DBG] src roll={msg.roll:.3f} pitch={msg.pitch:.3f} -> cmd roll={roll:.3f} pitch={pitch:.3f}"
                )
                last_dbg = now
        time.sleep(dt)


if __name__ == "__main__":
    main()
