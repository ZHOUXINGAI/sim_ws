#!/usr/bin/env python3
import argparse
import math
import re
import subprocess
import time


def run_cmd(cmd):
    return subprocess.run(cmd, text=True, capture_output=True)


def parse_pose_text(txt):
    vals = re.findall(r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?", txt or "")
    if len(vals) < 6:
        return None
    nums = [float(v) for v in vals[:6]]
    return nums[0], nums[1], nums[2], nums[3], nums[4], nums[5]


def get_model_pose(model_name, world_name):
    cmd = ["gz", "model"]
    if world_name:
        cmd += ["-w", world_name]
    cmd += ["-m", model_name, "-p"]
    p = run_cmd(cmd)
    if p.returncode != 0:
        return None
    if "Unable to get" in p.stdout or "Unable to find" in p.stdout:
        return None
    return parse_pose_text(p.stdout)


def set_model_pose(model_name, world_name, x, y, z, roll, pitch, yaw):
    cmd = ["gz", "model"]
    if world_name:
        cmd += ["-w", world_name]
    cmd += [
        "-m",
        model_name,
        "-x",
        f"{x:.6f}",
        "-y",
        f"{y:.6f}",
        "-z",
        f"{z:.6f}",
        "-R",
        f"{roll:.6f}",
        "-P",
        f"{pitch:.6f}",
        "-Y",
        f"{yaw:.6f}",
    ]
    run_cmd(cmd)


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def wrap_pi(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def main():
    ap = argparse.ArgumentParser(description="Simple carrier position follower (no MPC).")
    ap.add_argument("--world-name", default="default")
    ap.add_argument("--plane-model", default="plane_2")
    ap.add_argument("--carrier-model", default="iris_aircarrier_1")
    ap.add_argument("--rate", type=float, default=5.0, help="control rate in Hz")
    ap.add_argument("--offset-x", type=float, default=-20.0, help="target offset in plane body X (m)")
    ap.add_argument("--offset-y", type=float, default=0.0, help="target offset in plane body Y (m)")
    ap.add_argument("--offset-z", type=float, default=-2.0, help="target offset in world Z (m)")
    ap.add_argument("--kp-xy", type=float, default=0.8)
    ap.add_argument("--kp-z", type=float, default=0.8)
    ap.add_argument("--kp-yaw", type=float, default=1.2)
    ap.add_argument("--max-vxy", type=float, default=5.0, help="max XY speed command (m/s)")
    ap.add_argument("--max-vz", type=float, default=2.0, help="max Z speed command (m/s)")
    ap.add_argument("--max-wz", type=float, default=1.0, help="max yaw rate command (rad/s)")
    ap.add_argument("--follow-yaw", action="store_true", help="align carrier yaw to plane yaw")
    ap.add_argument(
        "--direct-set",
        action="store_true",
        default=True,
        help="Directly set carrier to target position each cycle (default on).",
    )
    ap.add_argument(
        "--no-direct-set",
        dest="direct_set",
        action="store_false",
        help="Use velocity-integrated position command instead of direct-set.",
    )
    ap.add_argument("--follow-attitude", action="store_true", help="copy plane roll/pitch to carrier")
    ap.add_argument("--roll-gain", type=float, default=1.0)
    ap.add_argument("--pitch-gain", type=float, default=1.0)
    ap.add_argument("--max-tilt", type=float, default=0.9, help="attitude clamp (rad)")
    ap.add_argument("--debug", action="store_true")
    args = ap.parse_args()

    dt = 1.0 / max(args.rate, 1.0)
    print(
        f"[INFO] follow start: plane={args.plane_model} carrier={args.carrier_model} "
        f"rate={args.rate:.1f}Hz"
    )

    # Start from current carrier attitude.
    carrier_pose = get_model_pose(args.carrier_model, args.world_name)
    if carrier_pose is None:
        raise RuntimeError(f"Carrier model not found: {args.carrier_model}")
    cx, cy, cz, cr, cp, cyaw = carrier_pose

    while True:
        t0 = time.time()

        plane_pose = get_model_pose(args.plane_model, args.world_name)
        carrier_pose = get_model_pose(args.carrier_model, args.world_name)
        if plane_pose is None or carrier_pose is None:
            time.sleep(dt)
            continue

        px, py, pz, proll, ppitch, pyaw = plane_pose
        cx, cy, cz, cr, cp, cyaw = carrier_pose

        # Plane body offset -> world offset.
        ox = math.cos(pyaw) * args.offset_x - math.sin(pyaw) * args.offset_y
        oy = math.sin(pyaw) * args.offset_x + math.cos(pyaw) * args.offset_y
        tx = px + ox
        ty = py + oy
        tz = pz + args.offset_z

        ex = tx - cx
        ey = ty - cy
        ez = tz - cz

        vx = clamp(args.kp_xy * ex, -args.max_vxy, args.max_vxy)
        vy = clamp(args.kp_xy * ey, -args.max_vxy, args.max_vxy)
        vz = clamp(args.kp_z * ez, -args.max_vz, args.max_vz)

        if not args.direct_set:
            nx = cx + vx * dt
            ny = cy + vy * dt
            nz = cz + vz * dt
        else:
            nx = tx
            ny = ty
            nz = tz

        nyaw = cyaw
        if args.follow_yaw:
            eyaw = wrap_pi(pyaw - cyaw)
            wz = clamp(args.kp_yaw * eyaw, -args.max_wz, args.max_wz)
            nyaw = wrap_pi(cyaw + wz * dt)

        nroll = cr
        npitch = cp
        if args.follow_attitude:
            nroll = clamp(proll * args.roll_gain, -args.max_tilt, args.max_tilt)
            npitch = clamp(ppitch * args.pitch_gain, -args.max_tilt, args.max_tilt)

        set_model_pose(args.carrier_model, args.world_name, nx, ny, nz, nroll, npitch, nyaw)

        if args.debug:
            print(
                f"[DBG] tgt=({tx:.1f},{ty:.1f},{tz:.1f}) "
                f"car=({nx:.1f},{ny:.1f},{nz:.1f}) "
                f"e=({ex:.1f},{ey:.1f},{ez:.1f}) "
                f"att=({nroll:.2f},{npitch:.2f},{nyaw:.2f})"
            )

        sleep_t = dt - (time.time() - t0)
        if sleep_t > 0:
            time.sleep(sleep_t)


if __name__ == "__main__":
    main()
