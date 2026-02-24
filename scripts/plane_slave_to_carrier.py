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


def main():
    ap = argparse.ArgumentParser(description="Force plane model to follow carrier model pose.")
    ap.add_argument("--world-name", default="default")
    ap.add_argument("--carrier-model", default="iris_aircarrier_1")
    ap.add_argument("--plane-model", default="plane_2")
    ap.add_argument("--rate", type=float, default=5.0)
    ap.add_argument("--offset-forward", type=float, default=0.0, help="m in carrier body X")
    ap.add_argument("--offset-right", type=float, default=-0.3, help="m in carrier body Y")
    ap.add_argument("--offset-up", type=float, default=0.0, help="m in world up")
    ap.add_argument("--copy-attitude", action="store_true")
    ap.add_argument("--debug", action="store_true")
    args = ap.parse_args()

    dt = 1.0 / max(args.rate, 1.0)
    print(f"[INFO] plane slave start: plane={args.plane_model} carrier={args.carrier_model}")

    while True:
        t0 = time.time()
        cpose = get_model_pose(args.carrier_model, args.world_name)
        if cpose is None:
            time.sleep(dt)
            continue

        cx, cy, cz, croll, cpitch, cyaw = cpose
        ox = math.cos(cyaw) * args.offset_forward - math.sin(cyaw) * args.offset_right
        oy = math.sin(cyaw) * args.offset_forward + math.cos(cyaw) * args.offset_right
        px = cx + ox
        py = cy + oy
        pz = cz + args.offset_up

        proll = croll if args.copy_attitude else 0.0
        ppitch = cpitch if args.copy_attitude else 0.0
        pyaw = cyaw
        set_model_pose(args.plane_model, args.world_name, px, py, pz, proll, ppitch, pyaw)

        if args.debug:
            print(f"[DBG] plane_pose=({px:.2f},{py:.2f},{pz:.2f}) yaw={pyaw:.2f}")

        sleep_t = dt - (time.time() - t0)
        if sleep_t > 0:
            time.sleep(sleep_t)


if __name__ == "__main__":
    main()
