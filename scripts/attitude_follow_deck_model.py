#!/usr/bin/env python3
import argparse
import math
import re
import subprocess
import time
from pathlib import Path

from pymavlink import mavutil


DEFAULT_DECK_SDF = (
    "/home/hw/sim_ws/px4/PX4-Autopilot/Tools/simulation/gazebo-classic/"
    "sitl_gazebo-classic/models/deck_marker/model.sdf"
)


def clamp(v, lim):
    return max(-lim, min(lim, v))


def run_cmd(cmd):
    return subprocess.run(cmd, text=True, capture_output=True)


def parse_pose_text(txt):
    vals = re.findall(r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?", txt or "")
    if len(vals) < 6:
        return None
    nums = [float(x) for x in vals[:6]]
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


def model_exists(model_name, world_name):
    return get_model_pose(model_name, world_name) is not None


def spawn_deck_model(model_name, world_name, sdf_file, x, y, z):
    # This Gazebo CLI needs both --spawn-file and --model-name.
    cmd = ["gz", "model"]
    if world_name:
        cmd += ["-w", world_name]
    cmd += [
        f"--spawn-file={sdf_file}",
        f"--model-name={model_name}",
        "-x",
        f"{x:.6f}",
        "-y",
        f"{y:.6f}",
        "-z",
        f"{z:.6f}",
    ]
    return run_cmd(cmd)


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


def wait_model(model_name, world_name, timeout=5.0):
    t0 = time.time()
    while time.time() - t0 < timeout:
        pose = get_model_pose(model_name, world_name)
        if pose is not None:
            return pose
        time.sleep(0.1)
    return None


def main():
    ap = argparse.ArgumentParser(description="Spawn simple deck model and make it follow plane roll/pitch.")
    ap.add_argument("--mavlink", default="udp:127.0.0.1:14542")
    ap.add_argument("--sysid", type=int, default=3)
    ap.add_argument("--carrier-model", default="iris_aircarrier_1")
    ap.add_argument("--deck-model", default="deck_marker_1")
    ap.add_argument("--world-name", default="default")
    ap.add_argument("--deck-sdf", default=DEFAULT_DECK_SDF)
    ap.add_argument("--rate", type=float, default=3.0)
    ap.add_argument("--max-tilt", type=float, default=0.8)
    ap.add_argument("--offset-x", type=float, default=0.0)
    ap.add_argument("--offset-y", type=float, default=0.0)
    ap.add_argument("--offset-z", type=float, default=0.0)
    ap.add_argument("--deck-z", type=float, default=0.20)
    ap.add_argument(
        "--carrier-static",
        action="store_true",
        help="Assume carrier pose fixed after startup to reduce Gazebo CLI calls.",
    )
    ap.add_argument("--roll-sign", type=float, default=1.0)
    ap.add_argument("--pitch-sign", type=float, default=1.0)
    ap.add_argument("--roll-gain", type=float, default=1.0)
    ap.add_argument("--pitch-gain", type=float, default=1.0)
    ap.add_argument("--debug", action="store_true")
    ap.add_argument(
        "--debug-interval",
        type=float,
        default=0.05,
        help="Debug print interval in seconds (only when --debug).",
    )
    ap.add_argument(
        "--carrier-pose-interval",
        type=float,
        default=2.0,
        help="How often to refresh carrier pose from Gazebo.",
    )
    ap.add_argument(
        "--deck-check-interval",
        type=float,
        default=10.0,
        help="How often to check/respawn deck model.",
    )
    args = ap.parse_args()

    sdf_file = Path(args.deck_sdf)
    if not sdf_file.exists():
        raise RuntimeError(f"Deck SDF not found: {sdf_file}")

    mav = mavutil.mavlink_connection(args.mavlink, autoreconnect=True)
    mav.wait_heartbeat()
    print(f"[INFO] MAVLink connected: {args.mavlink}, waiting ATTITUDE sysid={args.sysid}")

    carrier_pose = wait_model(args.carrier_model, args.world_name, timeout=10.0)
    if carrier_pose is None:
        raise RuntimeError(f"Carrier model not found: {args.carrier_model}")

    bx, by, bz, _, _, byaw = carrier_pose
    deck_name = args.deck_model
    if not model_exists(deck_name, args.world_name):
        sp = spawn_deck_model(deck_name, args.world_name, str(sdf_file), bx, by, bz + args.deck_z)
        time.sleep(0.3)
        if args.debug:
            if sp.stdout.strip():
                print("[DBG] spawn stdout:")
                print(sp.stdout.strip())
            if sp.stderr.strip():
                print("[DBG] spawn stderr:")
                print(sp.stderr.strip())
        if not model_exists(deck_name, args.world_name):
            # fallback name avoids collision with stale deleted model names
            deck_name = f"{args.deck_model}_{int(time.time())}"
            sp2 = spawn_deck_model(deck_name, args.world_name, str(sdf_file), bx, by, bz + args.deck_z)
            time.sleep(0.3)
            if args.debug:
                if sp2.stdout.strip():
                    print("[DBG] fallback spawn stdout:")
                    print(sp2.stdout.strip())
                if sp2.stderr.strip():
                    print("[DBG] fallback spawn stderr:")
                    print(sp2.stderr.strip())
            if not model_exists(deck_name, args.world_name):
                raise RuntimeError(f"Failed to spawn deck model using sdf={sdf_file}")

    print(f"[INFO] Deck model: {deck_name}")

    dt = 1.0 / max(args.rate, 1.0)
    last_dbg = 0.0
    last_carrier_update = 0.0
    last_deck_check = 0.0
    cached_carrier = get_model_pose(args.carrier_model, args.world_name)
    if cached_carrier is None:
        raise RuntimeError(f"Carrier model pose read failed: {args.carrier_model}")

    while True:
        t_loop = time.time()
        first = mav.recv_match(type="ATTITUDE", blocking=True, timeout=1.0)
        if first is None:
            continue

        # Drain queued ATTITUDE packets and keep the newest one to avoid lag.
        msg = first
        dropped = 0
        while True:
            nxt = mav.recv_match(type="ATTITUDE", blocking=False)
            if nxt is None:
                break
            msg = nxt
            dropped += 1

        if msg.get_srcSystem() != args.sysid:
            continue

        now = time.time()
        if (not args.carrier_static) and (now - last_carrier_update >= max(0.01, args.carrier_pose_interval)):
            pose = get_model_pose(args.carrier_model, args.world_name)
            if pose is not None:
                cached_carrier = pose
            last_carrier_update = now

        bx, by, bz, _, _, byaw = cached_carrier

        roll = clamp(msg.roll * args.roll_sign * args.roll_gain, args.max_tilt)
        pitch = clamp(msg.pitch * args.pitch_sign * args.pitch_gain, args.max_tilt)

        ox = math.cos(byaw) * args.offset_x - math.sin(byaw) * args.offset_y
        oy = math.sin(byaw) * args.offset_x + math.cos(byaw) * args.offset_y
        x = bx + ox
        y = by + oy
        z = bz + args.deck_z + args.offset_z

        if now - last_deck_check >= max(0.2, args.deck_check_interval):
            if not model_exists(deck_name, args.world_name):
                # Respawn if user deleted deck in GUI.
                spawn_deck_model(deck_name, args.world_name, str(sdf_file), x, y, z)
                time.sleep(0.1)
                if not model_exists(deck_name, args.world_name):
                    last_deck_check = now
                    continue
            last_deck_check = now

        set_model_pose(deck_name, args.world_name, x, y, z, roll, pitch, byaw)

        if args.debug:
            if now - last_dbg >= max(0.01, args.debug_interval):
                loop_dt = max(1e-6, time.time() - t_loop)
                print(
                    f"[DBG] src roll={msg.roll:.3f} pitch={msg.pitch:.3f} "
                    f"-> deck roll={roll:.3f} pitch={pitch:.3f} dropped={dropped} "
                    f"loop_hz={1.0/loop_dt:.1f}"
                )
                last_dbg = now

        sleep_t = dt - (time.time() - t_loop)
        if sleep_t > 0:
            time.sleep(sleep_t)


if __name__ == "__main__":
    main()
