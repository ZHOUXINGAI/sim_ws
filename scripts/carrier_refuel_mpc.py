#!/usr/bin/env python3
import argparse
import math
import re
import subprocess
import time

import numpy as np
from pymavlink import mavutil


PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6


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


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def clamp_norm3(vn, ve, vd, max_xy, max_z):
    h = math.hypot(vn, ve)
    if h > max_xy and h > 1e-6:
        s = max_xy / h
        vn *= s
        ve *= s
    vd = clamp(vd, -max_z, max_z)
    return vn, ve, vd


def wrap_pi(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def yaw_from_velocity(vn, ve, last_yaw):
    if math.hypot(vn, ve) > 0.5:
        return math.atan2(ve, vn)
    return last_yaw


def send_vel_sp(conn, target_sys, target_comp, vn, ve, vd, yaw):
    type_mask = (
        (1 << 0) | (1 << 1) | (1 << 2) |
        (1 << 6) | (1 << 7) | (1 << 8) |
        (1 << 11)
    )
    conn.mav.set_position_target_local_ned_send(
        0,
        target_sys,
        target_comp,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        type_mask,
        0.0, 0.0, 0.0,
        float(vn), float(ve), float(vd),
        0.0, 0.0, 0.0,
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


def arm(conn, target_sys, target_comp, do_arm=True):
    conn.mav.command_long_send(
        target_sys,
        target_comp,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1.0 if do_arm else 0.0,
        0,
        0,
        0,
        0,
        0,
        0,
    )


def set_param(conn, target_sys, target_comp, name, value):
    conn.mav.param_set_send(
        target_sys,
        target_comp,
        name.encode("utf-8"),
        float(value),
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
    )


def request_message_interval(conn, target_sys, target_comp, msg_id, hz):
    us = int(1e6 / max(0.1, float(hz)))
    conn.mav.command_long_send(
        target_sys,
        target_comp,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        float(msg_id),
        float(us),
        0,
        0,
        0,
        0,
        0,
    )


def request_streams(conn, target_sys, target_comp, hz):
    request_message_interval(conn, target_sys, target_comp, mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, hz)
    request_message_interval(conn, target_sys, target_comp, mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, hz)


def finite_horizon_lqr_first_gain(dt, horizon, q_xyz, r_xyz, qf_xyz):
    # z(k+1) = A z(k) + B v(k), with A=I, B=-dt*I
    a = np.eye(3)
    b = -dt * np.eye(3)
    q = np.diag(q_xyz)
    r = np.diag(r_xyz)
    p = np.diag(qf_xyz)
    k0 = np.zeros((3, 3))
    for i in range(horizon, 0, -1):
        bt_p = b.T @ p
        s = r + bt_p @ b
        k = np.linalg.solve(s, bt_p @ a)
        if i == 1:
            k0 = k
        p = q + a.T @ p @ (a - b @ k)
    return k0


def main():
    ap = argparse.ArgumentParser(description="Carrier refuel MPC: keep 1m below plane and relative velocity -> 0.")
    ap.add_argument("--plane-url", default="udp:127.0.0.1:14542")
    ap.add_argument("--carrier-url", default="udp:127.0.0.1:14541")
    ap.add_argument("--plane-sysid", type=int, default=3)
    ap.add_argument("--carrier-sysid", type=int, default=2)
    ap.add_argument("--rate", type=float, default=20.0)
    ap.add_argument("--stream-rate", type=float, default=30.0)
    ap.add_argument("--world-name", default="default")
    ap.add_argument("--use-gz-pose", action="store_true", default=True)
    ap.add_argument("--plane-model", default="plane_2")
    ap.add_argument("--carrier-model", default="iris_aircarrier_1")
    ap.add_argument("--gz-pose-rate", type=float, default=20.0)
    ap.add_argument("--target-rel-n", type=float, default=0.0)
    ap.add_argument("--target-rel-e", type=float, default=0.0)
    ap.add_argument("--target-rel-d", type=float, default=-1.0, help="plane minus carrier in NED down; -1 means carrier 1m below")
    ap.add_argument("--mpc-horizon", type=int, default=18)
    ap.add_argument("--q-n", type=float, default=2.0)
    ap.add_argument("--q-e", type=float, default=2.0)
    ap.add_argument("--q-d", type=float, default=4.0)
    ap.add_argument("--r-vn", type=float, default=0.12)
    ap.add_argument("--r-ve", type=float, default=0.12)
    ap.add_argument("--r-vd", type=float, default=0.25)
    ap.add_argument("--qf-scale", type=float, default=6.0)
    ap.add_argument("--max-xy-speed", type=float, default=18.0)
    ap.add_argument("--max-z-speed", type=float, default=3.0)
    ap.add_argument("--yaw-follow-plane-vel", action="store_true", default=True)
    ap.add_argument("--offboard-refresh-sec", type=float, default=0.5)
    ap.add_argument("--arm-carrier", action="store_true", default=True)
    ap.add_argument("--wait-arm", type=float, default=12.0)
    ap.add_argument("--relax-arm-checks", action="store_true", default=True)
    ap.add_argument("--debug", action="store_true")
    args = ap.parse_args()

    dt = 1.0 / max(1.0, args.rate)
    gz_dt = 1.0 / max(1.0, args.gz_pose_rate)

    plane = mavutil.mavlink_connection(args.plane_url, autoreconnect=True)
    carrier = mavutil.mavlink_connection(args.carrier_url, autoreconnect=True)
    phb = plane.wait_heartbeat(timeout=10)
    chb = carrier.wait_heartbeat(timeout=10)
    if phb is None or chb is None:
        raise RuntimeError("heartbeat timeout")
    plane_sys = args.plane_sysid or phb.get_srcSystem()
    plane_comp = phb.get_srcComponent()
    carrier_sys = args.carrier_sysid or chb.get_srcSystem()
    carrier_comp = chb.get_srcComponent()

    print(f"[INFO] connected: plane sys={plane_sys}, carrier sys={carrier_sys}")

    request_streams(plane, plane_sys, plane_comp, args.stream_rate)
    request_streams(carrier, carrier_sys, carrier_comp, args.stream_rate)

    if args.relax_arm_checks:
        set_param(carrier, carrier_sys, carrier_comp, "COM_RC_IN_MODE", 4)
        set_param(carrier, carrier_sys, carrier_comp, "COM_ARM_WO_GPS", 1)
        set_param(carrier, carrier_sys, carrier_comp, "COM_ARM_CHK", 0)
        set_param(carrier, carrier_sys, carrier_comp, "COM_DISARM_PRFLT", -1)
        set_param(carrier, carrier_sys, carrier_comp, "NAV_RCL_ACT", 0)
        set_param(carrier, carrier_sys, carrier_comp, "NAV_DLL_ACT", 0)
        set_param(carrier, carrier_sys, carrier_comp, "COM_OF_LOSS_T", 5)
        set_param(carrier, carrier_sys, carrier_comp, "COM_OBL_RC_ACT", 0)
        time.sleep(0.2)
        print("[INFO] applied relaxed arm-check params on carrier")

    # offboard prime
    prime_end = time.time() + 1.0
    while time.time() < prime_end:
        send_vel_sp(carrier, carrier_sys, carrier_comp, 0.0, 0.0, 0.0, 0.0)
        time.sleep(dt)
    set_mode_offboard(carrier, carrier_sys, carrier_comp)

    if args.arm_carrier:
        t_end = time.time() + args.wait_arm
        while time.time() < t_end:
            arm(carrier, carrier_sys, carrier_comp, True)
            send_vel_sp(carrier, carrier_sys, carrier_comp, 0.0, 0.0, 0.0, 0.0)
            time.sleep(0.2)
        print("[INFO] arm requests sent (verify armed in QGC)")

    q_xyz = np.array([args.q_n, args.q_e, args.q_d], dtype=float)
    r_xyz = np.array([args.r_vn, args.r_ve, args.r_vd], dtype=float)
    qf_xyz = q_xyz * args.qf_scale
    k0 = finite_horizon_lqr_first_gain(dt, max(2, args.mpc_horizon), q_xyz, r_xyz, qf_xyz)
    if args.debug:
        print(f"[DBG] K0=\n{k0}")

    rel_target = np.array([args.target_rel_n, args.target_rel_e, args.target_rel_d], dtype=float)
    last_offboard_refresh = 0.0
    last_dbg = 0.0
    last_yaw = 0.0

    plane_pos = None
    carrier_pos = None
    carrier_att = None
    plane_wpose = None
    carrier_wpose = None
    prev_plane_w = None
    prev_gz_t = None
    pvn = 0.0
    pve = 0.0
    pvd = 0.0

    while True:
        t_loop = time.time()

        # keep MAVLink telemetry fresh for fallback
        mp = plane.recv_match(type="LOCAL_POSITION_NED", blocking=False)
        while mp is not None:
            if mp.get_srcSystem() == plane_sys:
                plane_pos = mp
            mp = plane.recv_match(type="LOCAL_POSITION_NED", blocking=False)
        cp = carrier.recv_match(type="LOCAL_POSITION_NED", blocking=False)
        while cp is not None:
            if cp.get_srcSystem() == carrier_sys:
                carrier_pos = cp
            cp = carrier.recv_match(type="LOCAL_POSITION_NED", blocking=False)
        ca = carrier.recv_match(type="ATTITUDE", blocking=False)
        while ca is not None:
            if ca.get_srcSystem() == carrier_sys:
                carrier_att = ca
            ca = carrier.recv_match(type="ATTITUDE", blocking=False)

        rel_ned = None

        if args.use_gz_pose and (plane_wpose is None or (time.time() - (prev_gz_t or 0.0)) >= gz_dt):
            p = get_model_pose(args.plane_model, args.world_name)
            c = get_model_pose(args.carrier_model, args.world_name)
            if p is not None and c is not None:
                plane_wpose = p
                carrier_wpose = c
                now_gz = time.time()
                # ENU -> NED relative
                rel_n = p[1] - c[1]
                rel_e = p[0] - c[0]
                rel_d = -(p[2] - c[2])
                rel_ned = np.array([rel_n, rel_e, rel_d], dtype=float)
                if prev_plane_w is not None and prev_gz_t is not None:
                    dts = max(1e-3, now_gz - prev_gz_t)
                    pvn = (p[1] - prev_plane_w[1]) / dts
                    pve = (p[0] - prev_plane_w[0]) / dts
                    pvd = -((p[2] - prev_plane_w[2]) / dts)
                prev_plane_w = p
                prev_gz_t = now_gz

        if rel_ned is None and plane_pos is not None and carrier_pos is not None:
            rel_ned = np.array(
                [plane_pos.x - carrier_pos.x, plane_pos.y - carrier_pos.y, plane_pos.z - carrier_pos.z],
                dtype=float,
            )
            pvn = float(plane_pos.vx)
            pve = float(plane_pos.vy)
            pvd = float(plane_pos.vz)

        if rel_ned is None:
            send_vel_sp(carrier, carrier_sys, carrier_comp, 0.0, 0.0, 0.0, last_yaw)
            if args.debug and (time.time() - last_dbg) >= 1.0:
                print("[DBG] waiting telemetry (plane/carrier pose)")
                last_dbg = time.time()
            time.sleep(dt)
            continue

        # MPC state: z = rel - rel_target
        z = rel_ned - rel_target
        v_plane = np.array([pvn, pve, pvd], dtype=float)

        # First control from finite-horizon LQR (receding horizon MPC).
        # v = vc - vp, so vc = vp + v
        v_corr = -k0 @ z
        v_cmd = v_plane + v_corr

        vn_cmd, ve_cmd, vd_cmd = clamp_norm3(
            float(v_cmd[0]), float(v_cmd[1]), float(v_cmd[2]), args.max_xy_speed, args.max_z_speed
        )

        if args.yaw_follow_plane_vel:
            yaw_sp = yaw_from_velocity(pvn, pve, last_yaw)
        else:
            yaw_sp = carrier_att.yaw if carrier_att is not None else last_yaw
        yaw_sp = wrap_pi(yaw_sp)
        last_yaw = yaw_sp

        send_vel_sp(carrier, carrier_sys, carrier_comp, vn_cmd, ve_cmd, vd_cmd, yaw_sp)
        if time.time() - last_offboard_refresh >= max(0.2, args.offboard_refresh_sec):
            set_mode_offboard(carrier, carrier_sys, carrier_comp)
            last_offboard_refresh = time.time()

        if args.debug and (time.time() - last_dbg) >= 0.2:
            rel_vn = pvn - vn_cmd
            rel_ve = pve - ve_cmd
            rel_vd = pvd - vd_cmd
            print(
                f"[DBG] rel=({rel_ned[0]:.2f},{rel_ned[1]:.2f},{rel_ned[2]:.2f}) "
                f"target=({rel_target[0]:.2f},{rel_target[1]:.2f},{rel_target[2]:.2f}) "
                f"e=({z[0]:.2f},{z[1]:.2f},{z[2]:.2f}) "
                f"rel_v=({rel_vn:.2f},{rel_ve:.2f},{rel_vd:.2f}) "
                f"cmd_v=({vn_cmd:.2f},{ve_cmd:.2f},{vd_cmd:.2f})"
            )
            last_dbg = time.time()

        sleep_t = dt - (time.time() - t_loop)
        if sleep_t > 0:
            time.sleep(sleep_t)


if __name__ == "__main__":
    main()

