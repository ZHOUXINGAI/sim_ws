#!/usr/bin/env python3
import argparse
import math
import re
import subprocess
import time

from pymavlink import mavutil


PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6
PX4_CUSTOM_MAIN_MODE_AUTO = 4
PX4_CUSTOM_SUB_MODE_AUTO_LOITER = 3


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


def wrap_pi(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def send_pos_sp(conn, target_sys, target_comp, n, e, d, yaw, vn=0.0, ve=0.0, vd=0.0):
    type_mask = (
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
        float(vn),
        float(ve),
        float(vd),
        0.0,
        0.0,
        0.0,
        float(yaw),
        0.0,
    )


def send_vel_sp(conn, target_sys, target_comp, vn, ve, vd, yaw):
    # Velocity-only offboard setpoint (position ignored).
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
        0.0,
        0.0,
        0.0,
        float(vn),
        float(ve),
        float(vd),
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


def set_mode_loiter(conn, target_sys, target_comp):
    conn.mav.command_long_send(
        target_sys,
        target_comp,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        0,
        1,
        PX4_CUSTOM_MAIN_MODE_AUTO,
        PX4_CUSTOM_SUB_MODE_AUTO_LOITER,
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


def request_core_streams(conn, target_sys, target_comp, hz):
    # Required by tracker state machine.
    request_message_interval(conn, target_sys, target_comp, mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, hz)
    request_message_interval(conn, target_sys, target_comp, mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, hz)
    request_message_interval(conn, target_sys, target_comp, mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE_QUATERNION, hz)


def is_armed(conn, target_sys):
    hb = conn.recv_match(type="HEARTBEAT", blocking=False)
    while hb is not None:
        if hb.get_srcSystem() == target_sys:
            return bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        hb = conn.recv_match(type="HEARTBEAT", blocking=False)
    return None


def wait_armed_heartbeat(conn, target_sys, timeout_s=0.5):
    t0 = time.time()
    while time.time() - t0 < timeout_s:
        hb = conn.recv_match(type="HEARTBEAT", blocking=False)
        if hb is None:
            time.sleep(0.01)
            continue
        if hb.get_srcSystem() != target_sys:
            continue
        return bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
    return None


def recv_status_text(conn, target_sys):
    msg = conn.recv_match(type="STATUSTEXT", blocking=False)
    while msg is not None:
        if msg.get_srcSystem() == target_sys:
            txt = getattr(msg, "text", "")
            if isinstance(txt, bytes):
                txt = txt.decode("utf-8", errors="ignore")
            return str(txt).strip()
        msg = conn.recv_match(type="STATUSTEXT", blocking=False)
    return None


def wait_command_ack(conn, target_sys, expected_cmd=None, timeout_s=0.3):
    t0 = time.time()
    while time.time() - t0 < timeout_s:
        ack = conn.recv_match(type="COMMAND_ACK", blocking=False)
        if ack is None:
            time.sleep(0.01)
            continue
        if ack.get_srcSystem() != target_sys:
            continue
        if expected_cmd is not None and int(getattr(ack, "command", -1)) != int(expected_cmd):
            continue
        return ack
    return None


def body_offset_to_ned(yaw, forward, right, down):
    dn = math.cos(yaw) * forward - math.sin(yaw) * right
    de = math.sin(yaw) * forward + math.cos(yaw) * right
    dd = down
    return dn, de, dd


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def clamp_norm2(vx, vy, vmax):
    n = math.hypot(vx, vy)
    if n <= max(1e-6, vmax):
        return vx, vy
    s = vmax / n
    return vx * s, vy * s


def lpf(old_v, new_v, alpha):
    return old_v * (1.0 - alpha) + new_v * alpha


def quat_to_yaw(qw, qx, qy, qz):
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


def yaw_from_velocity(vn, ve, last_yaw):
    if math.hypot(vn, ve) > 0.8:
        return math.atan2(ve, vn)
    return last_yaw


def predict_planar_constant_turn(n, e, vn, ve, omega, t):
    if t <= 0.0:
        return n, e
    if abs(omega) < 1e-3:
        return n + vn * t, e + ve * t
    th = omega * t
    s = math.sin(th)
    c = math.cos(th)
    dn = (s / omega) * vn + ((1.0 - c) / omega) * (-ve)
    de = (s / omega) * ve + ((1.0 - c) / omega) * (vn)
    return n + dn, e + de


def slave_plane_to_carrier(world_name, carrier_model, plane_model, off_fwd, off_right, off_up):
    c = get_model_pose(carrier_model, world_name)
    if c is None:
        return False
    cx, cy, cz, cr, cp, cyaw = c
    ox = math.cos(cyaw) * off_fwd - math.sin(cyaw) * off_right
    oy = math.sin(cyaw) * off_fwd + math.cos(cyaw) * off_right
    px = cx + ox
    py = cy + oy
    pz = cz + off_up
    set_model_pose(plane_model, world_name, px, py, pz, cr, cp, cyaw)
    return True


def main():
    ap = argparse.ArgumentParser(description="Docking manager: track -> land align -> capture -> return home.")
    ap.add_argument("--plane-url", default="udp:127.0.0.1:14542")
    ap.add_argument("--carrier-url", default="udp:127.0.0.1:14541")
    ap.add_argument("--plane-sysid", type=int, default=3)
    ap.add_argument("--carrier-sysid", type=int, default=2)
    ap.add_argument("--world-name", default="default")
    ap.add_argument("--use-gz-pose", action="store_true", help="use Gazebo world pose for relative tracking")
    ap.add_argument("--gz-pose-rate", type=float, default=5.0, help="Gazebo pose polling rate (Hz)")
    ap.add_argument("--plane-model-name", default="plane_2", help="Gazebo plane model name")
    ap.add_argument("--carrier-model-name", default="iris_aircarrier_1", help="Gazebo carrier model name")
    ap.add_argument("--gz-max-model-speed", type=float, default=40.0, help="reject Gazebo pose jump if model speed exceeds this (m/s)")
    ap.add_argument("--gz-pose-alpha", type=float, default=0.35, help="low-pass alpha for Gazebo pose [0..1]")
    ap.add_argument("--plane-model", default="plane_2")
    ap.add_argument("--carrier-model", default="iris_aircarrier_1")
    ap.add_argument("--rate", type=float, default=20.0)
    ap.add_argument("--arm-carrier", action="store_true")
    ap.add_argument("--follow-yaw", action="store_true")
    ap.add_argument("--lead-time", type=float, default=1.2, help="seconds of plane position prediction")
    ap.add_argument(
        "--yaw-mode",
        choices=["plane", "intercept", "auto"],
        default="auto",
        help="carrier yaw target policy",
    )
    ap.add_argument("--yaw-auto-switch-dist", type=float, default=2.0, help="for yaw-mode=auto")

    # stage-1 track target (carrier relative to plane body)
    ap.add_argument("--track-forward", type=float, default=-1.0)
    ap.add_argument("--track-right", type=float, default=0.0)
    ap.add_argument("--track-down", type=float, default=1.0)
    ap.add_argument("--track-xy-thr", type=float, default=0.5)
    ap.add_argument("--track-z-thr", type=float, default=0.3)
    ap.add_argument("--track-hold-sec", type=float, default=1.0)
    ap.add_argument("--xy-first", action="store_true", default=True, help="first lock XY, then consider Z")
    ap.add_argument("--no-xy-first", dest="xy_first", action="store_false", help="disable XY-first tracking")
    ap.add_argument("--xy-first-hold-sec", type=float, default=1.0, help="XY-only hold time before TRACK_Z")
    ap.add_argument("--safe-down-sep", type=float, default=1.0, help="minimum carrier down offset from plane during TRACK_XY")
    ap.add_argument("--track-d-min", type=float, default=-8.0, help="min TRACK setpoint D (NED)")
    ap.add_argument("--track-d-max", type=float, default=-1.0, help="max TRACK setpoint D (NED)")

    # stage-2 align target
    ap.add_argument("--land-forward", type=float, default=0.0)
    ap.add_argument("--land-right", type=float, default=-0.3)
    ap.add_argument("--land-down", type=float, default=0.0)
    ap.add_argument("--land-xy-thr", type=float, default=0.2)
    ap.add_argument("--land-z-thr", type=float, default=0.15)
    ap.add_argument("--land-hold-sec", type=float, default=1.0)
    ap.add_argument("--land-d-min", type=float, default=-8.0, help="min LAND setpoint D (NED)")
    ap.add_argument("--land-d-max", type=float, default=-1.0, help="max LAND setpoint D (NED)")
    ap.add_argument("--hard-capture", action="store_true", help="force CAPTURE when relative thresholds are met")
    ap.add_argument("--hard-capture-xy", type=float, default=0.2, help="hard capture XY threshold (m)")
    ap.add_argument("--hard-capture-down-min", type=float, default=0.0, help="carrier must be at least this much below plane (m, NED down diff)")
    ap.add_argument("--hard-capture-down-max", type=float, default=0.5, help="carrier must be within this much below plane (m, NED down diff)")
    ap.add_argument("--hard-capture-hold-sec", type=float, default=0.5, help="hold time before hard capture (s)")

    # capture + return
    ap.add_argument("--plane-loiter-on-capture", action="store_true")
    ap.add_argument("--kill-plane-px4-on-capture", action="store_true")
    ap.add_argument("--slave-offset-forward", type=float, default=0.0)
    ap.add_argument("--slave-offset-right", type=float, default=-0.3)
    ap.add_argument("--slave-offset-up", type=float, default=0.0)
    ap.add_argument("--home-n", type=float, default=0.0)
    ap.add_argument("--home-e", type=float, default=0.0)
    ap.add_argument("--home-d", type=float, default=-2.0)
    ap.add_argument("--home-from-start", action="store_true", default=True, help="use carrier start LOCAL_POSITION_NED as return-home target")
    ap.add_argument("--no-home-from-start", dest="home_from_start", action="store_false", help="disable auto home-from-start")
    ap.add_argument("--home-thr", type=float, default=0.5)
    ap.add_argument("--wait-arm", type=float, default=8.0, help="seconds to keep retrying arm request")
    ap.add_argument(
        "--relax-arm-checks",
        action="store_true",
        help="set COM_RC_IN_MODE=4, COM_ARM_WO_GPS=1, COM_ARM_CHK=0 on carrier before arming",
    )
    ap.add_argument("--offboard-refresh-sec", type=float, default=1.0, help="periodically resend OFFBOARD mode")
    ap.add_argument("--stream-rate", type=float, default=20.0, help="requested telemetry stream rate (Hz)")
    ap.add_argument("--ff-plane-vel", type=float, default=1.0, help="feed-forward gain on plane velocity")
    ap.add_argument("--ff-kp-pos", type=float, default=0.2, help="P gain from position error to velocity command")
    ap.add_argument("--ff-max-xy", type=float, default=12.0, help="max XY speed command (m/s)")
    ap.add_argument("--ff-max-z", type=float, default=3.0, help="max Z speed command (m/s)")
    ap.add_argument("--track-vel-mode", action="store_true", default=True, help="use velocity pursuit in TRACK stages")
    ap.add_argument("--no-track-vel-mode", dest="track_vel_mode", action="store_false", help="disable velocity pursuit")
    ap.add_argument("--track-vel-switch-xy", type=float, default=2.0, help="switch to position mode when XY error is below this")
    ap.add_argument("--dock-vel-match", action="store_true", default=True, help="near-field velocity-match mode (aerial-refuel style)")
    ap.add_argument("--no-dock-vel-match", dest="dock_vel_match", action="store_false", help="disable near-field velocity-match mode")
    ap.add_argument("--dock-match-xy", type=float, default=3.0, help="enter velocity-match when XY error is below this (m)")
    ap.add_argument("--dock-kp-xy", type=float, default=0.8, help="position correction gain in velocity-match mode")
    ap.add_argument("--dock-kp-z", type=float, default=0.8, help="z correction gain in velocity-match mode")
    ap.add_argument("--dock-max-xy", type=float, default=10.0, help="max XY speed in velocity-match mode")
    ap.add_argument("--dock-max-z", type=float, default=2.0, help="max Z speed in velocity-match mode")
    ap.add_argument("--pure-pursuit", action="store_true", help="force simple chase: v_cmd = v_plane + Kp*pos_err (no hold-distance logic)")
    ap.add_argument("--pure-kp-xy", type=float, default=1.4, help="XY Kp for --pure-pursuit")
    ap.add_argument("--pure-kp-z", type=float, default=1.0, help="Z Kp for --pure-pursuit")
    ap.add_argument("--direct-chase", action="store_true", help="force direct LOS chase at far range")
    ap.add_argument("--direct-chase-speed", type=float, default=16.0, help="far-range chase speed (m/s)")
    ap.add_argument("--direct-chase-slow-radius", type=float, default=6.0, help="switch from direct chase to normal control below this XY range (m)")
    ap.add_argument("--direct-chase-ff", type=float, default=0.2, help="plane velocity feed-forward gain in direct chase mode")
    ap.add_argument("--radial-converge", action="store_true", help="enforce radial closing speed to avoid same-radius orbiting")
    ap.add_argument("--radial-hold-dist", type=float, default=0.5, help="distance where radial close demand becomes zero (m)")
    ap.add_argument("--radial-k", type=float, default=0.6, help="closing speed gain from range error")
    ap.add_argument("--radial-max", type=float, default=4.0, help="max extra radial closing speed (m/s)")
    ap.add_argument("--intercept-time", type=float, default=0.0, help="prediction time for intercept target (s); 0 uses --lead-time")
    ap.add_argument("--intercept-adaptive", action="store_true", help="adapt prediction time by current XY range")
    ap.add_argument("--intercept-time-min", type=float, default=0.6, help="min adaptive intercept time")
    ap.add_argument("--intercept-time-max", type=float, default=3.0, help="max adaptive intercept time")
    ap.add_argument("--intercept-dist-gain", type=float, default=0.08, help="adaptive T = dist * gain")
    ap.add_argument("--debug", action="store_true")
    args = ap.parse_args()

    plane = mavutil.mavlink_connection(args.plane_url, autoreconnect=True)
    carrier = mavutil.mavlink_connection(args.carrier_url, autoreconnect=True)
    phb = plane.wait_heartbeat(timeout=10)
    chb = carrier.wait_heartbeat(timeout=10)
    if phb is None or chb is None:
        raise RuntimeError("heartbeat timeout")
    plane_sys = args.plane_sysid or phb.get_srcSystem()
    carrier_sys = args.carrier_sysid or chb.get_srcSystem()
    plane_comp = phb.get_srcComponent()
    carrier_comp = chb.get_srcComponent()
    print(f"[INFO] connected: plane sys={plane_sys}, carrier sys={carrier_sys}")
    # Ask both PX4 instances to publish required messages at a known rate.
    request_core_streams(plane, plane_sys, plane_comp, args.stream_rate)
    request_core_streams(carrier, carrier_sys, carrier_comp, args.stream_rate)

    dt = 1.0 / max(args.rate, 1.0)
    last_dbg = 0.0
    last_wait_log = 0.0
    state = "TRACK_XY" if args.xy_first else "TRACK_Z"
    state_enter_t = time.time()
    xy_ok_t0 = None
    track_ok_t0 = None
    land_ok_t0 = None
    hard_cap_ok_t0 = None
    capture_initialized = False
    last_offboard_refresh = 0.0

    # prime offboard
    prime_t = time.time() + 1.0
    while time.time() < prime_t:
        send_pos_sp(carrier, carrier_sys, carrier_comp, 0.0, 0.0, args.home_d, 0.0)
        time.sleep(dt)

    if args.relax_arm_checks:
        set_param(carrier, carrier_sys, carrier_comp, "COM_RC_IN_MODE", 4)
        set_param(carrier, carrier_sys, carrier_comp, "COM_ARM_WO_GPS", 1)
        set_param(carrier, carrier_sys, carrier_comp, "COM_ARM_CHK", 0)
        # Prevent auto preflight disarm during SITL offboard bring-up.
        set_param(carrier, carrier_sys, carrier_comp, "COM_DISARM_PRFLT", -1)
        # Avoid RC / datalink failsafe kicking us out while running script-only control.
        set_param(carrier, carrier_sys, carrier_comp, "NAV_RCL_ACT", 0)
        set_param(carrier, carrier_sys, carrier_comp, "NAV_DLL_ACT", 0)
        # Keep OFFBOARD stable in SITL if stream hiccups.
        set_param(carrier, carrier_sys, carrier_comp, "COM_OF_LOSS_T", 5)
        set_param(carrier, carrier_sys, carrier_comp, "COM_OBL_RC_ACT", 0)
        time.sleep(0.2)
        print("[INFO] applied relaxed arm-check params on carrier")

    # Enter Offboard first, then arm while continuously sending setpoints.
    set_mode_offboard(carrier, carrier_sys, carrier_comp)

    if args.arm_carrier:
        t_arm_deadline = time.time() + max(0.0, args.wait_arm)
        last_arm_req = 0.0
        armed = False
        arm_ack_ok = False
        while time.time() < t_arm_deadline:
            # Keep OFFBOARD setpoint stream alive while waiting for ARM.
            send_pos_sp(carrier, carrier_sys, carrier_comp, 0.0, 0.0, args.home_d, 0.0)
            if time.time() - last_offboard_refresh >= max(0.2, args.offboard_refresh_sec):
                set_mode_offboard(carrier, carrier_sys, carrier_comp)
                last_offboard_refresh = time.time()
            if time.time() - last_arm_req >= 0.5:
                arm(carrier, carrier_sys, carrier_comp, True)
                last_arm_req = time.time()
                ack = wait_command_ack(
                    carrier,
                    carrier_sys,
                    expected_cmd=mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                    timeout_s=0.2,
                )
                if ack is not None and args.debug:
                    print(f"[DBG] arm ack cmd={ack.command} result={ack.result}")
                if ack is not None and int(ack.result) == int(mavutil.mavlink.MAV_RESULT_ACCEPTED):
                    arm_ack_ok = True
                    # Stop arm spamming once accepted; keep waiting for armed heartbeat.
                    last_arm_req = time.time()
            st = recv_status_text(carrier, carrier_sys)
            if st and args.debug:
                print(f"[DBG] carrier status: {st}")
            a = wait_armed_heartbeat(carrier, carrier_sys, timeout_s=0.1)
            if a is None:
                a = is_armed(carrier, carrier_sys)
            if a:
                armed = True
                break
            time.sleep(0.05)
        # If COMMAND_ACK accepted but heartbeat bit is delayed/lost on this link, continue.
        if not armed and arm_ack_ok:
            armed = True
        if not armed:
            print("[WARN] carrier not armed yet; continuing to send setpoints. You can arm from QGC.")
        else:
            print("[INFO] carrier armed")
    set_mode_offboard(carrier, carrier_sys, carrier_comp)

    plane_pos = None
    plane_yaw = 0.0
    plane_yaw_src = "init"
    carrier_pos = None
    carrier_att = None
    prev_plane_v = None
    prev_plane_v_t = None
    # Always keep sending at least one valid setpoint to avoid OFFBOARD signal loss.
    tn, te, td, tyaw = 0.0, 0.0, args.home_d, 0.0
    tvn, tve, tvd = 0.0, 0.0, 0.0
    home_set = False
    gz_period = 1.0 / max(0.5, args.gz_pose_rate)
    last_gz_pose_t = 0.0
    plane_wpose = None
    carrier_wpose = None
    prev_plane_wpose_raw = None
    prev_carrier_wpose_raw = None
    prev_gz_sample_t = None
    prev_plane_wpose = None
    prev_plane_wpose_t = None
    print(f"[INFO] state={state}")

    while True:
        t0 = time.time()

        mp = plane.recv_match(type="LOCAL_POSITION_NED", blocking=False)
        while mp is not None:
            if mp.get_srcSystem() == plane_sys:
                plane_pos = mp
            mp = plane.recv_match(type="LOCAL_POSITION_NED", blocking=False)
        ma = plane.recv_match(type="ATTITUDE", blocking=False)
        while ma is not None:
            if ma.get_srcSystem() == plane_sys:
                plane_yaw = ma.yaw
                plane_yaw_src = "att"
            ma = plane.recv_match(type="ATTITUDE", blocking=False)
        maq = plane.recv_match(type="ATTITUDE_QUATERNION", blocking=False)
        while maq is not None:
            if maq.get_srcSystem() == plane_sys:
                plane_yaw = quat_to_yaw(maq.q1, maq.q2, maq.q3, maq.q4)
                plane_yaw_src = "quat"
            maq = plane.recv_match(type="ATTITUDE_QUATERNION", blocking=False)

        cp = carrier.recv_match(type="LOCAL_POSITION_NED", blocking=False)
        while cp is not None:
            if cp.get_srcSystem() == carrier_sys:
                carrier_pos = cp
                if args.home_from_start and not home_set:
                    args.home_n = float(cp.x)
                    args.home_e = float(cp.y)
                    args.home_d = float(cp.z)
                    home_set = True
                    print(f"[INFO] home set from start: ({args.home_n:.2f}, {args.home_e:.2f}, {args.home_d:.2f})")
            cp = carrier.recv_match(type="LOCAL_POSITION_NED", blocking=False)
        ca = carrier.recv_match(type="ATTITUDE", blocking=False)
        while ca is not None:
            if ca.get_srcSystem() == carrier_sys:
                carrier_att = ca
            ca = carrier.recv_match(type="ATTITUDE", blocking=False)

        # Optional: use Gazebo world pose as unified reference frame for relative tracking.
        if args.use_gz_pose and (time.time() - last_gz_pose_t) >= gz_period:
            p = get_model_pose(args.plane_model_name, args.world_name)
            c = get_model_pose(args.carrier_model_name, args.world_name)
            if p is not None and c is not None:
                now_gz_sample = time.time()
                accept = True
                if prev_gz_sample_t is not None and prev_plane_wpose_raw is not None and prev_carrier_wpose_raw is not None:
                    dt_s = max(1e-3, now_gz_sample - prev_gz_sample_t)
                    vp = math.sqrt(
                        (p[0] - prev_plane_wpose_raw[0]) ** 2
                        + (p[1] - prev_plane_wpose_raw[1]) ** 2
                        + (p[2] - prev_plane_wpose_raw[2]) ** 2
                    ) / dt_s
                    vc = math.sqrt(
                        (c[0] - prev_carrier_wpose_raw[0]) ** 2
                        + (c[1] - prev_carrier_wpose_raw[1]) ** 2
                        + (c[2] - prev_carrier_wpose_raw[2]) ** 2
                    ) / dt_s
                    if vp > args.gz_max_model_speed or vc > args.gz_max_model_speed:
                        accept = False
                        if args.debug and (time.time() - last_wait_log) >= 0.5:
                            print(f"[DBG] reject gz jump: vp={vp:.1f} vc={vc:.1f} m/s")
                            last_wait_log = time.time()

                if accept:
                    a = clamp(args.gz_pose_alpha, 0.0, 1.0)
                    if plane_wpose is None:
                        plane_wpose = p
                    else:
                        plane_wpose = (
                            lpf(plane_wpose[0], p[0], a),
                            lpf(plane_wpose[1], p[1], a),
                            lpf(plane_wpose[2], p[2], a),
                            p[3], p[4], p[5],
                        )
                    if carrier_wpose is None:
                        carrier_wpose = c
                    else:
                        carrier_wpose = (
                            lpf(carrier_wpose[0], c[0], a),
                            lpf(carrier_wpose[1], c[1], a),
                            lpf(carrier_wpose[2], c[2], a),
                            c[3], c[4], c[5],
                        )
                    prev_plane_wpose_raw = p
                    prev_carrier_wpose_raw = c
                    prev_gz_sample_t = now_gz_sample
            last_gz_pose_t = time.time()

        missing = []
        if carrier_pos is None:
            missing.append("carrier_pos")
        if args.use_gz_pose:
            if plane_wpose is None:
                missing.append("plane_wpose")
            if carrier_wpose is None:
                missing.append("carrier_wpose")
        else:
            if plane_pos is None:
                missing.append("plane_pos")

        if missing:
            if args.debug and (time.time() - last_wait_log) >= 1.0:
                print(f"[DBG] waiting telemetry: missing={','.join(missing)}")
                last_wait_log = time.time()
                # Re-request streams while waiting for missing telemetry.
                request_core_streams(plane, plane_sys, plane_comp, args.stream_rate)
                request_core_streams(carrier, carrier_sys, carrier_comp, args.stream_rate)
            send_pos_sp(carrier, carrier_sys, carrier_comp, tn, te, td, tyaw, tvn, tve, tvd)
            if time.time() - last_offboard_refresh >= max(0.2, args.offboard_refresh_sec):
                set_mode_offboard(carrier, carrier_sys, carrier_comp)
                last_offboard_refresh = time.time()
            time.sleep(dt)
            continue

        cn, ce, cd = carrier_pos.x, carrier_pos.y, carrier_pos.z
        if args.use_gz_pose:
            # Map Gazebo ENU world-frame relative offset into carrier local NED setpoint frame:
            # ENU: x=east, y=north, z=up
            # NED: n=north, e=east, d=down
            pwx, pwy, pwz, _, _, pwyaw = plane_wpose
            cwx, cwy, cwz, _, _, _ = carrier_wpose
            dn_w = pwy - cwy
            de_w = pwx - cwx
            dd_w = -(pwz - cwz)  # world z-up -> NED down
            pn, pe, pd = cn + dn_w, ce + de_w, cd + dd_w
            pyaw = wrap_pi((math.pi * 0.5) - pwyaw)  # ENU yaw -> NED yaw
            plane_yaw_src = "gz"
            now_gz = time.time()
            if prev_plane_wpose is not None and prev_plane_wpose_t is not None:
                dt_gz = max(1e-3, now_gz - prev_plane_wpose_t)
                # Gazebo ENU velocity -> NED velocity:
                # vn (north) comes from ENU y, ve (east) comes from ENU x.
                pvn = (pwy - prev_plane_wpose[1]) / dt_gz
                pve = (pwx - prev_plane_wpose[0]) / dt_gz
                pvd = -((pwz - prev_plane_wpose[2]) / dt_gz)
            else:
                pvn, pve, pvd = 0.0, 0.0, 0.0
            prev_plane_wpose = (pwx, pwy, pwz)
            prev_plane_wpose_t = now_gz
        else:
            pn, pe, pd = plane_pos.x, plane_pos.y, plane_pos.z
            pvn, pve, pvd = plane_pos.vx, plane_pos.vy, plane_pos.vz
            pyaw = yaw_from_velocity(pvn, pve, plane_yaw)
            if plane_yaw_src not in ("att", "quat"):
                plane_yaw_src = "vel"

        # Estimate planar turn rate from velocity vector change.
        omega = 0.0
        now_t = time.time()
        if prev_plane_v is not None and prev_plane_v_t is not None:
            dtt = now_t - prev_plane_v_t
            if dtt > 1e-3:
                vpx, vpy = prev_plane_v
                v2 = pvn * pvn + pve * pve
                if v2 > 0.5:
                    # z-component of v x dv divided by |v|^2
                    dvx = pvn - vpx
                    dvy = pve - vpy
                    omega = (pvn * dvy - pve * dvx) / max(v2 * dtt, 1e-6)
        prev_plane_v = (pvn, pve)
        prev_plane_v_t = now_t

        # Predict future plane position (intercept target).
        if args.intercept_time > 0.0:
            t_pred = args.intercept_time
        else:
            t_pred = args.lead_time
        if args.intercept_adaptive:
            dist_xy = math.hypot(pn - cn, pe - ce)
            t_pred = clamp(dist_xy * args.intercept_dist_gain, args.intercept_time_min, args.intercept_time_max)
        pn_l, pe_l = predict_planar_constant_turn(pn, pe, pvn, pve, omega, t_pred)
        pd_l = pd + pvd * t_pred

        # Hard-capture condition in current relative frame (not predicted):
        # XY within threshold and carrier slightly below plane in NED down.
        rel_xy_now = math.hypot(pn - cn, pe - ce)
        rel_down_now = cd - pd
        hard_cap_cond = (
            rel_xy_now <= args.hard_capture_xy
            and args.hard_capture_down_min <= rel_down_now <= args.hard_capture_down_max
        )
        if args.hard_capture and state in ("TRACK_XY", "TRACK_Z", "LAND_ALIGN"):
            if hard_cap_cond:
                if hard_cap_ok_t0 is None:
                    hard_cap_ok_t0 = time.time()
                elif time.time() - hard_cap_ok_t0 >= args.hard_capture_hold_sec:
                    state = "CAPTURE"
                    state_enter_t = time.time()
                    hard_cap_ok_t0 = None
                    print(
                        f"[INFO] hard-capture triggered: xy={rel_xy_now:.2f}m down={rel_down_now:.2f}m -> state=CAPTURE"
                    )
            else:
                hard_cap_ok_t0 = None

        if state == "TRACK_XY":
            dn, de, _ = body_offset_to_ned(pyaw, args.track_forward, args.track_right, args.track_down)
            tn, te = pn_l + dn, pe_l + de
            # Keep carrier safely below plane while XY is converging.
            td = pd_l + max(args.track_down, args.safe_down_sep)
            td = clamp(td, args.track_d_min, args.track_d_max)
            ex, ey = tn - cn, te - ce
            err_xy = math.hypot(ex, ey)

            if err_xy <= args.track_xy_thr:
                if xy_ok_t0 is None:
                    xy_ok_t0 = time.time()
                elif time.time() - xy_ok_t0 >= args.xy_first_hold_sec:
                    state = "TRACK_Z"
                    state_enter_t = time.time()
                    xy_ok_t0 = None
                    print(f"[INFO] state={state}")
            else:
                xy_ok_t0 = None

        elif state == "TRACK_Z":
            dn, de, dd = body_offset_to_ned(pyaw, args.track_forward, args.track_right, args.track_down)
            tn, te, td = pn_l + dn, pe_l + de, pd_l + dd
            td = clamp(td, args.track_d_min, args.track_d_max)
            ex, ey, ez = tn - cn, te - ce, td - cd
            err_xy = math.hypot(ex, ey)
            err_z = abs(ez)
            if err_xy <= args.track_xy_thr and err_z <= args.track_z_thr:
                if track_ok_t0 is None:
                    track_ok_t0 = time.time()
                elif time.time() - track_ok_t0 >= args.track_hold_sec:
                    state = "LAND_ALIGN"
                    state_enter_t = time.time()
                    track_ok_t0 = None
                    print(f"[INFO] state={state}")
            else:
                track_ok_t0 = None

        elif state == "LAND_ALIGN":
            dn, de, dd = body_offset_to_ned(pyaw, args.land_forward, args.land_right, args.land_down)
            tn, te, td = pn_l + dn, pe_l + de, pd_l + dd
            td = clamp(td, args.land_d_min, args.land_d_max)
            ex, ey, ez = tn - cn, te - ce, td - cd
            err_xy = math.hypot(ex, ey)
            err_z = abs(ez)
            if err_xy <= args.land_xy_thr and err_z <= args.land_z_thr:
                if land_ok_t0 is None:
                    land_ok_t0 = time.time()
                elif time.time() - land_ok_t0 >= args.land_hold_sec:
                    state = "CAPTURE"
                    state_enter_t = time.time()
                    land_ok_t0 = None
                    print(f"[INFO] state={state}")
            else:
                land_ok_t0 = None

        elif state == "CAPTURE":
            if not capture_initialized:
                if args.plane_loiter_on_capture:
                    set_mode_loiter(plane, plane_sys, plane_comp)
                    time.sleep(0.05)
                if args.kill_plane_px4_on_capture:
                    run_cmd(["pkill", "-f", "bin/px4 -i 2"])
                    time.sleep(0.1)
                capture_initialized = True
            # hold carrier near current plane and slave plane to carrier
            tn, te, td = cn, ce, cd
            slave_plane_to_carrier(
                args.world_name,
                args.carrier_model,
                args.plane_model,
                args.slave_offset_forward,
                args.slave_offset_right,
                args.slave_offset_up,
            )
            # switch to return after short stabilization
            if time.time() - state_enter_t >= 1.0:
                state = "RETURN_HOME"
                state_enter_t = time.time()
                print(f"[INFO] state={state}")

        elif state == "RETURN_HOME":
            tn, te, td = args.home_n, args.home_e, args.home_d
            slave_plane_to_carrier(
                args.world_name,
                args.carrier_model,
                args.plane_model,
                args.slave_offset_forward,
                args.slave_offset_right,
                args.slave_offset_up,
            )
            err_home = math.sqrt((tn - cn) ** 2 + (te - ce) ** 2 + (td - cd) ** 2)
            if err_home <= args.home_thr:
                state = "DONE"
                print("[INFO] state=DONE")

        else:
            tn, te, td = cn, ce, cd

        err_xy_sp = math.hypot(tn - cn, te - ce)
        en = tn - cn
        ee = te - ce
        ed = td - cd
        # Position + velocity feed-forward interception.
        tvn = args.ff_plane_vel * pvn + args.ff_kp_pos * en
        tve = args.ff_plane_vel * pve + args.ff_kp_pos * ee
        close_req = 0.0
        in_track_states = state in ("TRACK_XY", "TRACK_Z", "LAND_ALIGN")

        # Force a simple converging chase law to avoid orbit/hold-distance behavior:
        # v_cmd = v_plane + Kp * position_error.
        pure_mode = args.pure_pursuit and in_track_states
        if pure_mode:
            tvn = pvn + args.pure_kp_xy * en
            tve = pve + args.pure_kp_xy * ee
            tvd = pvd + args.pure_kp_z * ed
            tvn, tve = clamp_norm2(tvn, tve, args.ff_max_xy)
            tvd = clamp(tvd, -args.ff_max_z, args.ff_max_z)
        else:
            tvd = clamp(args.ff_plane_vel * pvd + args.ff_kp_pos * ed, -args.ff_max_z, args.ff_max_z)

        # Enforce radial closure to break long-lived orbiting at fixed relative distance.
        if (not pure_mode) and args.radial_converge:
            rng = math.hypot(en, ee)
            if rng > 1e-3:
                exn = en / rng
                een = ee / rng
                plane_rad = pvn * exn + pve * een
                cmd_rad = tvn * exn + tve * een
                close_req = clamp(args.radial_k * max(0.0, rng - args.radial_hold_dist), 0.0, args.radial_max)
                need_cmd_rad = plane_rad + close_req
                if cmd_rad < need_cmd_rad:
                    delta = need_cmd_rad - cmd_rad
                    tvn += delta * exn
                    tve += delta * een
        tvn, tve = clamp_norm2(tvn, tve, args.ff_max_xy)

        # Near-field "aerial-refuel style" mode:
        # match plane velocity while removing remaining relative offset.
        vel_match_mode = (
            (not pure_mode)
            and args.track_vel_mode
            and args.dock_vel_match
            and state in ("TRACK_XY", "TRACK_Z", "LAND_ALIGN")
            and err_xy_sp <= args.dock_match_xy
        )
        if vel_match_mode:
            tvn = pvn + args.dock_kp_xy * en
            tve = pve + args.dock_kp_xy * ee
            tvd = pvd + args.dock_kp_z * ed
            tvn, tve = clamp_norm2(tvn, tve, args.dock_max_xy)
            tvd = clamp(tvd, -args.dock_max_z, args.dock_max_z)

        # Far-range direct chase: always push toward current LOS to break orbiting.
        direct_chase_mode = (
            (not pure_mode)
            and args.track_vel_mode
            and args.direct_chase
            and state in ("TRACK_XY", "TRACK_Z", "LAND_ALIGN")
            and err_xy_sp > args.direct_chase_slow_radius
        )
        if direct_chase_mode:
            rng = max(1e-3, math.hypot(en, ee))
            exn = en / rng
            een = ee / rng
            base_vn = args.direct_chase_speed * exn
            base_ve = args.direct_chase_speed * een
            tvn = base_vn + args.direct_chase_ff * pvn
            tve = base_ve + args.direct_chase_ff * pve
            tvn, tve = clamp_norm2(tvn, tve, args.ff_max_xy)
            tvd = clamp(args.ff_kp_pos * ed, -args.ff_max_z, args.ff_max_z)

        if not args.follow_yaw:
            tyaw = carrier_att.yaw if carrier_att is not None else tyaw
        else:
            if args.yaw_mode == "plane":
                tyaw = wrap_pi(pyaw)
            elif args.yaw_mode == "intercept":
                tyaw = wrap_pi(math.atan2(te - ce, tn - cn))
            else:
                # auto: far away face intercept direction, near target align to plane yaw
                if err_xy_sp > args.yaw_auto_switch_dist:
                    tyaw = wrap_pi(math.atan2(te - ce, tn - cn))
                else:
                    tyaw = wrap_pi(pyaw)
        if args.track_vel_mode and in_track_states:
            # Keep velocity control through tracking/alignment.
            # Near-field should transition to vel_match, not position braking.
            if pure_mode:
                use_vel_sp = True
            elif args.dock_vel_match:
                use_vel_sp = True
            else:
                use_vel_sp = err_xy_sp > args.track_vel_switch_xy
        else:
            use_vel_sp = False
        if use_vel_sp:
            send_vel_sp(carrier, carrier_sys, carrier_comp, tvn, tve, tvd, tyaw)
        else:
            send_pos_sp(carrier, carrier_sys, carrier_comp, tn, te, td, tyaw, tvn, tve, tvd)
        if time.time() - last_offboard_refresh >= max(0.2, args.offboard_refresh_sec):
            set_mode_offboard(carrier, carrier_sys, carrier_comp)
            last_offboard_refresh = time.time()

        if args.debug and (time.time() - last_dbg) >= 0.2:
            print(
                    f"[DBG] state={state} plane=({pn:.1f},{pe:.1f},{pd:.1f}) "
                    f"carrier=({cn:.1f},{ce:.1f},{cd:.1f}) sp=({tn:.1f},{te:.1f},{td:.1f}) "
                    f"v_plane=({pvn:.1f},{pve:.1f},{pvd:.1f}) v_sp=({tvn:.1f},{tve:.1f},{tvd:.1f}) "
                    f"pred_t={t_pred:.2f} w={omega:.3f} close_req={close_req:.2f} "
                    f"sp_mode={'pure' if pure_mode else ('direct_chase' if direct_chase_mode else ('vel_match' if vel_match_mode else ('vel' if use_vel_sp else 'pos')))} "
                    f"rel_xy={rel_xy_now:.2f} rel_down={rel_down_now:.2f} "
                    f"pyaw_src={plane_yaw_src} yaw_sp={tyaw:.2f}"
                )
            last_dbg = time.time()

        if state == "DONE":
            time.sleep(1.0)
            break

        sleep_t = dt - (time.time() - t0)
        if sleep_t > 0:
            time.sleep(sleep_t)


if __name__ == "__main__":
    main()
