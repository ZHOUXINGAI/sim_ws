#!/usr/bin/env python3
import argparse
from collections import deque
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


def clamp_norm2(vx, vy, vmax):
    n = math.hypot(vx, vy)
    if n <= max(1e-6, vmax):
        return vx, vy
    s = vmax / n
    return vx * s, vy * s


def slew_limit_1d(cur, prev, max_delta):
    d = cur - prev
    if d > max_delta:
        return prev + max_delta
    if d < -max_delta:
        return prev - max_delta
    return cur


def slew_limit_xy(vn, ve, prev_n, prev_e, max_dxy):
    dn = vn - prev_n
    de = ve - prev_e
    nrm = math.hypot(dn, de)
    if nrm <= max_dxy or nrm < 1e-6:
        return vn, ve
    s = max_dxy / nrm
    return prev_n + dn * s, prev_e + de * s


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


def radial_close_rate(ex, ey, pvn, pve, vn_cmd, ve_cmd, min_rate):
    err_xy = math.hypot(ex, ey)
    if err_xy <= 1e-3:
        return min_rate
    rel_vn_cmd = pvn - vn_cmd
    rel_ve_cmd = pve - ve_cmd
    return -((ex * rel_vn_cmd + ey * rel_ve_cmd) / err_xy)


def required_close_rate(err_xy, near_dist, far_dist, near_rate, far_rate):
    if err_xy <= near_dist:
        return near_rate
    if err_xy >= far_dist:
        return far_rate
    span = max(1e-6, far_dist - near_dist)
    t = (err_xy - near_dist) / span
    return near_rate + t * (far_rate - near_rate)


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


def lpf(old_v, new_v, alpha):
    return old_v * (1.0 - alpha) + new_v * alpha


def predict_turn_acc_seq(vn0, ve0, omega, dt, n_h):
    # Constant-turn velocity/acceleration prediction in N/E plane.
    v = np.array([vn0, ve0], dtype=float)
    aseq = np.zeros((n_h, 3), dtype=float)
    c = math.cos(omega * dt)
    s = math.sin(omega * dt)
    r = np.array([[c, -s], [s, c]], dtype=float)
    for k in range(n_h):
        v_next = r @ v
        a_xy = (v_next - v) / dt
        aseq[k, 0] = a_xy[0]
        aseq[k, 1] = a_xy[1]
        aseq[k, 2] = 0.0
        v = v_next
    return aseq


def predict_turn_vel_seq(vn0, ve0, vd0, omega, dt, n_h):
    # Constant-turn velocity prediction in N/E plane.
    v = np.array([vn0, ve0], dtype=float)
    vseq = np.zeros((n_h, 3), dtype=float)
    c = math.cos(omega * dt)
    s = math.sin(omega * dt)
    r = np.array([[c, -s], [s, c]], dtype=float)
    for k in range(n_h):
        v = r @ v
        vseq[k, 0] = v[0]
        vseq[k, 1] = v[1]
        vseq[k, 2] = vd0
    return vseq


def estimate_turn_circle(pn, pe, vn, ve, omega):
    v_xy = math.hypot(vn, ve)
    if v_xy < 1e-6 or abs(omega) < 1e-6:
        return None
    r = v_xy / abs(omega)
    # Left normal of velocity in N/E frame.
    n_left = -ve / v_xy
    e_left = vn / v_xy
    sgn = 1.0 if omega >= 0.0 else -1.0
    cn = pn + sgn * r * n_left
    ce = pe + sgn * r * e_left
    return cn, ce, r


def predict_circle_vel_seq(pn, pe, vd0, cn, ce, r, omega, dt, n_h):
    # Fixed-circle prediction from filtered center/radius and current phase.
    th0 = math.atan2(pe - ce, pn - cn)
    vseq = np.zeros((n_h, 3), dtype=float)
    for k in range(n_h):
        th = th0 + omega * dt * (k + 1)
        vseq[k, 0] = -omega * r * math.sin(th)
        vseq[k, 1] = omega * r * math.cos(th)
        vseq[k, 2] = vd0
    return vseq


def predict_direct_loiter_vel_seq(pn, pe, pvn, pve, pvd, omega, dt, n_h):
    v_xy = math.hypot(pvn, pve)
    if v_xy < 1e-3 or abs(omega) < 1e-3:
        return np.tile(np.array([pvn, pve, pvd], dtype=float), (n_h, 1))
    r = v_xy / abs(omega)
    n_left = -pve / v_xy
    e_left = pvn / v_xy
    sgn = 1.0 if omega >= 0.0 else -1.0
    cn = pn + sgn * r * n_left
    ce = pe + sgn * r * e_left
    th0 = math.atan2(pe - ce, pn - cn)
    vseq = np.zeros((n_h, 3), dtype=float)
    for k in range(n_h):
        th = th0 + omega * dt * (k + 1)
        vseq[k, 0] = -omega * r * math.sin(th)
        vseq[k, 1] = omega * r * math.cos(th)
        vseq[k, 2] = pvd
    return vseq


def fit_circle_xy(points):
    # Kasa least-squares circle fit: x^2 + y^2 + a*x + b*y + c = 0
    if len(points) < 3:
        return None
    x = np.array([p[0] for p in points], dtype=float)
    y = np.array([p[1] for p in points], dtype=float)
    a = np.column_stack((x, y, np.ones_like(x)))
    b = -(x * x + y * y)
    try:
        coef, _, _, _ = np.linalg.lstsq(a, b, rcond=None)
    except np.linalg.LinAlgError:
        return None
    aa, bb, cc = coef
    cx = -0.5 * aa
    cy = -0.5 * bb
    r2 = cx * cx + cy * cy - cc
    if r2 <= 1e-6:
        return None
    r = math.sqrt(r2)
    d = np.hypot(x - cx, y - cy)
    rms = float(np.sqrt(np.mean((d - r) ** 2)))
    return cx, cy, r, rms


def build_prediction_mats(a, b, g, n_h):
    nx = a.shape[0]
    nu = b.shape[1]
    sx = np.zeros((nx * n_h, nx))
    su = np.zeros((nx * n_h, nu * n_h))
    sg = np.zeros((nx * n_h, g.shape[1] * n_h))
    ap = np.eye(nx)
    for i in range(n_h):
        ap = a @ ap
        sx[i * nx:(i + 1) * nx, :] = ap
        for j in range(i + 1):
            aij = np.linalg.matrix_power(a, i - j)
            su[i * nx:(i + 1) * nx, j * nu:(j + 1) * nu] = aij @ b
            sg[i * nx:(i + 1) * nx, j * nu:(j + 1) * nu] = aij @ g
    return sx, su, sg


def solve_box_qp_pg(h, f, lo, hi, iters=60):
    # Projected gradient for convex box-QP: 0.5*x'Hx + f'x
    x = np.clip(-f / (np.diag(h) + 1e-6), lo, hi)
    lips = max(1e-6, np.linalg.norm(h, 2))
    step = 1.0 / lips
    for _ in range(iters):
        grad = h @ x + f
        x = np.clip(x - step * grad, lo, hi)
    return x


def main():
    ap = argparse.ArgumentParser(description="True MPC: keep carrier under plane (relative pose + rel-velocity convergence).")
    ap.add_argument("--plane-url", default="udp:127.0.0.1:14542")
    ap.add_argument("--carrier-url", default="udp:127.0.0.1:14541")
    ap.add_argument("--plane-sysid", type=int, default=3)
    ap.add_argument("--carrier-sysid", type=int, default=2)
    ap.add_argument("--world-name", default="default")
    ap.add_argument("--plane-model", default="plane_2")
    ap.add_argument("--carrier-model", default="iris_aircarrier_1")
    ap.add_argument("--use-gz-pose", dest="use_gz_pose", action="store_true")
    ap.add_argument("--no-use-gz-pose", dest="use_gz_pose", action="store_false")
    ap.set_defaults(use_gz_pose=True)
    ap.add_argument("--prefer-mavlink", dest="prefer_mavlink", action="store_true", help="prefer LOCAL_POSITION_NED relative pose over gz world pose")
    ap.add_argument("--no-prefer-mavlink", dest="prefer_mavlink", action="store_false", help="keep gz world pose as primary relative pose")
    ap.set_defaults(prefer_mavlink=False)
    ap.add_argument("--mavlink-fallback-pose", action="store_true", default=False, help="allow LOCAL_POSITION_NED pose fallback when gz pose is unavailable")
    ap.add_argument("--rate", type=float, default=20.0)
    ap.add_argument("--stream-rate", type=float, default=30.0)
    ap.add_argument("--gz-pose-rate", type=float, default=20.0)
    ap.add_argument("--target-rel-n", type=float, default=0.0)
    ap.add_argument("--target-rel-e", type=float, default=0.0)
    ap.add_argument("--target-rel-d", type=float, default=-1.0)
    ap.add_argument("--controller", choices=["tracker_ff", "mpc"], default="tracker_ff")
    ap.add_argument("--track-lead-sec", type=float, default=0.9, help="lead-time for plane state prediction (s)")
    ap.add_argument("--track-kp-xy", type=float, default=0.85, help="position gain in XY for tracker_ff")
    ap.add_argument("--track-kd-xy", type=float, default=0.55, help="relative-velocity gain in XY for tracker_ff")
    ap.add_argument("--track-ki-xy", type=float, default=0.03, help="integral gain in XY for tracker_ff")
    ap.add_argument("--track-kp-z", type=float, default=0.95, help="position gain in Z for tracker_ff")
    ap.add_argument("--track-kd-z", type=float, default=0.55, help="relative-velocity gain in Z for tracker_ff")
    ap.add_argument("--track-ki-z", type=float, default=0.05, help="integral gain in Z for tracker_ff")
    ap.add_argument("--track-phase-gain", type=float, default=1.0, help="gain on estimated phase-lag time for adaptive lead")
    ap.add_argument("--track-phase-alpha", type=float, default=0.22, help="LPF alpha on adaptive phase lead")
    ap.add_argument("--track-phase-lead-max", type=float, default=2.5, help="max additional adaptive lead (s)")
    ap.add_argument("--track-phase-k-rad", type=float, default=1.0, help="radial speed gain for phase circle lock")
    ap.add_argument("--track-phase-k-tan", type=float, default=0.55, help="tangential phase correction gain for phase circle lock")
    ap.add_argument("--track-i-zone-xy", type=float, default=20.0, help="integrator active zone for XY error (m)")
    ap.add_argument("--track-i-zone-z", type=float, default=8.0, help="integrator active zone for Z error (m)")
    ap.add_argument("--track-i-limit-xy", type=float, default=35.0, help="integrator clamp for XY (m*s)")
    ap.add_argument("--track-i-limit-z", type=float, default=20.0, help="integrator clamp for Z (m*s)")
    ap.add_argument("--track-close-boost", type=float, default=0.10, help="extra closing boost gain in XY when far (m/s per m)")
    ap.add_argument("--track-close-boost-start", type=float, default=12.0, help="error threshold to enable closing boost (m)")
    ap.add_argument("--track-close-boost-max", type=float, default=4.0, help="max extra closing boost in XY (m/s)")
    ap.add_argument("--horizon", type=int, default=16)
    ap.add_argument("--q-pos-n", type=float, default=5.0)
    ap.add_argument("--q-pos-e", type=float, default=5.0)
    ap.add_argument("--q-pos-d", type=float, default=8.0)
    ap.add_argument("--q-vel-n", type=float, default=1.2)
    ap.add_argument("--q-vel-e", type=float, default=1.2)
    ap.add_argument("--q-vel-d", type=float, default=2.0)
    ap.add_argument("--r-acc-n", type=float, default=0.10)
    ap.add_argument("--r-acc-e", type=float, default=0.10)
    ap.add_argument("--r-acc-d", type=float, default=0.18)
    ap.add_argument("--qf-scale", type=float, default=7.0)
    ap.add_argument("--acc-max-xy", type=float, default=9.0)
    ap.add_argument("--acc-max-z", type=float, default=5.0)
    ap.add_argument("--max-xy-speed", type=float, default=18.0)
    ap.add_argument("--max-z-speed", type=float, default=3.0)
    ap.add_argument("--plane-vel-alpha", type=float, default=0.45)
    ap.add_argument("--plane-acc-alpha", type=float, default=0.35)
    ap.add_argument("--plane-acc-max", type=float, default=6.0, help="clip estimated plane accel (m/s^2)")
    ap.add_argument("--loiter-predict", dest="loiter_predict", action="store_true", help="enable turn-rate based prediction blend")
    ap.add_argument("--no-loiter-predict", dest="loiter_predict", action="store_false", help="disable turn-rate prediction blend")
    ap.set_defaults(loiter_predict=True)
    ap.add_argument("--omega-alpha", type=float, default=0.35, help="LPF on estimated turn-rate omega")
    ap.add_argument("--omega-low", type=float, default=0.04, help="lower omega bound for turn-model blending (rad/s)")
    ap.add_argument("--omega-high", type=float, default=0.20, help="upper omega bound for turn-model blending (rad/s)")
    ap.add_argument("--speed-low", type=float, default=4.0, help="lower speed bound for turn-model blending (m/s)")
    ap.add_argument("--speed-high", type=float, default=9.0, help="upper speed bound for turn-model blending (m/s)")
    ap.add_argument("--direct-loiter-predict", action="store_true", default=True, help="predict loiter directly from plane position+velocity")
    ap.add_argument("--direct-min-omega", type=float, default=0.08, help="min |omega| for direct loiter predictor (rad/s)")
    ap.add_argument("--direct-min-speed", type=float, default=4.0, help="min speed for direct loiter predictor (m/s)")
    ap.add_argument("--circle-predict", dest="circle_predict", action="store_true", help="enable fixed-circle loiter prediction")
    ap.add_argument("--no-circle-predict", dest="circle_predict", action="store_false", help="disable fixed-circle loiter prediction")
    ap.set_defaults(circle_predict=True)
    ap.add_argument("--circle-center-alpha", type=float, default=0.18, help="LPF alpha for estimated circle center")
    ap.add_argument("--circle-radius-alpha", type=float, default=0.20, help="LPF alpha for estimated circle radius")
    ap.add_argument("--circle-fit-window", type=float, default=14.0, help="history window for circle fit (s)")
    ap.add_argument("--circle-fit-min-points", type=int, default=35, help="minimum points for circle fit")
    ap.add_argument("--circle-fit-max-rms", type=float, default=2.2, help="max RMS fitting residual for full circle weight (m)")
    ap.add_argument("--circle-omega-alpha", type=float, default=0.20, help="LPF alpha for circle angular-rate estimate")
    ap.add_argument("--circle-min-radius", type=float, default=12.0, help="min valid loiter radius (m)")
    ap.add_argument("--circle-max-radius", type=float, default=260.0, help="max valid loiter radius (m)")
    ap.add_argument("--circle-min-omega", type=float, default=0.12, help="min omega for circle fitting (rad/s)")
    ap.add_argument("--circle-min-speed", type=float, default=5.0, help="min speed for circle fitting (m/s)")
    ap.add_argument("--circle-max-rel-err", type=float, default=0.22, help="max normalized radius residual for full circle weight")
    ap.add_argument("--circle-lock-min-w", type=float, default=0.65, help="weight threshold to consider predictor locked on circle")
    ap.add_argument("--offboard-refresh-sec", type=float, default=0.4)
    ap.add_argument("--guard-enable", action="store_true", default=True, help="fallback to guaranteed-closing pursuit when MPC command is not converging")
    ap.add_argument("--guard-kp-xy", type=float, default=0.30, help="pursuit gain for XY in guard mode")
    ap.add_argument("--guard-kp-z", type=float, default=0.50, help="pursuit gain for Z in guard mode")
    ap.add_argument("--guard-min-close-rate", type=float, default=0.5, help="required radial closing speed (m/s); otherwise guard overrides MPC")
    ap.add_argument("--guard-min-close-rate-near", type=float, default=0.3, help="required radial closing speed when near target (m/s)")
    ap.add_argument("--guard-min-close-rate-far", type=float, default=2.0, help="required radial closing speed when far from target (m/s)")
    ap.add_argument("--guard-close-near-dist", type=float, default=8.0, help="distance threshold for near close-rate requirement (m)")
    ap.add_argument("--guard-close-far-dist", type=float, default=40.0, help="distance threshold for far close-rate requirement (m)")
    ap.add_argument("--cmd-acc-max-xy", type=float, default=3.5, help="limit command velocity slew in XY as accel (m/s^2)")
    ap.add_argument("--cmd-acc-max-z", type=float, default=2.0, help="limit command velocity slew in Z as accel (m/s^2)")
    ap.add_argument("--arm-carrier", action="store_true", default=True)
    ap.add_argument("--wait-arm", type=float, default=12.0)
    ap.add_argument("--relax-arm-checks", action="store_true", default=True)
    ap.add_argument("--debug", action="store_true")
    args = ap.parse_args()

    dt = 1.0 / max(1.0, args.rate)
    gz_dt = 1.0 / max(1.0, args.gz_pose_rate)

    sx = su = sg = qbar = wmbar = h = u_lo = u_hi = None
    if args.controller == "mpc":
        # Velocity-MPC:
        # e_{k+1} = e_k + dt * (v_p_k - v_c_k)
        a = np.eye(3)
        b = -dt * np.eye(3)
        g = dt * np.eye(3)
        sx, su, sg = build_prediction_mats(a, b, g, args.horizon)

        qx = np.diag([args.q_pos_n, args.q_pos_e, args.q_pos_d])
        qn = qx * args.qf_scale
        ru = np.diag([args.r_acc_n, args.r_acc_e, args.r_acc_d])
        wm = np.diag([args.q_vel_n, args.q_vel_e, args.q_vel_d])
        qbar = np.kron(np.eye(args.horizon), qx)
        qbar[-3:, -3:] = qn
        rbar = np.kron(np.eye(args.horizon), ru)
        wmbar = np.kron(np.eye(args.horizon), wm)

        h = su.T @ qbar @ su + rbar + wmbar
        h = 0.5 * (h + h.T) + 1e-9 * np.eye(h.shape[0])

        u_lo = np.zeros(3 * args.horizon)
        u_hi = np.zeros(3 * args.horizon)
        for i in range(args.horizon):
            k = 3 * i
            u_lo[k + 0] = -args.max_xy_speed
            u_hi[k + 0] = args.max_xy_speed
            u_lo[k + 1] = -args.max_xy_speed
            u_hi[k + 1] = args.max_xy_speed
            u_lo[k + 2] = -args.max_z_speed
            u_hi[k + 2] = args.max_z_speed

    plane = mavutil.mavlink_connection(args.plane_url, autoreconnect=True)
    carrier = mavutil.mavlink_connection(args.carrier_url, autoreconnect=True)
    phb = plane.wait_heartbeat(timeout=10)
    chb = carrier.wait_heartbeat(timeout=10)
    if phb is None or chb is None:
        raise RuntimeError("heartbeat timeout")
    hb_plane_sys = phb.get_srcSystem()
    hb_carrier_sys = chb.get_srcSystem()
    plane_sys = int(args.plane_sysid)
    carrier_sys = int(args.carrier_sysid)

    # Safety: auto-detect swapped UDP endpoints and fix in-place.
    if hb_plane_sys == carrier_sys and hb_carrier_sys == plane_sys:
        plane, carrier = carrier, plane
        phb, chb = chb, phb
        hb_plane_sys, hb_carrier_sys = hb_carrier_sys, hb_plane_sys
        print("[WARN] plane/carrier UDP endpoints were swapped; auto-corrected.")

    # Hard guard: never continue if endpoint mapping is still inconsistent.
    if hb_plane_sys != plane_sys or hb_carrier_sys != carrier_sys:
        raise RuntimeError(
            f"URL<->SYSID mismatch: plane-url sees sysid={hb_plane_sys} (expect {plane_sys}), "
            f"carrier-url sees sysid={hb_carrier_sys} (expect {carrier_sys})."
        )

    plane_comp = phb.get_srcComponent()
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

    last_offboard_refresh = 0.0
    last_dbg = 0.0
    last_yaw = 0.0

    plane_pos = None
    carrier_pos = None
    carrier_att = None
    last_gz_t = 0.0
    pw = None
    cw = None
    prev_pw = None
    prev_cw = None
    prev_t = None
    gz_has_state = False
    gz_plane_n = gz_plane_e = gz_plane_d = 0.0
    gz_rel = np.zeros(3, dtype=float)
    gz_pvn = gz_pve = gz_pvd = 0.0
    gz_cvn = gz_cve = gz_cvd = 0.0
    gz_rel_v = np.zeros(3, dtype=float)

    pvn = pve = pvd = 0.0
    cvn = cve = cvd = 0.0
    apn = ape = apd = 0.0
    prev_pvn = prev_pve = prev_pvd = 0.0
    omega = 0.0
    prev_chi = None
    omega_chi = 0.0
    omega_circle = 0.0
    circle_cn = None
    circle_ce = None
    circle_r = None
    circle_hist = deque()

    rel_target = np.array([args.target_rel_n, args.target_rel_e, args.target_rel_d], dtype=float)
    prev_vn_cmd = 0.0
    prev_ve_cmd = 0.0
    prev_vd_cmd = 0.0
    i_en = 0.0
    i_ee = 0.0
    i_ed = 0.0
    phase_lead = 0.0

    while True:
        t0 = time.time()

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

        rel = None
        rel_v = None
        rel_gz = None
        rel_local = None
        plane_n = None
        plane_e = None
        plane_d = None
        if args.use_gz_pose and (time.time() - last_gz_t >= gz_dt):
            p = get_model_pose(args.plane_model, args.world_name)
            c = get_model_pose(args.carrier_model, args.world_name)
            last_gz_t = time.time()
            if p is not None and c is not None:
                pw, cw = p, c
                plane_n_g = p[1]
                plane_e_g = p[0]
                plane_d_g = -p[2]
                rel_gz_new = np.array([p[1] - c[1], p[0] - c[0], -(p[2] - c[2])], dtype=float)
                if prev_pw is not None and prev_cw is not None and prev_t is not None:
                    dts = max(1e-3, time.time() - prev_t)
                    raw_pvn = (p[1] - prev_pw[1]) / dts
                    raw_pve = (p[0] - prev_pw[0]) / dts
                    raw_pvd = -((p[2] - prev_pw[2]) / dts)
                    raw_cvn = (c[1] - prev_cw[1]) / dts
                    raw_cve = (c[0] - prev_cw[0]) / dts
                    raw_cvd = -((c[2] - prev_cw[2]) / dts)
                    gz_pvn = lpf(gz_pvn, raw_pvn, args.plane_vel_alpha)
                    gz_pve = lpf(gz_pve, raw_pve, args.plane_vel_alpha)
                    gz_pvd = lpf(gz_pvd, raw_pvd, args.plane_vel_alpha)
                    gz_cvn = raw_cvn
                    gz_cve = raw_cve
                    gz_cvd = raw_cvd
                    gz_plane_n = plane_n_g
                    gz_plane_e = plane_e_g
                    gz_plane_d = plane_d_g
                    gz_rel = rel_gz_new
                    gz_rel_v = np.array([gz_pvn - gz_cvn, gz_pve - gz_cve, gz_pvd - gz_cvd], dtype=float)
                    gz_has_state = True
                prev_pw = p
                prev_cw = c
                prev_t = time.time()

        if args.use_gz_pose and gz_has_state:
            plane_n = gz_plane_n
            plane_e = gz_plane_e
            plane_d = gz_plane_d
            rel_gz = gz_rel.copy()
            rel = rel_gz.copy()
            pvn, pve, pvd = gz_pvn, gz_pve, gz_pvd
            cvn, cve, cvd = gz_cvn, gz_cve, gz_cvd
            rel_v = gz_rel_v.copy()

        if plane_pos is not None and carrier_pos is not None:
            rel_local = np.array(
                [plane_pos.x - carrier_pos.x, plane_pos.y - carrier_pos.y, plane_pos.z - carrier_pos.z],
                dtype=float,
            )

            # Only use LOCAL_POSITION_NED as pose if explicitly requested or no gz pose available.
            if args.prefer_mavlink or (args.mavlink_fallback_pose and rel is None):
                plane_n = float(plane_pos.x)
                plane_e = float(plane_pos.y)
                plane_d = float(plane_pos.z)
                rel = rel_local
                pvn = lpf(pvn, float(plane_pos.vx), args.plane_vel_alpha)
                pve = lpf(pve, float(plane_pos.vy), args.plane_vel_alpha)
                pvd = lpf(pvd, float(plane_pos.vz), args.plane_vel_alpha)
                cvn = float(carrier_pos.vx)
                cve = float(carrier_pos.vy)
                cvd = float(carrier_pos.vz)
                rel_v = np.array([pvn - cvn, pve - cve, pvd - cvd], dtype=float)

        if rel is None or rel_v is None:
            send_vel_sp(carrier, carrier_sys, carrier_comp, 0.0, 0.0, 0.0, last_yaw)
            if args.debug and (time.time() - last_dbg) > 1.0:
                print("[DBG] waiting telemetry")
                last_dbg = time.time()
            time.sleep(dt)
            continue

        now_t = time.time()
        if plane_n is not None and plane_e is not None:
            circle_hist.append((now_t, plane_n, plane_e))
            while circle_hist and (now_t - circle_hist[0][0]) > args.circle_fit_window:
                circle_hist.popleft()

        # plane accel estimate
        raw_apn = clamp((pvn - prev_pvn) / dt, -args.plane_acc_max, args.plane_acc_max)
        raw_ape = clamp((pve - prev_pve) / dt, -args.plane_acc_max, args.plane_acc_max)
        raw_apd = clamp((pvd - prev_pvd) / dt, -args.plane_acc_max, args.plane_acc_max)
        prev_pvn, prev_pve, prev_pvd = pvn, pve, pvd
        apn = lpf(apn, raw_apn, args.plane_acc_alpha)
        ape = lpf(ape, raw_ape, args.plane_acc_alpha)
        apd = lpf(apd, raw_apd, args.plane_acc_alpha)
        v2 = max(0.5, pvn * pvn + pve * pve)
        raw_omega = (pvn * ape - pve * apn) / v2
        omega = lpf(omega, raw_omega, args.omega_alpha)
        v_xy = math.hypot(pvn, pve)
        if v_xy > 1e-3:
            chi = math.atan2(pve, pvn)
            if prev_chi is not None:
                raw_omega_chi = wrap_pi(chi - prev_chi) / dt
                omega_chi = lpf(omega_chi, raw_omega_chi, args.omega_alpha)
            prev_chi = chi

        x0 = rel - rel_target  # relative position error [N,E,D]
        w_turn = 0.0
        w_direct = 0.0
        w_circle = 0.0
        pred_model = "leadff"
        circle_rel_err = -1.0
        circle_fit_rms = -1.0
        phase_err = 0.0
        phase_lag_t = 0.0
        lead_dyn = clamp(args.track_lead_sec, 0.0, 3.0)
        u0 = np.zeros(3, dtype=float)

        if args.controller == "tracker_ff":
            a_ff = np.array([apn, ape, apd], dtype=float)
            v_plane = np.array([pvn, pve, pvd], dtype=float)
            carrier_n = (plane_n - rel[0]) if plane_n is not None else None
            carrier_e = (plane_e - rel[1]) if plane_e is not None else None
            carrier_d = (plane_d - rel[2]) if plane_d is not None else None

            # Estimate loiter circle from plane history for adaptive phase-lead compensation.
            if args.circle_predict and plane_n is not None and plane_e is not None:
                v_xy = math.hypot(pvn, pve)
                if len(circle_hist) >= max(3, args.circle_fit_min_points):
                    pts = [(hh[1], hh[2]) for hh in circle_hist]
                    fit = fit_circle_xy(pts)
                    if fit is not None:
                        est_cn, est_ce, est_r, est_rms = fit
                        if args.circle_min_radius <= est_r <= args.circle_max_radius:
                            circle_fit_rms = est_rms
                            if circle_cn is None:
                                circle_cn, circle_ce, circle_r = est_cn, est_ce, est_r
                            else:
                                circle_cn = lpf(circle_cn, est_cn, args.circle_center_alpha)
                                circle_ce = lpf(circle_ce, est_ce, args.circle_center_alpha)
                                circle_r = lpf(circle_r, est_r, args.circle_radius_alpha)
                            om_inst = 0.0
                            if est_r > 1e-3:
                                om_inst = v_xy / est_r
                                cross = (plane_n - est_cn) * pve - (plane_e - est_ce) * pvn
                                if cross < 0.0:
                                    om_inst = -om_inst
                            omega_circle = lpf(omega_circle, om_inst, args.circle_omega_alpha)

            ref_plane_n = None
            ref_plane_e = None
            ref_plane_d = None
            ref_plane_vn = pvn + lead_dyn * apn
            ref_plane_ve = pve + lead_dyn * ape
            ref_plane_vd = pvd + lead_dyn * apd

            # Circular phase-lead: convert phase lag to equivalent time lag and compensate it.
            if (
                circle_cn is not None
                and circle_r is not None
                and abs(omega_circle) >= args.circle_min_omega
                and circle_fit_rms >= 0.0
                and circle_fit_rms <= (args.circle_fit_max_rms * 1.8)
                and carrier_n is not None
                and carrier_e is not None
            ):
                th_p = math.atan2(plane_e - circle_ce, plane_n - circle_cn)
                th_c = math.atan2(carrier_e - circle_ce, carrier_n - circle_cn)
                phase_err = wrap_pi(th_p - th_c)
                lag_t = phase_err / omega_circle
                lag_t = clamp(lag_t, -args.track_phase_lead_max, args.track_phase_lead_max)
                phase_lag_t = lag_t
                lag_t_pos = max(0.0, lag_t)
                phase_lead = lpf(phase_lead, lag_t_pos, args.track_phase_alpha)
                lead_dyn = clamp(
                    args.track_lead_sec + args.track_phase_gain * phase_lead,
                    0.0,
                    args.track_lead_sec + args.track_phase_lead_max,
                )
                th_ref = th_p + omega_circle * lead_dyn
                ref_plane_n = circle_cn + circle_r * math.cos(th_ref)
                ref_plane_e = circle_ce + circle_r * math.sin(th_ref)
                ref_plane_vn = -omega_circle * circle_r * math.sin(th_ref)
                ref_plane_ve = omega_circle * circle_r * math.cos(th_ref)
                if plane_d is not None:
                    ref_plane_d = plane_d + lead_dyn * pvd + 0.5 * (lead_dyn * lead_dyn) * apd
                pred_model = "phase"
            else:
                phase_lead = lpf(phase_lead, 0.0, args.track_phase_alpha * 0.5)

            # Fallback delay-compensated prediction in inertial frame.
            if ref_plane_n is None:
                ref_plane_n = plane_n + lead_dyn * pvn + 0.5 * (lead_dyn * lead_dyn) * apn if plane_n is not None else None
                ref_plane_e = plane_e + lead_dyn * pve + 0.5 * (lead_dyn * lead_dyn) * ape if plane_e is not None else None
                if plane_d is not None:
                    ref_plane_d = plane_d + lead_dyn * pvd + 0.5 * (lead_dyn * lead_dyn) * apd

            if ref_plane_n is not None and carrier_n is not None:
                e_pred_n = (ref_plane_n - carrier_n) - rel_target[0]
                e_pred_e = (ref_plane_e - carrier_e) - rel_target[1]
            else:
                e_pred_n = x0[0] + lead_dyn * pvn + 0.5 * (lead_dyn * lead_dyn) * apn
                e_pred_e = x0[1] + lead_dyn * pve + 0.5 * (lead_dyn * lead_dyn) * ape
            if ref_plane_d is not None and carrier_d is not None:
                e_pred_d = (ref_plane_d - carrier_d) - rel_target[2]
            else:
                e_pred_d = x0[2] + lead_dyn * pvd + 0.5 * (lead_dyn * lead_dyn) * apd

            e_pred = np.array([e_pred_n, e_pred_e, e_pred_d], dtype=float)
            rel_v_pred = np.array([ref_plane_vn - cvn, ref_plane_ve - cve, ref_plane_vd - cvd], dtype=float)

            if abs(x0[0]) <= args.track_i_zone_xy:
                i_en = clamp(i_en + x0[0] * dt, -args.track_i_limit_xy, args.track_i_limit_xy)
            else:
                i_en *= 0.98
            if abs(x0[1]) <= args.track_i_zone_xy:
                i_ee = clamp(i_ee + x0[1] * dt, -args.track_i_limit_xy, args.track_i_limit_xy)
            else:
                i_ee *= 0.98
            if abs(x0[2]) <= args.track_i_zone_z:
                i_ed = clamp(i_ed + x0[2] * dt, -args.track_i_limit_z, args.track_i_limit_z)
            else:
                i_ed *= 0.98

            if pred_model == "phase" and carrier_n is not None and carrier_e is not None and circle_cn is not None and circle_r is not None:
                # Geometry-consistent circle lock:
                # tangential term closes phase, radial term closes radius, preserving circular trajectory.
                dn_c = carrier_n - circle_cn
                de_c = carrier_e - circle_ce
                r_car = max(1e-3, math.hypot(dn_c, de_c))
                r_hat_n = dn_c / r_car
                r_hat_e = de_c / r_car
                sgn_om = 1.0 if omega_circle >= 0.0 else -1.0
                t_hat_n = -r_hat_e * sgn_om
                t_hat_e = r_hat_n * sgn_om
                v_base = abs(omega_circle) * circle_r
                phase_along = clamp(phase_err * sgn_om, -math.pi, math.pi)
                v_t = v_base * (1.0 + args.track_phase_k_tan * phase_along)
                v_t = clamp(v_t, 0.0, args.max_xy_speed * 1.5)
                v_r = args.track_phase_k_rad * (circle_r - r_car)
                vn_cmd = float(t_hat_n * v_t + r_hat_n * v_r + 0.25 * args.track_kp_xy * e_pred[0])
                ve_cmd = float(t_hat_e * v_t + r_hat_e * v_r + 0.25 * args.track_kp_xy * e_pred[1])
                vd_cmd = float(ref_plane_vd + args.track_kp_z * e_pred[2] + args.track_kd_z * rel_v_pred[2] + args.track_ki_z * i_ed)
            else:
                vn_cmd = float(ref_plane_vn + args.track_kp_xy * e_pred[0] + args.track_kd_xy * rel_v_pred[0] + args.track_ki_xy * i_en)
                ve_cmd = float(ref_plane_ve + args.track_kp_xy * e_pred[1] + args.track_kd_xy * rel_v_pred[1] + args.track_ki_xy * i_ee)
                vd_cmd = float(ref_plane_vd + args.track_kp_z * e_pred[2] + args.track_kd_z * rel_v_pred[2] + args.track_ki_z * i_ed)

            err_xy = math.hypot(x0[0], x0[1])
            if args.track_close_boost > 1e-6 and err_xy > args.track_close_boost_start:
                boost = args.track_close_boost * (err_xy - args.track_close_boost_start)
                boost = clamp(boost, 0.0, args.track_close_boost_max)
                inv_err = 1.0 / max(1e-6, err_xy)
                vn_cmd += boost * x0[0] * inv_err
                ve_cmd += boost * x0[1] * inv_err

            u0 = np.array([vn_cmd, ve_cmd, vd_cmd], dtype=float)
            vn_cmd, ve_cmd = clamp_norm2(vn_cmd, ve_cmd, args.max_xy_speed)
            vd_cmd = clamp(vd_cmd, -args.max_z_speed, args.max_z_speed)
            mode = "trkff"
        else:
            # Generic adaptive predictor:
            # blend constant-velocity model with turn-rate model and circle-fit model.
            v_const = np.tile(np.array([pvn, pve, pvd], dtype=float), (args.horizon, 1))
            pred_model = "cv"
            if args.loiter_predict:
                v_turn = predict_turn_vel_seq(pvn, pve, pvd, omega, dt, args.horizon)
                om_abs = abs(omega)
                v_xy = math.hypot(pvn, pve)
                w_om = clamp((om_abs - args.omega_low) / max(1e-6, (args.omega_high - args.omega_low)), 0.0, 1.0)
                w_v = clamp((v_xy - args.speed_low) / max(1e-6, (args.speed_high - args.speed_low)), 0.0, 1.0)
                w_turn = w_om * w_v
                v_mix = (1.0 - w_turn) * v_const + w_turn * v_turn
                if w_turn > 0.25:
                    pred_model = "turn"
            else:
                v_mix = v_const

            if args.direct_loiter_predict and plane_n is not None and plane_e is not None:
                om_abs_chi = abs(omega_chi)
                v_xy = math.hypot(pvn, pve)
                if om_abs_chi >= args.direct_min_omega and v_xy >= args.direct_min_speed:
                    v_direct = predict_direct_loiter_vel_seq(plane_n, plane_e, pvn, pve, pvd, omega_chi, dt, args.horizon)
                    w_direct_om = clamp((om_abs_chi - args.direct_min_omega) / max(1e-6, (args.omega_high - args.direct_min_omega)), 0.0, 1.0)
                    w_direct_v = clamp((v_xy - args.direct_min_speed) / max(1e-6, (args.speed_high - args.direct_min_speed)), 0.0, 1.0)
                    w_direct = w_direct_om * w_direct_v
                    v_mix = (1.0 - w_direct) * v_mix + w_direct * v_direct
                    if w_direct >= 0.5:
                        pred_model = "direct"

            if args.circle_predict and plane_n is not None and plane_e is not None:
                v_xy = math.hypot(pvn, pve)
                om_abs = abs(omega)
                if len(circle_hist) >= max(3, args.circle_fit_min_points):
                    pts = [(hh[1], hh[2]) for hh in circle_hist]
                    fit = fit_circle_xy(pts)
                    if fit is not None:
                        est_cn, est_ce, est_r, est_rms = fit
                        if args.circle_min_radius <= est_r <= args.circle_max_radius:
                            circle_fit_rms = est_rms
                            if circle_cn is None:
                                circle_cn, circle_ce, circle_r = est_cn, est_ce, est_r
                            else:
                                circle_cn = lpf(circle_cn, est_cn, args.circle_center_alpha)
                                circle_ce = lpf(circle_ce, est_ce, args.circle_center_alpha)
                                circle_r = lpf(circle_r, est_r, args.circle_radius_alpha)
                            om_inst = 0.0
                            if est_r > 1e-3:
                                om_inst = v_xy / est_r
                                cross = (plane_n - est_cn) * pve - (plane_e - est_ce) * pvn
                                if cross < 0.0:
                                    om_inst = -om_inst
                            omega_circle = lpf(omega_circle, om_inst, args.circle_omega_alpha)

                if circle_cn is not None and circle_r is not None and abs(omega_circle) >= 1e-4:
                    dist_c = math.hypot(plane_n - circle_cn, plane_e - circle_ce)
                    circle_rel_err = abs(dist_c - circle_r) / max(1.0, circle_r)
                    w_geom = clamp(1.0 - (circle_fit_rms / max(1e-6, args.circle_fit_max_rms)), 0.0, 1.0)
                    w_om = clamp((om_abs - args.circle_min_omega) / max(1e-6, (args.omega_high - args.circle_min_omega)), 0.0, 1.0)
                    w_v = clamp((v_xy - args.circle_min_speed) / max(1e-6, (args.speed_high - args.circle_min_speed)), 0.0, 1.0)
                    w_circle = w_geom * w_om * w_v
                    v_circle = predict_circle_vel_seq(plane_n, plane_e, pvd, circle_cn, circle_ce, circle_r, omega_circle, dt, args.horizon)
                    v_mix = (1.0 - w_circle) * v_mix + w_circle * v_circle
                    if w_circle >= args.circle_lock_min_w:
                        pred_model = "circle"

            vseq = v_mix.reshape(-1)
            x_bias = sx @ x0 + sg @ vseq
            f = su.T @ qbar @ x_bias - wmbar @ vseq

            u_seq = solve_box_qp_pg(h, f, u_lo, u_hi, iters=120)
            u0 = u_seq[:3]
            vn_cmd = float(u0[0])
            ve_cmd = float(u0[1])
            vd_cmd = float(u0[2])
            vn_cmd, ve_cmd = clamp_norm2(vn_cmd, ve_cmd, args.max_xy_speed)
            vd_cmd = clamp(vd_cmd, -args.max_z_speed, args.max_z_speed)
            mode = "mpc"

        # Smooth command transitions to prevent box-like limit cycles under saturation.
        max_dxy = args.cmd_acc_max_xy * dt
        max_dz = args.cmd_acc_max_z * dt
        vn_cmd, ve_cmd = slew_limit_xy(vn_cmd, ve_cmd, prev_vn_cmd, prev_ve_cmd, max_dxy)
        vd_cmd = slew_limit_1d(vd_cmd, prev_vd_cmd, max_dz)

        # Convergence guard must be checked on the *actual* command after slew limiting.
        guard_suppress = (args.controller == "tracker_ff")
        if args.guard_enable and not guard_suppress:
            ex, ey, ez = float(x0[0]), float(x0[1]), float(x0[2])
            err_xy = math.hypot(ex, ey)
            close_req = required_close_rate(
                err_xy,
                args.guard_close_near_dist,
                args.guard_close_far_dist,
                args.guard_min_close_rate_near,
                args.guard_min_close_rate_far,
            )
            close_req = max(close_req, args.guard_min_close_rate)
            close_rate = radial_close_rate(ex, ey, pvn, pve, vn_cmd, ve_cmd, close_req)
            if close_rate < close_req:
                vn_guard = pvn + args.guard_kp_xy * ex
                ve_guard = pve + args.guard_kp_xy * ey
                vd_guard = pvd + args.guard_kp_z * ez
                vn_guard, ve_guard = clamp_norm2(vn_guard, ve_guard, args.max_xy_speed)
                vd_guard = clamp(vd_guard, -args.max_z_speed, args.max_z_speed)
                vn_cmd, ve_cmd, vd_cmd = vn_guard, ve_guard, vd_guard
                close_rate = radial_close_rate(ex, ey, pvn, pve, vn_cmd, ve_cmd, close_req)
                mode = "guard"
        else:
            close_rate = 0.0
            close_req = 0.0

        prev_vn_cmd, prev_ve_cmd, prev_vd_cmd = vn_cmd, ve_cmd, vd_cmd

        yaw_sp = yaw_from_velocity(pvn, pve, last_yaw)
        if carrier_att is not None and math.hypot(pvn, pve) < 0.5:
            yaw_sp = carrier_att.yaw
        yaw_sp = wrap_pi(yaw_sp)
        last_yaw = yaw_sp

        send_vel_sp(carrier, carrier_sys, carrier_comp, vn_cmd, ve_cmd, vd_cmd, yaw_sp)
        if time.time() - last_offboard_refresh >= max(0.2, args.offboard_refresh_sec):
            set_mode_offboard(carrier, carrier_sys, carrier_comp)
            last_offboard_refresh = time.time()

        frame_bias = -1.0
        if rel_gz is not None and rel_local is not None:
            frame_bias = math.hypot(rel_local[0] - rel_gz[0], rel_local[1] - rel_gz[1])

        if args.debug and (time.time() - last_dbg) >= 0.2:
            print(
                f"[DBG] rel=({rel[0]:.2f},{rel[1]:.2f},{rel[2]:.2f}) "
                f"e=({x0[0]:.2f},{x0[1]:.2f},{x0[2]:.2f}) "
                f"rel_v=({rel_v[0]:.2f},{rel_v[1]:.2f},{rel_v[2]:.2f}) "
                f"a_p=({apn:.2f},{ape:.2f},{apd:.2f}) "
                f"omega={omega:.3f}/{omega_chi:.3f}/{omega_circle:.3f} w_turn={w_turn:.2f} w_direct={w_direct:.2f} w_circle={w_circle:.2f} "
                f"u0(vcmd)=({u0[0]:.2f},{u0[1]:.2f},{u0[2]:.2f}) "
                f"cmd_v=({vn_cmd:.2f},{ve_cmd:.2f},{vd_cmd:.2f}) "
                f"mode={mode}/{pred_model} close_rate={close_rate:.2f} close_req={close_req:.2f} "
                f"lead={lead_dyn:.2f} ph_err={phase_err:.2f} ph_lag={phase_lag_t:.2f} ph_ld={phase_lead:.2f} "
                f"frame_bias={frame_bias:.2f} "
                f"circle_r={(-1.0 if circle_r is None else circle_r):.2f} circle_err={circle_rel_err:.3f} fit_rms={circle_fit_rms:.2f} "
                f"v_car=({cvn:.2f},{cve:.2f},{cvd:.2f}) v_plane=({pvn:.2f},{pve:.2f},{pvd:.2f})"
            )
            last_dbg = time.time()

        sleep_t = dt - (time.time() - t0)
        if sleep_t > 0:
            time.sleep(sleep_t)


if __name__ == "__main__":
    main()
