#!/usr/bin/env python3
import argparse
import math
import re
import subprocess
import time

from pymavlink import mavutil


PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6
EARTH_RADIUS_M = 6378137.0


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


def clamp_norm2(vn, ve, vmax):
    nrm = math.hypot(vn, ve)
    if nrm <= max(1e-6, vmax):
        return vn, ve
    s = vmax / nrm
    return vn * s, ve * s


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


def geodetic_to_ned(lat, lon, alt_m, lat0, lon0, alt0_m):
    dlat = math.radians(lat - lat0)
    dlon = math.radians(lon - lon0)
    north = dlat * EARTH_RADIUS_M
    east = dlon * EARTH_RADIUS_M * math.cos(math.radians(lat0))
    down = alt0_m - alt_m
    return north, east, down


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
    request_message_interval(conn, target_sys, target_comp, mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, hz)
    request_message_interval(conn, target_sys, target_comp, mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, hz)


def global_valid(msg):
    if msg is None:
        return False
    # lat/lon == 0 usually means invalid in SITL startup.
    return not (int(msg.lat) == 0 and int(msg.lon) == 0)


def main():
    ap = argparse.ArgumentParser(description="Global-frame tracker: align carrier with plane in position and velocity.")
    ap.add_argument("--plane-url", default="udp:127.0.0.1:14542")
    ap.add_argument("--carrier-url", default="udp:127.0.0.1:14541")
    ap.add_argument("--plane-sysid", type=int, default=3)
    ap.add_argument("--carrier-sysid", type=int, default=2)
    ap.add_argument("--rate", type=float, default=20.0)
    ap.add_argument("--stream-rate", type=float, default=20.0)
    ap.add_argument("--target-rel-n", type=float, default=0.0)
    ap.add_argument("--target-rel-e", type=float, default=0.0)
    ap.add_argument("--target-rel-d", type=float, default=-1.0)
    ap.add_argument("--lead-sec", type=float, default=1.1)
    ap.add_argument("--lead-relv-gain", type=float, default=0.25, help="how much lead is applied to relative-velocity term in error predictor")
    ap.add_argument("--lag-adapt-enable", action="store_true", default=True, help="adaptive extra lead from estimated tracking lag")
    ap.add_argument("--lag-alpha", type=float, default=0.18, help="LPF alpha for lag estimate")
    ap.add_argument("--lag-max", type=float, default=2.0, help="max adaptive lag estimate (s)")
    ap.add_argument("--lag-gain", type=float, default=0.9, help="gain from lag estimate to extra lead")
    ap.add_argument("--kp-xy", type=float, default=0.26)
    ap.add_argument("--kd-xy", type=float, default=0.45)
    ap.add_argument("--ki-xy", type=float, default=0.02)
    ap.add_argument("--kp-z", type=float, default=0.70)
    ap.add_argument("--kd-z", type=float, default=0.45)
    ap.add_argument("--ki-z", type=float, default=0.03)
    ap.add_argument("--plane-acc-alpha", type=float, default=0.35, help="LPF alpha for plane accel estimate")
    ap.add_argument("--plane-acc-max", type=float, default=8.0, help="clip plane accel estimate (m/s^2)")
    ap.add_argument("--ff-acc-xy", type=float, default=0.8, help="acceleration feedforward gain in XY")
    ap.add_argument("--ff-acc-z", type=float, default=0.8, help="acceleration feedforward gain in Z")
    ap.add_argument("--i-zone-xy", type=float, default=40.0)
    ap.add_argument("--i-zone-z", type=float, default=10.0)
    ap.add_argument("--i-limit-xy", type=float, default=60.0)
    ap.add_argument("--i-limit-z", type=float, default=20.0)
    ap.add_argument("--close-boost", type=float, default=0.05)
    ap.add_argument("--close-boost-start", type=float, default=20.0)
    ap.add_argument("--close-boost-max", type=float, default=3.0)
    ap.add_argument("--max-xy-speed", type=float, default=12.0)
    ap.add_argument("--max-z-speed", type=float, default=3.0)
    ap.add_argument("--cmd-acc-max-xy", type=float, default=8.0)
    ap.add_argument("--cmd-acc-max-z", type=float, default=3.0)
    ap.add_argument("--guard-enable", action="store_true", default=True)
    ap.add_argument("--guard-min-close-rate", type=float, default=0.8)
    ap.add_argument("--guard-kp-xy", type=float, default=0.45)
    ap.add_argument("--guard-kp-z", type=float, default=0.65)
    ap.add_argument("--offboard-refresh-sec", type=float, default=0.4)
    ap.add_argument("--arm-carrier", action="store_true", default=True)
    ap.add_argument("--wait-arm", type=float, default=12.0)
    ap.add_argument("--relax-arm-checks", action="store_true", default=True)
    ap.add_argument("--use-gz-fallback", action="store_true", default=False, help="fallback to gz pose if global position invalid")
    ap.add_argument("--world-name", default="default")
    ap.add_argument("--plane-model", default="plane_2")
    ap.add_argument("--carrier-model", default="iris_aircarrier_1")
    ap.add_argument("--debug", action="store_true")
    args = ap.parse_args()

    dt = 1.0 / max(1.0, args.rate)
    lead = clamp(args.lead_sec, 0.0, 4.0)

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

    if hb_plane_sys == carrier_sys and hb_carrier_sys == plane_sys:
        plane, carrier = carrier, plane
        phb, chb = chb, phb
        hb_plane_sys, hb_carrier_sys = hb_carrier_sys, hb_plane_sys
        print("[WARN] plane/carrier UDP endpoints were swapped; auto-corrected.")

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
    geo_ref = None

    plane_g = None
    carrier_g = None
    carrier_att = None
    last_gz_t = 0.0
    prev_p_gz = None
    prev_c_gz = None
    prev_gz_t = None
    pvn_gz = pve_gz = pvd_gz = 0.0
    cvn_gz = cve_gz = cvd_gz = 0.0

    prev_vn_cmd = 0.0
    prev_ve_cmd = 0.0
    prev_vd_cmd = 0.0
    prev_pvn = prev_pve = prev_pvd = 0.0
    apn = ape = apd = 0.0
    lag_est = 0.0
    i_n = i_e = i_d = 0.0
    rel_target = [args.target_rel_n, args.target_rel_e, args.target_rel_d]

    while True:
        t0 = time.time()

        pm = plane.recv_match(type="GLOBAL_POSITION_INT", blocking=False)
        while pm is not None:
            if pm.get_srcSystem() == plane_sys:
                plane_g = pm
            pm = plane.recv_match(type="GLOBAL_POSITION_INT", blocking=False)

        cm = carrier.recv_match(type="GLOBAL_POSITION_INT", blocking=False)
        while cm is not None:
            if cm.get_srcSystem() == carrier_sys:
                carrier_g = cm
            cm = carrier.recv_match(type="GLOBAL_POSITION_INT", blocking=False)

        ca = carrier.recv_match(type="ATTITUDE", blocking=False)
        while ca is not None:
            if ca.get_srcSystem() == carrier_sys:
                carrier_att = ca
            ca = carrier.recv_match(type="ATTITUDE", blocking=False)

        rel = None
        rel_vn = rel_ve = rel_vd = 0.0
        pvn = pve = pvd = 0.0
        cvn = cve = cvd = 0.0
        source = "none"

        if global_valid(plane_g) and global_valid(carrier_g):
            p_lat = float(plane_g.lat) * 1e-7
            p_lon = float(plane_g.lon) * 1e-7
            p_alt = float(plane_g.alt) * 1e-3
            c_lat = float(carrier_g.lat) * 1e-7
            c_lon = float(carrier_g.lon) * 1e-7
            c_alt = float(carrier_g.alt) * 1e-3

            if geo_ref is None:
                geo_ref = (p_lat, p_lon, p_alt)

            pn, pe, pd = geodetic_to_ned(p_lat, p_lon, p_alt, geo_ref[0], geo_ref[1], geo_ref[2])
            cn, ce, cd = geodetic_to_ned(c_lat, c_lon, c_alt, geo_ref[0], geo_ref[1], geo_ref[2])
            rel = [pn - cn, pe - ce, pd - cd]

            pvn = float(plane_g.vx) * 0.01
            pve = float(plane_g.vy) * 0.01
            pvd = float(plane_g.vz) * 0.01
            cvn = float(carrier_g.vx) * 0.01
            cve = float(carrier_g.vy) * 0.01
            cvd = float(carrier_g.vz) * 0.01
            rel_vn = pvn - cvn
            rel_ve = pve - cve
            rel_vd = pvd - cvd
            source = "global"
        elif args.use_gz_fallback and (time.time() - last_gz_t) >= 0.08:
            p = get_model_pose(args.plane_model, args.world_name)
            c = get_model_pose(args.carrier_model, args.world_name)
            last_gz_t = time.time()
            if p is not None and c is not None:
                rel = [p[1] - c[1], p[0] - c[0], -(p[2] - c[2])]
                if prev_p_gz is not None and prev_c_gz is not None and prev_gz_t is not None:
                    dts = max(1e-3, time.time() - prev_gz_t)
                    pvn_gz = (p[1] - prev_p_gz[1]) / dts
                    pve_gz = (p[0] - prev_p_gz[0]) / dts
                    pvd_gz = -((p[2] - prev_p_gz[2]) / dts)
                    cvn_gz = (c[1] - prev_c_gz[1]) / dts
                    cve_gz = (c[0] - prev_c_gz[0]) / dts
                    cvd_gz = -((c[2] - prev_c_gz[2]) / dts)
                    pvn, pve, pvd = pvn_gz, pve_gz, pvd_gz
                    cvn, cve, cvd = cvn_gz, cve_gz, cvd_gz
                    rel_vn = pvn - cvn
                    rel_ve = pve - cve
                    rel_vd = pvd - cvd
                prev_p_gz = p
                prev_c_gz = c
                prev_gz_t = time.time()
                source = "gz"

        if rel is None:
            send_vel_sp(carrier, carrier_sys, carrier_comp, 0.0, 0.0, 0.0, last_yaw)
            if args.debug and (time.time() - last_dbg) > 1.0:
                print("[DBG] waiting GLOBAL_POSITION_INT")
                last_dbg = time.time()
            time.sleep(dt)
            continue

        ex = rel[0] - rel_target[0]
        ey = rel[1] - rel_target[1]
        ez = rel[2] - rel_target[2]

        if args.lag_adapt_enable:
            vrel2 = rel_vn * rel_vn + rel_ve * rel_ve
            if vrel2 > 0.04:
                lag_inst = (ex * rel_vn + ey * rel_ve) / vrel2
                lag_inst = clamp(lag_inst, 0.0, args.lag_max)
            else:
                lag_inst = 0.0
            lag_est = (1.0 - args.lag_alpha) * lag_est + args.lag_alpha * lag_inst
        else:
            lag_est = 0.0
        lead_eff = clamp(lead + args.lag_gain * lag_est, 0.0, lead + args.lag_max)

        raw_apn = clamp((pvn - prev_pvn) / dt, -args.plane_acc_max, args.plane_acc_max)
        raw_ape = clamp((pve - prev_pve) / dt, -args.plane_acc_max, args.plane_acc_max)
        raw_apd = clamp((pvd - prev_pvd) / dt, -args.plane_acc_max, args.plane_acc_max)
        prev_pvn, prev_pve, prev_pvd = pvn, pve, pvd
        apn = (1.0 - args.plane_acc_alpha) * apn + args.plane_acc_alpha * raw_apn
        ape = (1.0 - args.plane_acc_alpha) * ape + args.plane_acc_alpha * raw_ape
        apd = (1.0 - args.plane_acc_alpha) * apd + args.plane_acc_alpha * raw_apd

        # Use most lead on feedforward velocity, and less on error term to avoid steady offset.
        epx = ex + args.lead_relv_gain * lead_eff * rel_vn + 0.5 * args.lead_relv_gain * (lead_eff * lead_eff) * apn
        epy = ey + args.lead_relv_gain * lead_eff * rel_ve + 0.5 * args.lead_relv_gain * (lead_eff * lead_eff) * ape
        epz = ez + args.lead_relv_gain * lead_eff * rel_vd + 0.5 * args.lead_relv_gain * (lead_eff * lead_eff) * apd

        pvn_ff = pvn + args.ff_acc_xy * lead_eff * apn
        pve_ff = pve + args.ff_acc_xy * lead_eff * ape
        pvd_ff = pvd + args.ff_acc_z * lead_eff * apd

        if abs(ex) <= args.i_zone_xy:
            i_n = clamp(i_n + ex * dt, -args.i_limit_xy, args.i_limit_xy)
        else:
            i_n *= 0.98
        if abs(ey) <= args.i_zone_xy:
            i_e = clamp(i_e + ey * dt, -args.i_limit_xy, args.i_limit_xy)
        else:
            i_e *= 0.98
        if abs(ez) <= args.i_zone_z:
            i_d = clamp(i_d + ez * dt, -args.i_limit_z, args.i_limit_z)
        else:
            i_d *= 0.98

        vn_cmd = pvn_ff + args.kp_xy * epx + args.kd_xy * rel_vn + args.ki_xy * i_n
        ve_cmd = pve_ff + args.kp_xy * epy + args.kd_xy * rel_ve + args.ki_xy * i_e
        vd_cmd = pvd_ff + args.kp_z * epz + args.kd_z * rel_vd + args.ki_z * i_d

        err_xy = math.hypot(ex, ey)
        if args.close_boost > 1e-6 and err_xy > args.close_boost_start:
            boost = args.close_boost * (err_xy - args.close_boost_start)
            boost = clamp(boost, 0.0, args.close_boost_max)
            inv = 1.0 / max(1e-6, err_xy)
            vn_cmd += boost * ex * inv
            ve_cmd += boost * ey * inv

        vn_cmd, ve_cmd = clamp_norm2(vn_cmd, ve_cmd, args.max_xy_speed)
        vd_cmd = clamp(vd_cmd, -args.max_z_speed, args.max_z_speed)

        max_dxy = args.cmd_acc_max_xy * dt
        max_dz = args.cmd_acc_max_z * dt
        vn_cmd, ve_cmd = slew_limit_xy(vn_cmd, ve_cmd, prev_vn_cmd, prev_ve_cmd, max_dxy)
        vd_cmd = slew_limit_1d(vd_cmd, prev_vd_cmd, max_dz)
        mode = "track"

        if err_xy > 1e-3:
            close_rate = -((ex * (pvn - vn_cmd) + ey * (pve - ve_cmd)) / err_xy)
        else:
            close_rate = args.guard_min_close_rate

        if args.guard_enable and close_rate < args.guard_min_close_rate:
            vn_guard = pvn + args.guard_kp_xy * ex
            ve_guard = pve + args.guard_kp_xy * ey
            vd_guard = pvd + args.guard_kp_z * ez
            vn_guard, ve_guard = clamp_norm2(vn_guard, ve_guard, args.max_xy_speed)
            vd_guard = clamp(vd_guard, -args.max_z_speed, args.max_z_speed)
            vn_cmd, ve_cmd, vd_cmd = vn_guard, ve_guard, vd_guard
            mode = "guard"
            if err_xy > 1e-3:
                close_rate = -((ex * (pvn - vn_cmd) + ey * (pve - ve_cmd)) / err_xy)
            else:
                close_rate = args.guard_min_close_rate

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

        if args.debug and (time.time() - last_dbg) >= 0.2:
            a_plane_xy = math.hypot(apn, ape)
            a_margin = args.cmd_acc_max_xy - a_plane_xy
            v_margin = args.max_xy_speed - math.hypot(pvn, pve)
            track_ok = (a_margin > 1.0 and v_margin > 0.8)
            print(
                f"[DBG] src={source} "
                f"rel=({rel[0]:.2f},{rel[1]:.2f},{rel[2]:.2f}) "
                f"e=({ex:.2f},{ey:.2f},{ez:.2f}) "
                f"rel_v=({rel_vn:.2f},{rel_ve:.2f},{rel_vd:.2f}) "
                f"cmd_v=({vn_cmd:.2f},{ve_cmd:.2f},{vd_cmd:.2f}) mode={mode} "
                f"close_rate={close_rate:.2f} "
                f"a_p=({apn:.2f},{ape:.2f},{apd:.2f}) a_xy={a_plane_xy:.2f} a_margin={a_margin:.2f} "
                f"v_margin={v_margin:.2f} track_ok={int(track_ok)} "
                f"lag={lag_est:.2f} lead_eff={lead_eff:.2f} "
                f"v_car=({cvn:.2f},{cve:.2f},{cvd:.2f}) v_plane=({pvn:.2f},{pve:.2f},{pvd:.2f})"
            )
            last_dbg = time.time()

        sleep_t = dt - (time.time() - t0)
        if sleep_t > 0:
            time.sleep(sleep_t)


if __name__ == "__main__":
    main()
