#!/usr/bin/env python3
import argparse
import math
import time

from pymavlink import mavutil


PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def wrap_pi(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def set_mode_offboard(conn, target_sys, target_comp):
    conn.mav.command_long_send(
        target_sys,
        target_comp,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        0,
        1,  # MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
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


def send_pos_sp(conn, target_sys, target_comp, n, e, d, yaw):
    # position + yaw only
    type_mask = (
        (1 << 3) | (1 << 4) | (1 << 5) |
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
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        float(yaw),
        0.0,
    )


def wait_heartbeat(conn, name):
    hb = conn.wait_heartbeat(timeout=10)
    if hb is None:
        raise RuntimeError(f"{name}: heartbeat timeout")
    return hb.get_srcSystem(), hb.get_srcComponent()


def main():
    ap = argparse.ArgumentParser(description="Offboard: carrier tracks plane in LOCAL_NED.")
    ap.add_argument("--plane-url", default="udp:127.0.0.1:14542")
    ap.add_argument("--carrier-url", default="udp:127.0.0.1:14541")
    ap.add_argument("--plane-sysid", type=int, default=3)
    ap.add_argument("--carrier-sysid", type=int, default=2)
    ap.add_argument("--rate", type=float, default=20.0)
    ap.add_argument("--offset-forward", type=float, default=-20.0, help="m in plane body-forward")
    ap.add_argument("--offset-right", type=float, default=0.0, help="m in plane body-right")
    ap.add_argument("--offset-down", type=float, default=0.0, help="m in NED down")
    ap.add_argument("--follow-yaw", action="store_true")
    ap.add_argument("--arm", action="store_true")
    ap.add_argument("--debug", action="store_true")
    ap.add_argument("--debug-interval", type=float, default=0.2)
    args = ap.parse_args()

    plane = mavutil.mavlink_connection(args.plane_url, autoreconnect=True)
    carrier = mavutil.mavlink_connection(args.carrier_url, autoreconnect=True)

    plane_hb_sys, _ = wait_heartbeat(plane, "plane")
    carrier_hb_sys, carrier_hb_comp = wait_heartbeat(carrier, "carrier")
    print(
        f"[INFO] connected: plane={args.plane_url}(hb sys={plane_hb_sys}), "
        f"carrier={args.carrier_url}(hb sys={carrier_hb_sys})"
    )

    target_carrier_sys = args.carrier_sysid or carrier_hb_sys
    target_plane_sys = args.plane_sysid or plane_hb_sys

    dt = 1.0 / max(args.rate, 1.0)
    last_dbg = 0.0

    # Prime Offboard with a short burst of setpoints.
    t_prime_end = time.time() + 1.0
    while time.time() < t_prime_end:
        msg_p = plane.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=1.0)
        msg_a = plane.recv_match(type="ATTITUDE", blocking=True, timeout=1.0)
        if not msg_p or not msg_a:
            continue
        if msg_p.get_srcSystem() != target_plane_sys or msg_a.get_srcSystem() != target_plane_sys:
            continue
        pn, pe, pd = msg_p.x, msg_p.y, msg_p.z
        pyaw = msg_a.yaw
        dn = math.cos(pyaw) * args.offset_forward - math.sin(pyaw) * args.offset_right
        de = math.sin(pyaw) * args.offset_forward + math.cos(pyaw) * args.offset_right
        tn, te, td = pn + dn, pe + de, pd + args.offset_down
        tyaw = pyaw if args.follow_yaw else 0.0
        send_pos_sp(carrier, target_carrier_sys, carrier_hb_comp, tn, te, td, tyaw)
        time.sleep(dt)

    if args.arm:
        arm(carrier, target_carrier_sys, carrier_hb_comp, do_arm=True)
        time.sleep(0.2)
    set_mode_offboard(carrier, target_carrier_sys, carrier_hb_comp)
    print(f"[INFO] offboard requested on carrier sysid={target_carrier_sys}")

    while True:
        t0 = time.time()
        msg_p = plane.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=1.0)
        msg_a = plane.recv_match(type="ATTITUDE", blocking=True, timeout=1.0)
        if not msg_p or not msg_a:
            continue
        if msg_p.get_srcSystem() != target_plane_sys or msg_a.get_srcSystem() != target_plane_sys:
            continue

        pn, pe, pd = msg_p.x, msg_p.y, msg_p.z
        pyaw = msg_a.yaw
        dn = math.cos(pyaw) * args.offset_forward - math.sin(pyaw) * args.offset_right
        de = math.sin(pyaw) * args.offset_forward + math.cos(pyaw) * args.offset_right
        tn = pn + dn
        te = pe + de
        td = pd + args.offset_down
        tyaw = wrap_pi(pyaw) if args.follow_yaw else 0.0

        send_pos_sp(carrier, target_carrier_sys, carrier_hb_comp, tn, te, td, tyaw)

        if args.debug and (time.time() - last_dbg) >= max(0.05, args.debug_interval):
            print(
                f"[DBG] plane_ned=({pn:.1f},{pe:.1f},{pd:.1f}) "
                f"target_ned=({tn:.1f},{te:.1f},{td:.1f}) yaw={tyaw:.2f}"
            )
            last_dbg = time.time()

        sleep_t = dt - (time.time() - t0)
        if sleep_t > 0:
            time.sleep(sleep_t)


if __name__ == "__main__":
    main()

