import argparse
import sys
import time

from pymavlink import mavutil

CONN = "tcp:127.0.0.1:5678"
ALT_MAX = 2.5
CELLS = 2
MIN_CELL_V = 3.75
ACK = {0: "ACCEPTED", 1: "TEMP_REJECTED", 2: "DENIED", 3: "UNSUPPORTED",
       4: "FAILED", 5: "IN_PROGRESS"}
MAV = mavutil.mavlink


def connect():
    m = mavutil.mavlink_connection(CONN)
    if m.wait_heartbeat(timeout=10) is None:
        sys.exit("No heartbeat. Is mavlink-router running and the FC powered?")
    for mid in (MAV.MAVLINK_MSG_ID_SYS_STATUS, MAV.MAVLINK_MSG_ID_GPS_RAW_INT,
                MAV.MAVLINK_MSG_ID_GLOBAL_POSITION_INT):
        m.mav.command_long_send(m.target_system, m.target_component,
                                MAV.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                                mid, int(1e6 / 4), 0, 0, 0, 0, 0)
    return m


def hb_state(m):
    hb = m.recv_match(type="HEARTBEAT", blocking=True, timeout=3)
    if hb is None:
        sys.exit("Lost heartbeat.")
    return m.flightmode, bool(hb.base_mode & MAV.MAV_MODE_FLAG_SAFETY_ARMED)


def cmd(m, command, *params):
    p = list(params) + [0] * (7 - len(params))
    m.mav.command_long_send(m.target_system, m.target_component,
                            command, 0, *p)
    end = time.time() + 5
    while time.time() < end:
        a = m.recv_match(type="COMMAND_ACK", blocking=True, timeout=1)
        if a is not None and a.command == command:
            print("  ack:", ACK.get(a.result, a.result))
            return a.result == 0
    print("  no ack")
    return False


def set_mode(m, name):
    modes = m.mode_mapping() or {}
    if name not in modes:
        sys.exit("Unknown mode " + name)
    cmd(m, MAV.MAV_CMD_DO_SET_MODE, MAV.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        modes[name])
    end = time.time() + 5
    while time.time() < end:
        m.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
        if m.flightmode == name:
            print("  mode is now", name)
            return True
    print("  WARNING: vehicle did not report", name)
    return False


def readings(m):
    ss = m.recv_match(type="SYS_STATUS", blocking=True, timeout=6)
    gps = m.recv_match(type="GPS_RAW_INT", blocking=True, timeout=6)
    if ss is None or gps is None:
        sys.exit("No SYS_STATUS / GPS_RAW_INT yet. Wait a few seconds, retry.")
    return ss.voltage_battery / 1000.0, gps.fix_type, gps.satellites_visible


def do_status(m):
    mode, armed = hb_state(m)
    v, fix, sats = readings(m)
    print("mode=%s armed=%s  battery=%.2f V (%.2f V/cell, %d cells)  "
          "gps_fix=%d sats=%d" % (mode, armed, v, v / CELLS, CELLS, fix, sats))


def do_takeoff(m, alt, live):
    if alt <= 0 or alt > ALT_MAX:
        sys.exit("Altitude must be 0 < alt <= %.1f m" % ALT_MAX)
    mode, armed = hb_state(m)
    v, fix, sats = readings(m)
    per_cell = v / CELLS
    problems = []
    if armed:
        problems.append("already armed")
    if fix < 3:
        problems.append("GPS fix %d (need 3D)" % fix)
    if per_cell < MIN_CELL_V:
        problems.append("battery %.2f V/cell < %.2f" % (per_cell, MIN_CELL_V))
    print("mode=%s armed=%s battery=%.2f V/cell gps_fix=%d sats=%d"
          % (mode, armed, per_cell, fix, sats))
    if problems:
        sys.exit("REFUSED: " + "; ".join(problems))
    if not live:
        print("DRY RUN OK: would set GUIDED, arm, take off to %.1f m. "
              "Nothing was sent. Add --live to do it." % alt)
        return
    print("Props on, inside the 20 ft cage, RC in your hand?")
    if input("Type FLY to continue: ").strip() != "FLY":
        sys.exit("Cancelled.")
    try:
        if not set_mode(m, "GUIDED"):
            sys.exit("Could not enter GUIDED, stopping.")
        print("arming...")
        if not cmd(m, MAV.MAV_CMD_COMPONENT_ARM_DISARM, 1):
            sys.exit("Arm refused. Read the PreArm message in Mission Planner.")
        end = time.time() + 8
        while time.time() < end:
            if hb_state(m)[1]:
                break
        else:
            sys.exit("Never reported armed, stopping.")
        print("taking off to %.1f m..." % alt)
        if not cmd(m, MAV.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, alt):
            print("Takeoff refused, landing.")
            set_mode(m, "LAND")
            return
        end = time.time() + 25
        while time.time() < end:
            g = m.recv_match(type="GLOBAL_POSITION_INT", blocking=True,
                             timeout=1)
            if g is None:
                continue
            h = g.relative_alt / 1000.0
            print("  height %.2f m" % h)
            if h >= 0.9 * alt:
                print("Reached height. Hovering in GUIDED. "
                      "Take over with the RC mode switch.")
                return
        print("Did not reach height in time, landing.")
        set_mode(m, "LAND")
    except KeyboardInterrupt:
        print("\nCtrl+C -> LAND")
        set_mode(m, "LAND")


def main():
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="c", required=True)
    sub.add_parser("status")
    sub.add_parser("land")
    t = sub.add_parser("takeoff")
    t.add_argument("alt", type=float)
    t.add_argument("--live", action="store_true")
    a = ap.parse_args()
    m = connect()
    if a.c == "status":
        do_status(m)
    elif a.c == "land":
        set_mode(m, "LAND")
    else:
        do_takeoff(m, a.alt, a.live)


main()
