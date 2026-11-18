#!/usr/bin/env python3
"""
land_now.py — put an ArduPilot vehicle into LAND mode and exit.

Derived from land_ardupilot_landmode.py, corrected for this aircraft
(Orange Pi Zero 3W + MatekF405-TE, talking through mavlink-router).

    python3 land_now.py                 # default: tcp:127.0.0.1:5678
    python3 land_now.py --status        # read-only, changes nothing
    python3 land_now.py --device /dev/ttyS2 --baud 57600   # direct serial

Why the default is TCP, not a serial device
-------------------------------------------
mavlink-routerd holds /dev/ttyS2 *exclusively*. A second process opening that
port either fails or corrupts the stream for both. The router republishes the
same link on TCP 5678, which is the safe place to connect.

(The original script defaulted to /dev/serial0, which is a Raspberry Pi symlink
and does not exist on this board at all.)

Why DO_SET_MODE instead of SET_MODE
-----------------------------------
SET_MODE is fire-and-forget: a malformed or rejected request looks identical to
a successful one. DO_SET_MODE goes through COMMAND_LONG and returns a
COMMAND_ACK, so the result can be checked. The mode change is then confirmed
against the vehicle's own HEARTBEAT before reporting success.
"""
import argparse
import sys
import time

try:
    from pymavlink import mavutil
except ImportError:
    sys.exit("pymavlink is not installed.  python3 -m pip install pymavlink")

DEFAULT_DEVICE = "tcp:127.0.0.1:5678"
DEFAULT_BAUD = 57600          # only used for a direct serial connection

ACK = {0: "ACCEPTED", 1: "TEMPORARILY_REJECTED", 2: "DENIED",
       3: "UNSUPPORTED", 4: "FAILED", 5: "IN_PROGRESS"}


def connect(device, baud):
    if device.startswith(("tcp:", "udp:", "tcpin:")):
        return mavutil.mavlink_connection(device)
    return mavutil.mavlink_connection(device, baud=baud)


def wait_heartbeat(master, timeout=10):
    print("Waiting for heartbeat...")
    hb = master.wait_heartbeat(timeout=timeout)
    if hb is None:
        raise RuntimeError(
            "No heartbeat within %ds.\n"
            "  - Is the aircraft powered on?\n"
            "  - Is mavlink-router running?   systemctl status mavlink-router\n"
            "  - If using a serial device: is another process holding the port?"
            % timeout)
    print("Heartbeat: sys=%d comp=%d" % (master.target_system, master.target_component))
    return hb


def current_mode(master, timeout=3):
    hb = master.recv_match(type="HEARTBEAT", blocking=True, timeout=timeout)
    if hb is None:
        return None, None
    armed = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
    name = None
    for n, i in (master.mode_mapping() or {}).items():
        if i == hb.custom_mode:
            name = n
            break
    return name, armed


def set_land(master, timeout=5):
    modes = master.mode_mapping() or {}
    if "LAND" not in modes:
        raise RuntimeError("'LAND' not in mode_mapping: %s" % list(modes.keys()))
    mode_id = modes["LAND"]

    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id, 0, 0, 0, 0, 0)

    end = time.time() + timeout
    while time.time() < end:
        m = master.recv_match(type="COMMAND_ACK", blocking=True, timeout=1)
        if m and m.command == mavutil.mavlink.MAV_CMD_DO_SET_MODE:
            print("DO_SET_MODE ack: %s" % ACK.get(m.result, m.result))
            if m.result != 0:
                # Older firmware may not accept DO_SET_MODE; fall back.
                print("Falling back to SET_MODE (no acknowledgement available)...")
                master.mav.set_mode_send(
                    master.target_system,
                    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                    mode_id)
            break
    else:
        print("No ack received; falling back to SET_MODE...")
        master.mav.set_mode_send(
            master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id)

    # Confirm against the vehicle's own heartbeat rather than assuming.
    end = time.time() + timeout
    while time.time() < end:
        name, armed = current_mode(master, timeout=1)
        if name == "LAND":
            print("CONFIRMED: vehicle reports LAND (armed=%s)" % armed)
            return True
    print("WARNING: vehicle did not report LAND within %ds." % timeout)
    return False


def main():
    ap = argparse.ArgumentParser(description="Set LAND mode on ArduPilot and exit.")
    ap.add_argument("--device", default=DEFAULT_DEVICE,
                    help="connection string (default: %s). Use a serial path only "
                         "if mavlink-router is stopped." % DEFAULT_DEVICE)
    ap.add_argument("--baud", type=int, default=DEFAULT_BAUD,
                    help="baud rate, serial connections only (default %d)" % DEFAULT_BAUD)
    ap.add_argument("--status", action="store_true",
                    help="report mode and armed state, change nothing")
    a = ap.parse_args()

    try:
        master = connect(a.device, a.baud)
    except Exception as e:
        print("Could not open %s: %s" % (a.device, e))
        return 1

    try:
        wait_heartbeat(master, timeout=10)
        name, armed = current_mode(master)
        print("Current: mode=%s armed=%s" % (name, armed))

        if a.status:
            return 0

        # Set LAND regardless of armed state. Doing it while disarmed is a
        # legitimate thing to want: it stages the vehicle so the next arm goes
        # straight into LAND, and it is a safe way to verify the command path
        # on the bench without anything spinning.
        if not armed:
            print("Vehicle is disarmed — setting LAND anyway.")

        return 0 if set_land(master) else 1
    except KeyboardInterrupt:
        print("Interrupted.")
        return 130
    except Exception as e:
        print("Error: %s" % e)
        return 1
    finally:
        try:
            master.close()
        except Exception:
            pass


if __name__ == "__main__":
    sys.exit(main())
