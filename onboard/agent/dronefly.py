import os
# dronefly.py -- Phase III: LLM decides whether to land.
# Run ONLY in the 20' cage with the RC in your hand.
#
# Usage:
#   python dronefly.py
#
# Every run appends one line to dronefly_log.txt in this folder. Upload that
# file AND the drone's .BIN log to Canvas. Do not edit either file.
#
# WARNING: your API key goes in this file, and this file gets uploaded to
# Canvas. Replace it with the placeholder again before you submit, or your key
# is visible to anyone who reads your submission.

import re
import sys
import textwrap
import time
from datetime import datetime

from portkey_ai import Portkey
from pymavlink import mavutil


def _mk(code):
    """ANSI colour, disabled when stdout is not a terminal so a redirected
    log file stays free of escape codes."""
    if sys.stdout.isatty():
        return lambda t: f"\033[{code}m{t}\033[0m"
    return lambda t: t

BOLD   = _mk("1")
DIM    = _mk("2")
CYAN   = _mk("36")
YELLOW = _mk("33")
GREEN  = _mk("32")
RED    = _mk("31")
BLUE   = _mk("94")

TEAM = "B1"                      # <-- YOUR TEAM: A1..A10, B1..B10, ... F1..F10
API_KEY = os.environ["PORTKEY_API_KEY"]   # <-- YOUR KEY
LOGFILE = "dronefly_log.txt"

# Connect through mavlink-router's TCP port. This leaves the router running, so
# Mission Planner can stay connected and you can watch what your script does.
# (If you stop mavlink-router and want to talk to the serial port directly,
#  use "/dev/ttyS2" instead -- NOT "/dev/serial0", which does not exist on the
#  Orange Pi.)
CONNECTION = "tcp:127.0.0.1:5678"

CELLS = 2                          # 2S pack
#
# There is deliberately NO voltage threshold in this script. The LLM is given
# the battery state and nothing else, and must use its own judgement -- that
# is the whole point of the exercise. If we told it "land below X volts" the
# model would just be doing a comparison, and one line of Python would do the
# same job more cheaply and more reliably.
#
# For reference, measured on this airframe (do NOT put these in the prompt):
#   fresh pack                4.14 V/cell
#   normal flight             3.72 - 4.07 V/cell
#   median in flight          3.80 V/cell  (7045 samples, throttle >25%, 8 logs)
#   lowest ever seen in air   3.57 V/cell  (hovering, pack nearly done)
# Load sag is only ~0.03-0.05 V/cell here, so hovering and resting readings
# are nearly the same. A LiPo cell is roughly half charged at 3.80 V and
# nearly flat at 3.50 V.

if not re.fullmatch(r"[A-F](10|[1-9])", TEAM):
    raise SystemExit(f"TEAM must be A1..F10, not '{TEAM}'. Edit the top of dronefly.py.")

if API_KEY.startswith("PUT-YOUR"):
    raise SystemExit("Edit API_KEY at the top of dronefly.py with your Portkey key.")

# ---- 1. Connect to the drone ----
master = mavutil.mavlink_connection(CONNECTION)
if master.wait_heartbeat(timeout=15) is None:
    raise SystemExit(
        "No heartbeat in 15s.\n"
        "  - Is the drone powered on?\n"
        "  - Is mavlink-router running?   systemctl status mavlink-router")
print(GREEN("Heartbeat OK"))


def need(msg_type, timeout=10):
    """Wait for a message, with a clear error instead of an AttributeError."""
    m = master.recv_match(type=msg_type, blocking=True, timeout=timeout)
    if m is None:
        raise SystemExit(
            f"No {msg_type} within {timeout}s. The flight controller may not be\n"
            f"streaming it yet -- wait a few seconds after power-on and retry.")
    return m

def request_sys_status(rate_hz=2):
    """Ask the FC to stream SYS_STATUS.

    HEARTBEAT flows on its own, which is why land_now.py works with no setup.
    SYS_STATUS does not: on this link its stream rate is 0 until something
    requests it, so a plain recv_match("SYS_STATUS") just times out. We send
    both the modern request and the legacy one; whichever the firmware
    ignores is harmless.
    """
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS,   # which message
        int(1e6 / rate_hz),                          # interval, microseconds
        0, 0, 0, 0, 0)
    master.mav.request_data_stream_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS, rate_hz, 1)


# ---- 2. Sense: read the battery VOLTAGE, armed state and flight mode ----
#
# VOLTAGE ONLY. The current sensor on this aircraft is not calibrated, so
# nothing derived from it is read: not battery_remaining, not mAh consumed,
# not any "time remaining" figure. At rest with the props stopped it reads
# -0.35 A, which makes current_consumed negative (-93 mAh) and drives
# battery_remaining ABOVE 100% -- 103% observed at 3.82 V/cell, climbing the
# longer the FC stays powered. An earlier flight log showed the mirror of the
# same fault: 89% reported while the pack was already down to 3.82 V/cell.
#
# Voltage under load is a blunt instrument, but it is an actual measurement.
# The percentage is deliberately not read, not printed and not logged.
# Do not add it back.
request_sys_status()
msg = master.recv_match(type="SYS_STATUS", blocking=True, timeout=6)
if msg is None:                       # one retry: the first request can be
    request_sys_status()              # dropped while the FC is still booting
    msg = need("SYS_STATUS", timeout=8)
voltage = msg.voltage_battery / 1000.0          # volts
per_cell = voltage / CELLS

hb = need("HEARTBEAT")
armed = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
mode_before = master.flightmode
print(f"Battery {BOLD(f'{voltage:.2f} V')} "
      f"({BOLD(f'{per_cell:.2f} V/cell')})  "
      f"armed={armed}  mode={mode_before}")

# ---- 3. Think: ask the LLM ----
MODEL = "@zotgpt-api-azure/gpt-5.5"
TOKEN_BUDGET = 1024    # see the note at the API call below

system_msg = ("You are a drone safety pilot. Use your own judgement about "
              "battery chemistry and risk. Reply with one word: LAND or HOLD.")

# The last sentence before the question states the OBJECTIVE, deliberately
# with no number in it. Without it the model applies the cell-damage floor
# (~3.0-3.3 V under load) and keeps flying far past the point a pilot would
# land. Measured 50% LAND point, 5 calls per voltage:
#
#   no objective sentence   ~3.65 V/cell
#   this sentence           ~3.75 V/cell   <-- lands 5/5 at 3.71
#   "a cut-short flight costs nothing..."  ~3.82 V/cell  (too eager)
#
# This still leaves the decision to the model: no threshold is given, and the
# answer stays stochastic in a band either side of the boundary. It tells the
# model which question is being asked -- preserve the pack, not merely avoid
# damaging a cell -- which it cannot weigh if nobody says so.
user_msg = (f"The drone is hovering in a test cage, a few metres up. "
            f"Its {CELLS}S LiPo reads {voltage:.2f} V under hover load, "
            f"which is {per_cell:.2f} V per cell. "
            f"This LiPo is reused across many flights and must not be "
            f"deep-discharged. "
            f"Should it land now, or keep hovering?")

# Print the exact prompt that goes over the wire, so there is no guessing about
# what the model was actually asked.
print()
print(CYAN("=" * 72))
print(CYAN(BOLD(f"PROMPT SENT TO {MODEL}")))
print(CYAN("=" * 72))
print(YELLOW("[system]"))
print(DIM(textwrap.fill(system_msg, 70, initial_indent="  ",
                        subsequent_indent="  ")))
print(YELLOW("[user]"))
print(textwrap.fill(user_msg, 70, initial_indent="  ", subsequent_indent="  "))
print(CYAN("=" * 72))

portkey = Portkey(api_key=API_KEY)
t0 = time.time()
response = portkey.chat.completions.create(
    model=MODEL,
    messages=[
        {"role": "system", "content": system_msg},
        {"role": "user",   "content": user_msg},
    ],
    # This model rejects max_tokens outright: "Unsupported parameter:
    # 'max_tokens' is not supported with this model. Use
    # 'max_completion_tokens' instead."
    #
    # The budget is large on purpose. max_completion_tokens covers the model's
    # internal reasoning as well as the visible one-word reply, and reasoning
    # runs first. An exhausted budget returns content="" with
    # finish_reason="length" -- no answer at all -- which the parse step below
    # treats as an error rather than silently as HOLD.
    #
    # Measured on this model with a 2S pack. Tokens used for a ONE-WORD reply:
    #
    #   confident ends (4.14 or 3.50 V/cell)      55 - 74
    #   near its own boundary (3.62 - 3.66)      201 - 221
    #   with extra context in the prompt         109 - 523, and 2 of 6 calls
    #                                            blew a 512 budget entirely
    #
    # So 512 is NOT enough once the prompt gives the model anything to weigh.
    # 1024 costs nothing unless it is used: billing is on tokens produced, not
    # on the ceiling. Do not trim this to "tighten" the script.
    max_completion_tokens=TOKEN_BUDGET,
)
llm_seconds = round(time.time() - t0, 2)

# And print what came back, raw, before any parsing.
choice = response.choices[0]
usage = getattr(response, "usage", None)
print(BLUE(BOLD("RESPONSE")))
print(BLUE("=" * 72))
print("  raw content     : " + BOLD((choice.message.content or "").strip()
                                    or "<empty>"))
fr = getattr(choice, "finish_reason", None)
print("  finish_reason   : " + (GREEN(str(fr)) if fr == "stop" else RED(str(fr))))
if usage is not None:
    print(f"  tokens          : prompt={getattr(usage, 'prompt_tokens', '?')} "
          f"completion={getattr(usage, 'completion_tokens', '?')} of {TOKEN_BUDGET}")
print(f"  round trip      : {llm_seconds} s")
print(BLUE("=" * 72))
print()
raw = (response.choices[0].message.content or "").strip()

# An empty reply is a FAILED decision, not a HOLD. It happens when the token
# budget is used up by reasoning. Treat it as an error rather than letting it
# fall through to "keep flying", which is what a bare else-branch would do.
if not raw:
    finish = getattr(response.choices[0], "finish_reason", "unknown")
    raise SystemExit(
        f"The model returned an empty answer (finish_reason={finish!r}).\n"
        f"No decision was made, so no command was sent. If finish_reason is\n"
        f"'length', raise max_completion_tokens.")

# Be forgiving about punctuation and stray words: "LAND." or "land now" count.
# HOLD wins ties: "do not land" contains the word LAND, and reading that as a
# landing command would invert the model's answer.
upper = raw.upper()
if re.search(r"\bHOLD\b", upper) or re.search(r"\bNOT?\b\s+\w*\s*\bLAND\b", upper):
    decision = "HOLD"
elif re.search(r"\bLAND\b", upper):
    decision = "LAND"
else:
    raise SystemExit(f"Could not read a decision from the model's reply: {raw!r}")
tag = RED(BOLD(" LAND ")) if decision == "LAND" else GREEN(BOLD(" HOLD "))
print(f"DECISION: {tag}   (model said {raw.strip()!r})")

# Stamp team, LLM answer, Pi clock time and LLM latency into the DRONE's own
# log (.BIN), so it sits on the same timeline as the LAND mode change.
stamp = datetime.now().strftime("%H:%M:%S")
master.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_INFO,
    f"TEAM {TEAM} LLM {decision} T{stamp} {llm_seconds}s".encode()[:50])

# ---- 4. Act: Python checks the answer, then sends the command ----
if decision == "LAND":
    mode_id = master.mode_mapping()["LAND"]
    # DO_SET_MODE returns a COMMAND_ACK, so we can tell whether the flight
    # controller accepted it rather than assuming it did.
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id, 0, 0, 0, 0, 0)
    ack = master.recv_match(type="COMMAND_ACK", blocking=True, timeout=5)
    if ack and ack.command == mavutil.mavlink.MAV_CMD_DO_SET_MODE:
        print(f"LAND command acknowledged: result={ack.result} (0 = accepted)")
    else:
        print("No acknowledgement -- falling back to set_mode()")
        master.set_mode(mode_id)
else:
    print(GREEN("Holding. Nothing sent."))

# Read back the mode the drone actually ended up in.
time.sleep(1)
need("HEARTBEAT", timeout=5)
mode_after = master.flightmode

# ---- 5. Log it ----
line = (f"{datetime.now().isoformat(timespec='seconds')} team={TEAM} sysid={master.target_system} "
        f"voltage={voltage:.2f} per_cell={per_cell:.2f} "
        f"armed={armed} mode_before={mode_before} "
        f"llm={decision} llm_raw={raw!r} llm_seconds={llm_seconds} mode_after={mode_after}\n")
with open(LOGFILE, "a") as f:
    f.write(line)
print("Logged:", line.strip())
