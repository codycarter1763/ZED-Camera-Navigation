# clarq_mp_script.py
# Mission Planner Python Script (IronPython compatible)
#
# Scripts tab → Select Script → choose this file
# Check "Redirect Program Output"
#
# Two roles:
#   1. RELAY — watches for NAMED_VALUE_INT from GUI
#              and forwards them to Jetson via Pixhawk
#   2. AUTO  — monitors telemetry and fires triggers
#              automatically (low battery, near home, etc)

import MissionPlanner
import clr

# ── Command IDs ───────────────────────────────────────────────
CMD_PING       = 1
CMD_LAND       = 2
CMD_START_TAG  = 3
CMD_STOP_TAG   = 4
CMD_STOP_ALL   = 5
CMD_LAUNCH     = 6
CMD_LAUNCH_SIM = 7
CMD_SET_HOME   = 8
CMD_START_SCAN = 9
CMD_SAVE_END   = 10
CMD_GO_HOME    = 11

CMD_NAMES = {
    1:  "PING",
    2:  "LAND",
    3:  "START_TAG",
    4:  "STOP_TAG",
    5:  "STOP_ALL",
    6:  "LAUNCH",
    7:  "LAUNCH_SIM",
    8:  "SET_HOME",
    9:  "START_SCAN",
    10: "SAVE_END",
    11: "GO_HOME",
}

# ── Thresholds ────────────────────────────────────────────────
BATT_LOW_VOLTS = 14.0
BATT_LOW_PCT   = 15
HOME_RADIUS_M  = 10.0

# ── Jetson response labels ────────────────────────────────────
JETSON_RESPONSES = {
    "CLARQ_PONG":         "JETSON ONLINE",
    "CLARQ_HOME_SET":     "HOME POINT SAVED",
    "CLARQ_SCAN_STARTED": "SCAN MISSION STARTED",
    "CLARQ_END_SAVED":    "END POINT SAVED",
    "CLARQ_GOING_HOME":   "RETURNING HOME + LANDING",
    "CLARQ_LAUNCHED":     "DRONE STACK LAUNCHED",
    "CLARQ_LAUNCHED_SIM": "SIM STACK LAUNCHED",
    "CLARQ_ALL_STOPPED":  "ALL PROCESSES STOPPED",
    "CLARQ_TAG_RUNNING":  "APRILTAG RUNNING",
    "CLARQ_TAG_STOPPED":  "APRILTAG STOPPED",
}

# ── State ─────────────────────────────────────────────────────
go_home_sent    = False
scan_end_sent   = False
last_mode       = ""
last_statustext = ""
last_nvi_value  = -1
loop            = 0

print("[CLARQ] Script started")
print("[CLARQ] Relay + auto-trigger active")

def send(cmd_id, label):
    """Forward command to Jetson via Pixhawk."""
    try:
        mavlink.SendNamedInt("CLARQ", cmd_id)
        print("[CLARQ] >> " + label +
              " (id=" + str(cmd_id) + ")")
    except Exception as e:
        print("[CLARQ] Send failed: " + str(e))

# ── Main loop ─────────────────────────────────────────────────
while True:
    loop += 1

    # ── ROLE 1: RELAY — forward GUI button presses ────────────
    # The GUI sends NAMED_VALUE_INT with name=CLARQ, value=cmd_id
    # We read the latest value and forward it to the Jetson
    try:
        nvi = cs.messages.get("NAMED_VALUE_INT", None)
        if nvi is not None:
            name  = str(nvi.name).strip().rstrip('\x00')
            value = int(nvi.value)
            if name == "CLARQ" and value != last_nvi_value:
                last_nvi_value = value
                label = CMD_NAMES.get(value, "CMD_" + str(value))
                print("[CLARQ] GUI button: " + label)
                send(value, label)
    except Exception as e:
        pass

    # ── Check for Jetson responses via STATUSTEXT ─────────────
    try:
        stext = str(cs.messages.get("STATUSTEXT", ""))
        if stext and stext != last_statustext:
            last_statustext = stext
            for key in JETSON_RESPONSES:
                if key in stext:
                    print("[CLARQ] << JETSON: " +
                          JETSON_RESPONSES[key])
                    break
    except Exception:
        pass

    # ── Read telemetry ────────────────────────────────────────
    try:
        batt_v    = float(cs.battery_voltage)
        batt_pct  = int(cs.battery_remaining)
        alt       = float(cs.alt)
        dist_home = float(cs.DistToHome)
        mode      = str(cs.mode)
        armed     = bool(cs.armed)
    except Exception as e:
        print("[CLARQ] Telemetry error: " + str(e))
        Script.Sleep(1000)
        continue

    # Status every 10s
    if loop % 10 == 1:
        print("[CLARQ]"
              " Mode:" + mode +
              " Batt:" + str(int(batt_v * 10) / 10.0) +
              "V/" + str(batt_pct) + "%" +
              " Alt:" + str(int(alt * 10) / 10.0) + "m" +
              " Home:" + str(int(dist_home)) + "m" +
              " Armed:" + str(armed))

    # ── ROLE 2: AUTO triggers (armed flight only) ─────────────
    if not armed:
        if go_home_sent or scan_end_sent:
            print("[CLARQ] Disarmed - resetting triggers")
            go_home_sent  = False
            scan_end_sent = False
        last_mode = mode
        Script.Sleep(1000)
        continue

    # Low battery → GO HOME
    if not go_home_sent:
        if batt_v < BATT_LOW_VOLTS or batt_pct < BATT_LOW_PCT:
            print("[CLARQ] LOW BATTERY - sending GO HOME")
            send(CMD_GO_HOME, "GO HOME low battery")
            go_home_sent = True

    # Near home in LOITER → precision land
    if not go_home_sent:
        if dist_home < HOME_RADIUS_M and mode == "LOITER":
            print("[CLARQ] Near home in LOITER - precision land")
            send(CMD_GO_HOME, "GO HOME near home")
            go_home_sent = True

    # Entered LOITER → save end point
    if not scan_end_sent:
        if mode == "LOITER" and last_mode != "LOITER":
            print("[CLARQ] Entered LOITER - saving end point")
            send(CMD_SAVE_END, "SAVE END POINT")
            scan_end_sent = True

    last_mode = mode
    Script.Sleep(1000)
