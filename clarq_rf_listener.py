import json
import os
import subprocess
import sys
import threading
import time
import serial

# Force unbuffered output for systemd journal / SSH viewing
os.environ['PYTHONUNBUFFERED'] = '1'
sys.stdout.reconfigure(line_buffering=True)
sys.stderr.reconfigure(line_buffering=True)

# ── RYLR998 LoRa UART Config ──────────────────────────────────
LORA_PORT       = "/dev/ttyTHS1"
LORA_BAUD       = 115200
LORA_NETWORK_ID = 18
LORA_ADDRESS    = 11
LORA_DEST_ADDR  = 10
LORA_BAND       = 915000000
LORA_SF         = 9
LORA_BANDWIDTH  = 7
LORA_CR         = 1
LORA_PREAMBLE   = 12
LORA_POWER      = 22

# ── Command IDs ───────────────────────────────────────────────
CMD_ID_PING            = 1
CMD_ID_LAND            = 2
CMD_ID_START_TAG       = 3
CMD_ID_STOP_TAG        = 4
CMD_ID_STOP_ALL        = 5
CMD_ID_LAUNCH          = 6
CMD_ID_LAUNCH_SIM      = 7
CMD_ID_SET_HOME        = 8
CMD_ID_START_SCAN      = 9
CMD_ID_SAVE_END        = 10
CMD_ID_GO_HOME         = 11
CMD_ID_START_3D_FUSION = 12
CMD_ID_STOP_3D_FUSION  = 13
CMD_ID_START_PHOTO     = 14
CMD_ID_START_VIDEO     = 15
CMD_ID_STOP_CAPTURE    = 16
CMD_ID_SET_GIMBAL      = 17

# ── Status response IDs ───────────────────────────────────────
STATUS_PONG              = CMD_ID_PING
STATUS_LAND_ACK          = CMD_ID_LAND
STATUS_TAG_RUNNING       = CMD_ID_START_TAG
STATUS_TAG_STOPPED       = CMD_ID_STOP_TAG
STATUS_ALL_STOPPED       = CMD_ID_STOP_ALL
STATUS_LAUNCHED          = CMD_ID_LAUNCH
STATUS_LAUNCHED_SIM      = CMD_ID_LAUNCH_SIM
STATUS_HOME_SET          = CMD_ID_SET_HOME
STATUS_SCAN_STARTED      = CMD_ID_START_SCAN
STATUS_END_SAVED         = CMD_ID_SAVE_END
STATUS_GOING_HOME        = CMD_ID_GO_HOME
STATUS_3D_FUSION_STARTED = CMD_ID_START_3D_FUSION
STATUS_3D_FUSION_STOPPED = CMD_ID_STOP_3D_FUSION
STATUS_PHOTO_STARTED     = CMD_ID_START_PHOTO
STATUS_VIDEO_STARTED     = CMD_ID_START_VIDEO
STATUS_CAPTURE_STOPPED   = CMD_ID_STOP_CAPTURE
STATUS_GIMBAL_SET        = CMD_ID_SET_GIMBAL

# ── Paths ─────────────────────────────────────────────────────
BASE          = os.path.expanduser('~/ZED Navigation')
ZED_NAV_DIR   = os.path.expanduser('~/ZED Navigation/ZED-Camera-Navigation')
LAUNCH_SCRIPT = f'{BASE}/launch_drone.sh'
RETURN_LAND   = f'{ZED_NAV_DIR}/return_land.py'
SAVE_POSITION = f'{ZED_NAV_DIR}/save_position.py'
ZED_FUSION    = f'{ZED_NAV_DIR}/zed_fusion_scan.py'
ZED_PHOTO     = f'{ZED_NAV_DIR}/zed_photo_capture.py'
ZED_VIDEO     = f'{ZED_NAV_DIR}/zed_video_capture.py'
ORIGIN_FILE   = f'{ZED_NAV_DIR}/origin_T0.json'
END_POS_FILE  = f'{ZED_NAV_DIR}/scan_end_pos.json'
TRIGGER_FILE  = f'{ZED_NAV_DIR}/.save_end_trigger'
TAGS_CONFIG   = f'{ZED_NAV_DIR}/tags_config.yaml'
SCANS_DIR     = f'{ZED_NAV_DIR}/scans'
CAPTURES_DIR  = f'{ZED_NAV_DIR}/captures'

# ── Gimbal serial port ────────────────────────────────────────
GIMBAL_PORT = "/dev/ttyACM2"
GIMBAL_BAUD = 115200

# ── ROS/ZED setup ─────────────────────────────────────────────
ROS_SETUP    = 'source /opt/ros/humble/setup.bash'
ZED_SETUP    = 'source ~/ros2_ws/install/setup.bash'
FULL_SETUP   = f'{ROS_SETUP} && {ZED_SETUP}'
TMUX_SESSION = 'drone'

# ── Process handles ───────────────────────────────────────────
_apriltag_proc    = None
_return_land_proc = None
_scan_proc        = None
_fusion_proc      = None
_position_proc    = None
_photo_proc       = None
_video_proc       = None

# ── LoRa serial handle + lock ─────────────────────────────────
_lora: serial.Serial | None = None
_lora_lock = threading.Lock()


# ═══════════════════════════════════════════════════════════════
# RYLR998 INIT
# ═══════════════════════════════════════════════════════════════
def _send_at(ser: serial.Serial, cmd: str, timeout: float = 3.0) -> bool:
    ser.reset_input_buffer()
    ser.write((cmd + "\r\n").encode())

    deadline = time.time() + timeout
    resp = ""
    while time.time() < deadline:
        time.sleep(0.05)
        if ser.in_waiting:
            resp += ser.read(ser.in_waiting).decode(errors="ignore")
        if "+OK"    in resp: return True
        if "+READY" in resp: return True
        if "+ERR"   in resp: return False
    return False


def init_lora() -> serial.Serial:
    global _lora
    print("[LoRa] Initialising RYLR998...")

    ser = serial.Serial(LORA_PORT, 115200, timeout=1)
    time.sleep(1.0)

    ok = _send_at(ser, "AT", timeout=2.0)
    print(f"[LoRa] Initial ping: {'✓' if ok else 'FAIL — check wiring'}")

    if not ok:
        print("[LoRa] Cannot communicate with module — stopping here")
        ser.close()
        sys.exit(1)

    cmds = [
        ("AT+RESET",    2.0),
        (f"AT+ADDRESS={LORA_ADDRESS}",                                          1.0),
        (f"AT+NETWORKID={LORA_NETWORK_ID}",                                     1.0),
        (f"AT+BAND={LORA_BAND}",                                                1.0),
        (f"AT+PARAMETER={LORA_SF},{LORA_BANDWIDTH},{LORA_CR},{LORA_PREAMBLE}", 1.0),
        (f"AT+CRFOP={LORA_POWER}",                                              1.0),
    ]

    for cmd, wait in cmds:
        time.sleep(0.3)
        ok = _send_at(ser, cmd, timeout=wait + 1)
        print(f"  {cmd}  →  {'✓' if ok else 'WARN (no +OK)'}")

    _lora = ser
    print(f"[LoRa] Ready on {LORA_PORT} @ {LORA_BAUD} baud ✓")
    return ser


# ═══════════════════════════════════════════════════════════════
# SEND STATUS BACK TO ARDUINO / GUI
# ═══════════════════════════════════════════════════════════════
def send_status(status_id: int):
    global _lora
    if _lora is None:
        return

    checksum    = status_id ^ 0xAA
    hex_payload = f"{status_id:02X}{checksum:02X}"
    at_cmd      = f"AT+SEND={LORA_DEST_ADDR},4,{hex_payload}"  # 4 ASCII chars

    with _lora_lock:
        for attempt in range(3):
            ok = _send_at(_lora, at_cmd, timeout=4.0)
            if ok:
                print(f"[LoRa] Status sent: id={status_id} ✓")
                return
            print(f"[LoRa] Send attempt {attempt+1} failed, retrying...")
            time.sleep(0.1)
        print(f"[LoRa] Status send failed after 3 attempts: id={status_id}")


# ═══════════════════════════════════════════════════════════════
# INCOMING PACKET PARSER
# ═══════════════════════════════════════════════════════════════
def parse_rcv(line: str) -> int | None:
    if not line.startswith("+RCV="):
        return None

    try:
        body  = line[5:]
        parts = body.split(",")
        if len(parts) < 5:
            return None

        hex_data = parts[2]
        rssi     = parts[3]
        snr      = parts[4]

        if len(hex_data) < 4:
            print(f"[LoRa] Short payload: '{hex_data}'")
            return None

        cmd_id   = int(hex_data[0:2], 16)
        checksum = int(hex_data[2:4], 16)

        if checksum != (cmd_id ^ 0xAA):
            print(f"[LoRa] Checksum fail — "
                  f"got {checksum:#x} expected {cmd_id ^ 0xAA:#x}")
            return None

        if not 1 <= cmd_id <= 17:
            print(f"[LoRa] Out-of-range cmd_id: {cmd_id}")
            return None

        print(f"[LoRa] RX  cmd={cmd_id}  RSSI={rssi} dBm  SNR={snr} dB")
        return cmd_id

    except Exception as e:
        print(f"[LoRa] Parse error: {e}  raw='{line}'")
        return None


# ═══════════════════════════════════════════════════════════════
# HELPERS
# ═══════════════════════════════════════════════════════════════
def _run(label, cmd):
    try:
        proc = subprocess.Popen(
            cmd, shell=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE)
        print(f"  {label} started (pid {proc.pid})")
        return proc
    except Exception as e:
        print(f"  {label} failed: {e}")
        return None


def _tmux_send(window, command):
    subprocess.run(
        ['tmux', 'send-keys', '-t', f'{TMUX_SESSION}:{window}', command, 'Enter'])


def _tmux_running():
    result = subprocess.run(
        f'tmux has-session -t {TMUX_SESSION} 2>/dev/null',
        shell=True)
    return result.returncode == 0


def _start_apriltag():
    global _apriltag_proc
    if _apriltag_proc and _apriltag_proc.poll() is None:
        print("  AprilTag already running")
        return
    cmd = (
        f'bash -c "'
        f'{FULL_SETUP} && '
        f'ros2 run apriltag_ros apriltag_node --ros-args '
        f'-r image_rect:=/zed/zed_node/rgb/color/rect/image '
        f'-r camera_info:='
        f'/zed/zed_node/left/color/rect/camera_info '
        f'-r detections:=/detections '
        f'--params-file \\"{TAGS_CONFIG}\\""'
    )
    _apriltag_proc = _run("AprilTag node", cmd)
    send_status(STATUS_TAG_RUNNING)


def _stop_apriltag():
    global _apriltag_proc
    if _apriltag_proc and _apriltag_proc.poll() is None:
        _apriltag_proc.terminate()
        _apriltag_proc = None
        print("  AprilTag stopped")
    subprocess.run('pkill -f apriltag_node 2>/dev/null', shell=True)
    send_status(STATUS_TAG_STOPPED)


# ═══════════════════════════════════════════════════════════════
# COMMAND HANDLERS
# ═══════════════════════════════════════════════════════════════
def handle_ping():
    print("  PING → sending PONG")
    send_status(STATUS_PONG)


def handle_launch(sim=False):
    mode = "SIM" if sim else "REAL"
    print(f"  LAUNCH ({mode}) — running launch_drone.sh")
    if not os.path.exists(LAUNCH_SCRIPT):
        print(f"  ERROR: {LAUNCH_SCRIPT} not found!")
        return
    flag = "--sim" if sim else ""
    subprocess.Popen(f'bash "{LAUNCH_SCRIPT}" {flag}', shell=True)
    time.sleep(1)
    send_status(STATUS_LAUNCHED_SIM if sim else STATUS_LAUNCHED)


def handle_stop_all():
    global _return_land_proc, _scan_proc, _fusion_proc, \
           _position_proc, _photo_proc, _video_proc
    print("  STOP ALL — killing drone tmux session + processes")
    _stop_apriltag()
    for proc, name in [
        (_return_land_proc, "return_land.py"),
        (_scan_proc,        "save_position.py"),
        (_fusion_proc,      "zed_fusion_scan.py"),
        (_position_proc,    "position tracking"),
        (_photo_proc,       "photo capture"),
        (_video_proc,       "video recording"),
    ]:
        if proc and proc.poll() is None:
            proc.terminate()
            print(f"  {name} stopped")
    _return_land_proc = _scan_proc = _fusion_proc = None
    _position_proc    = _photo_proc = _video_proc = None
    subprocess.run(f'tmux kill-session -t {TMUX_SESSION} 2>/dev/null', shell=True)
    print(f"  tmux session '{TMUX_SESSION}' killed")
    send_status(STATUS_ALL_STOPPED)


def handle_set_home():
    print("  SET HOME — writing trigger file for set_origin.py")
    if not _tmux_running():
        print("  ERROR: tmux session not running — launch first!")
        return
    trigger = f'{ZED_NAV_DIR}/.set_home_trigger'
    try:
        with open(trigger, 'w') as f:
            f.write(str(time.time()))
        print(f"  Trigger written -> {trigger}")
    except Exception as e:
        print(f"  ERROR writing trigger: {e}")
        return

    def _confirm():
        time.sleep(3)
        if os.path.exists(ORIGIN_FILE):
            print("  origin_T0.json confirmed ✓")
        else:
            print("  WARNING: origin_T0.json not found after 3s")
        send_status(STATUS_HOME_SET)

    threading.Thread(target=_confirm, daemon=True).start()


def handle_start_scan():
    print("  START SCAN — running save_position.py in tmux window 3")
    if not _tmux_running():
        print("  ERROR: tmux session not running — launch first!")
        return
    _tmux_send(3, f'{FULL_SETUP} && python3 "{SAVE_POSITION}"')
    time.sleep(2)
    send_status(STATUS_SCAN_STARTED)


def handle_start_3d_fusion():
    global _fusion_proc, _position_proc
    print("  START 3D FUSION")
    if not _tmux_running():
        print("  ERROR: tmux session not running — launch first!")
        return
    os.makedirs(SCANS_DIR, exist_ok=True)
    if not os.path.exists(ZED_FUSION):
        print(f"  ERROR: {ZED_FUSION} not found!")
        return
    _fusion_proc   = _run("ZED Fusion", f'python3 "{ZED_FUSION}"')
    time.sleep(1)
    _position_proc = _run("Position tracking",
        f'bash -c "{FULL_SETUP} && python3 \\"{SAVE_POSITION}\\""')
    if _fusion_proc and _position_proc:
        send_status(STATUS_3D_FUSION_STARTED)
    else:
        print("  ERROR: Failed to start one or both processes")


def handle_stop_3d_fusion():
    global _fusion_proc, _position_proc
    print("  STOP 3D FUSION")
    stopped = False
    for proc, name in [(_fusion_proc, "ZED Fusion"), (_position_proc, "Position tracking")]:
        if proc and proc.poll() is None:
            proc.terminate()
            proc.wait(timeout=5)
            stopped = True
            print(f"  {name} stopped")
    _fusion_proc = _position_proc = None
    if not stopped:
        print("  No fusion processes were running")
    send_status(STATUS_3D_FUSION_STOPPED)


def handle_start_photo(quality="HIGH"):
    global _photo_proc
    print(f"  START PHOTO CAPTURE — Quality: {quality}")
    if not _tmux_running():
        print("  ERROR: tmux session not running — launch first!")
        return
    os.makedirs(CAPTURES_DIR, exist_ok=True)
    if not os.path.exists(ZED_PHOTO):
        print(f"  ERROR: {ZED_PHOTO} not found!")
        return
    _photo_proc = _run(f"Photo capture ({quality})", f'python3 "{ZED_PHOTO}" {quality}')
    if _photo_proc:
        send_status(STATUS_PHOTO_STARTED)


def handle_start_video(quality="HIGH"):
    global _video_proc
    print(f"  START VIDEO RECORDING — Quality: {quality}")
    if not _tmux_running():
        print("  ERROR: tmux session not running — launch first!")
        return
    os.makedirs(CAPTURES_DIR, exist_ok=True)
    if not os.path.exists(ZED_VIDEO):
        print(f"  ERROR: {ZED_VIDEO} not found!")
        return
    _video_proc = _run(f"Video recording ({quality})", f'python3 "{ZED_VIDEO}" {quality}')
    if _video_proc:
        send_status(STATUS_VIDEO_STARTED)


def handle_stop_capture():
    global _photo_proc, _video_proc
    print("  STOP CAPTURE")
    stopped = False
    for proc, name in [(_photo_proc, "photo capture"), (_video_proc, "video recording")]:
        if proc and proc.poll() is None:
            proc.terminate()
            proc.wait(timeout=5)
            stopped = True
            print(f"  {name} stopped")
    _photo_proc = _video_proc = None
    if not stopped:
        print("  No capture processes were running")
    send_status(STATUS_CAPTURE_STOPPED)


def handle_set_gimbal(angle=0):
    print(f"  SET GIMBAL — Angle: {angle}°")
    try:
        import struct
        gimbal = serial.Serial(GIMBAL_PORT, GIMBAL_BAUD, timeout=1)
        time.sleep(0.1)
        cmd_id = 67
        for mode, pitch_val in [(2, int(angle / 0.02197265625)), (0, 0)]:
            data   = struct.pack('<Bhhhhhh', mode, 0, 0, 300 if mode == 2 else 0, pitch_val, 0, 0)
            size   = len(data)
            hchk   = (cmd_id + size) & 0xFF
            bchk   = sum(data) & 0xFF
            packet = bytes([0x3E, cmd_id, size, hchk]) + data + bytes([bchk])
            gimbal.write(packet)
            time.sleep(0.5)
        gimbal.close()
        print(f"  ✓ Gimbal set to {angle}°")
    except Exception as e:
        print(f"  ERROR: Gimbal control failed: {e}")
    send_status(STATUS_GIMBAL_SET)


def handle_save_end():
    print("  SAVE END — writing trigger file")
    def _snap():
        try:
            os.makedirs(os.path.dirname(TRIGGER_FILE), exist_ok=True)
            with open(TRIGGER_FILE, 'w') as f:
                f.write(str(time.time()))
            print(f"  Trigger written → {TRIGGER_FILE}")
            time.sleep(1)
            send_status(STATUS_END_SAVED)
        except Exception as e:
            print(f"  SAVE END failed: {e}")
    threading.Thread(target=_snap, daemon=True).start()


def handle_go_home():
    print("  GO HOME — starting return_land.py + AprilTag")
    if not _tmux_running():
        print("  ERROR: tmux session not running — launch first!")
        return
    if not os.path.exists(ORIGIN_FILE):
        print(f"  ERROR: {ORIGIN_FILE} not found — run SET HOME first!")
        return
    _tmux_send(1, (
        f'{FULL_SETUP} && ros2 run apriltag_ros apriltag_node --ros-args '
        f'-r image_rect:=/zed/zed_node/rgb/color/rect/image '
        f'-r camera_info:=/zed/zed_node/left/color/rect/camera_info '
        f'-r detections:=/detections '
        f'--params-file "{TAGS_CONFIG}"'
    ))
    time.sleep(2)
    _tmux_send(4, f'{FULL_SETUP} && python3 "{RETURN_LAND}"')
    send_status(STATUS_GOING_HOME)


def handle_start_tag():
    print("  START TAG")
    _start_apriltag()


def handle_stop_tag():
    print("  STOP TAG")
    _stop_apriltag()


def handle_land():
    global _return_land_proc
    print("  LAND — AprilTag + return_land.py")
    _start_apriltag()
    time.sleep(3)
    if not _tmux_running():
        print("  ERROR: tmux session not running!")
        return
    _tmux_send(4, f'{FULL_SETUP} && python3 "{RETURN_LAND}"')
    send_status(STATUS_LAND_ACK)


# ═══════════════════════════════════════════════════════════════
# COMMAND DISPATCHER
# ═══════════════════════════════════════════════════════════════
def dispatch(cmd_id: int):
    print(f"\n{'='*44}")
    print(f"  COMMAND: {cmd_id}")
    print(f"{'='*44}")

    sync_map = {
        CMD_ID_PING:     handle_ping,
        CMD_ID_STOP_TAG: handle_stop_tag,
        CMD_ID_STOP_ALL: handle_stop_all,
        CMD_ID_SAVE_END: handle_save_end,
    }

    thread_map = {
        CMD_ID_LAND:            handle_land,
        CMD_ID_START_TAG:       handle_start_tag,
        CMD_ID_LAUNCH:          handle_launch,
        CMD_ID_SET_HOME:        handle_set_home,
        CMD_ID_START_SCAN:      handle_start_scan,
        CMD_ID_START_3D_FUSION: handle_start_3d_fusion,
        CMD_ID_STOP_3D_FUSION:  handle_stop_3d_fusion,
        CMD_ID_START_PHOTO:     lambda: handle_start_photo("HIGH"),
        CMD_ID_START_VIDEO:     lambda: handle_start_video("HIGH"),
        CMD_ID_STOP_CAPTURE:    handle_stop_capture,
        CMD_ID_GO_HOME:         handle_go_home,
        CMD_ID_SET_GIMBAL:      lambda: handle_set_gimbal(0),
    }

    if cmd_id == CMD_ID_LAUNCH_SIM:
        threading.Thread(target=lambda: handle_launch(sim=True), daemon=True).start()
    elif cmd_id in sync_map:
        sync_map[cmd_id]()
    elif cmd_id in thread_map:
        threading.Thread(target=thread_map[cmd_id], daemon=True).start()
    else:
        print(f"  Unknown command ID: {cmd_id}")


# ═══════════════════════════════════════════════════════════════
# LORA RECEIVE LOOP
# ═══════════════════════════════════════════════════════════════
def lora_listen_loop(ser: serial.Serial):
    print("[LoRa] Listening for commands...")
    buf = ""

    while True:
        try:
            with _lora_lock:
                chunk = ser.read(ser.in_waiting or 1).decode(errors="ignore")

            buf += chunk

            while "\n" in buf:
                line, buf = buf.split("\n", 1)
                line = line.strip()

                if not line:
                    continue

                if line.startswith("+RCV="):
                    cmd_id = parse_rcv(line)
                    if cmd_id is not None:
                        dispatch(cmd_id)
                elif line.startswith(("+OK", "+ERR", "+READY")):
                    print(f"[LoRa] Module: {line}")
                else:
                    print(f"[LoRa] Unhandled: {line}")

        except Exception as e:
            print(f"[LoRa] Receive error: {e}")
            time.sleep(0.5)


# ═══════════════════════════════════════════════════════════════
# ENTRY POINT
# ═══════════════════════════════════════════════════════════════
if __name__ == '__main__':
    print("=" * 44)
    print("  CLARQ Command Listener")
    print("  Transport: RYLR998 LoRa (868/915 MHz)")
    print(f"  Port:      {LORA_PORT}  @  {LORA_BAUD} baud")
    print(f"  Address:   {LORA_ADDRESS}  →  dest {LORA_DEST_ADDR}")
    print(f"  Base path: {BASE}")
    print(f"  Gimbal:    {GIMBAL_PORT}")
    print("=" * 44)

    if subprocess.run('which tmux', shell=True, capture_output=True).returncode != 0:
        print("WARNING: tmux not found — sudo apt install tmux")

    # Kill any stale process holding the port
    subprocess.run(f"fuser -k {LORA_PORT} 2>/dev/null", shell=True)
    time.sleep(0.5)

    lora = init_lora()

    try:
        lora_listen_loop(lora)
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        if _lora and _lora.is_open:
            _lora.close()
        print("[LoRa] Port closed.")