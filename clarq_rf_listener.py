import json
import os
import subprocess
import sys
import threading
import time

# Force unbuffered output for systemd journal / SSH viewing
os.environ['PYTHONUNBUFFERED'] = '1'
sys.stdout.reconfigure(line_buffering=True)
sys.stderr.reconfigure(line_buffering=True)

# ── nRF24L01 ──────────────────────────────────────────────────
from pyrf24 import RF24, RF24_PA_MAX, RF24_250KBPS, RF24_CRC_16

# Must match clarq_rf_arduino.ino exactly
RF_CHANNEL  = 100
READ_PIPE   = b"CLARQ"   # Arduino → Jetson (commands in)
WRITE_PIPE  = b"JTSN0"   # Jetson → Arduino (ACK/status out)

CE_PIN  = 18             # BCM GPIO pin
SPI_BUS = 0              # SPI bus (0 or 1 depending on Jetson)
SPI_DEV = 0              # SPI device (CS)

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

# ── Status response IDs (sent back to GUI via nRF) ────────────
# Must match ARDUINO_RX_MAP in CLARQ_GUI.py
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

# ── Gimbal serial port (Jetson → SimpleBGC controller) ────────
GIMBAL_PORT = "/dev/ttyACM2"   # adjust if port differs on your Jetson
GIMBAL_BAUD = 115200

# ── ROS/ZED setup ─────────────────────────────────────────────
ROS_SETUP  = 'source /opt/ros/humble/setup.bash'
ZED_SETUP  = 'source ~/ros2_ws/install/setup.bash'
FULL_SETUP = f'{ROS_SETUP} && {ZED_SETUP}'

# tmux session name (must match launch_drone.sh)
TMUX_SESSION = 'drone'

# ── Process handles ───────────────────────────────────────────
_apriltag_proc    = None
_return_land_proc = None
_scan_proc        = None
_fusion_proc      = None
_position_proc    = None
_photo_proc       = None
_video_proc       = None

# ── nRF radio handle ──────────────────────────────────────────
_radio = None


# ═══════════════════════════════════════════════════════════════
# NRF24 INIT
# ═══════════════════════════════════════════════════════════════
def init_radio():
    global _radio
    print("[nRF] Initializing NRF24L01...")
    radio = RF24(CE_PIN, SPI_BUS * 10 + SPI_DEV)

    if not radio.begin():
        print("[nRF] ERROR: NRF24L01 not found — check wiring!")
        sys.exit(1)

    radio.setChannel(RF_CHANNEL)
    radio.setPALevel(RF24_PA_MAX)
    radio.setDataRate(RF24_250KBPS)
    radio.setCRCLength(RF24_CRC_16)
    radio.setAutoAck(True)
    radio.setRetries(5, 15)
    radio.openReadingPipe(1, READ_PIPE)

    # Use new pyrf24 API — pass TX address directly to stopListening()
    # Replaces the deprecated openWritingPipe() call
    radio.stopListening(WRITE_PIPE)
    radio.startListening()

    _radio = radio
    print(f"[nRF] Ready — listening on channel {RF_CHANNEL} ✓")
    return radio


# ═══════════════════════════════════════════════════════════════
# SEND STATUS BACK TO GUI
# ═══════════════════════════════════════════════════════════════
def send_status(status_id):
    """
    Send a 2-byte response packet back to Arduino → GUI.
    Packet format matches clarq_rf_arduino.ino:
      byte 0: status_id
      byte 1: checksum (status_id XOR 0xAA)
    """
    global _radio
    if _radio is None:
        return
    try:
        checksum = status_id ^ 0xAA
        payload  = bytes([status_id, checksum])

        # Use new pyrf24 API — pass TX address directly to stopListening()
        _radio.stopListening(WRITE_PIPE)
        time.sleep(0.01)

        for attempt in range(3):
            ok = _radio.write(payload)
            if ok:
                print(f"[nRF] Status sent: id={status_id} ✓")
                break
            if attempt < 2:
                time.sleep(0.01)
        else:
            print(f"[nRF] Status send failed: id={status_id}")

        time.sleep(0.01)
        _radio.startListening()

    except Exception as e:
        print(f"[nRF] send_status error: {e}")
        try:
            _radio.startListening()
        except:
            pass


# ═══════════════════════════════════════════════════════════════
# HELPERS
# ═══════════════════════════════════════════════════════════════
def _run(label, cmd):
    """Launch a shell command, return the Popen handle."""
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
    """Send a command to a specific tmux window."""
    subprocess.run(
        ['tmux', 'send-keys', '-t', f'{TMUX_SESSION}:{window}', command, 'Enter'])


def _tmux_running():
    """Check if the drone tmux session is alive."""
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
    """Confirms Jetson is alive."""
    print("  PING → sending PONG")
    send_status(STATUS_PONG)


def handle_launch(sim=False):
    """Runs launch_drone.sh to create the tmux session."""
    mode = "SIM" if sim else "REAL"
    print(f"  LAUNCH ({mode}) — running launch_drone.sh")

    if not os.path.exists(LAUNCH_SCRIPT):
        print(f"  ERROR: {LAUNCH_SCRIPT} not found!")
        return

    flag = "--sim" if sim else ""
    subprocess.Popen(f'bash "{LAUNCH_SCRIPT}" {flag}', shell=True)

    time.sleep(1)
    status = STATUS_LAUNCHED_SIM if sim else STATUS_LAUNCHED
    send_status(status)


def handle_stop_all():
    """Kills the tmux session and all CLARQ processes."""
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

    _return_land_proc = None
    _scan_proc        = None
    _fusion_proc      = None
    _position_proc    = None
    _photo_proc       = None
    _video_proc       = None

    subprocess.run(
        f'tmux kill-session -t {TMUX_SESSION} 2>/dev/null',
        shell=True)
    print(f"  tmux session '{TMUX_SESSION}' killed")

    send_status(STATUS_ALL_STOPPED)


def handle_set_home():
    """
    Writes a trigger file that set_origin.py watches for.
    set_origin.py detects the file, saves current ZED pose
    as origin_T0.json, then deletes the trigger file.
    """
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
    """
    Sends save_position.py command to tmux window 3.
    Legacy position-only tracking (no 3D reconstruction).
    """
    print("  START SCAN — running save_position.py in tmux window 3")

    if not _tmux_running():
        print("  ERROR: tmux session not running — launch first!")
        return

    _tmux_send(3, f'{FULL_SETUP} && python3 "{SAVE_POSITION}"')
    print("  save_position.py started in tmux window 3")

    time.sleep(2)
    send_status(STATUS_SCAN_STARTED)


def handle_start_3d_fusion():
    """
    Starts ZED Fusion 3D reconstruction + position tracking.
    Records to .SVO file and tracks drone position simultaneously.
    """
    global _fusion_proc, _position_proc
    print("  START 3D FUSION — launching ZED Fusion + position tracking")

    if not _tmux_running():
        print("  ERROR: tmux session not running — launch first!")
        return

    os.makedirs(SCANS_DIR, exist_ok=True)

    if not os.path.exists(ZED_FUSION):
        print(f"  ERROR: {ZED_FUSION} not found!")
        return

    print(f"  Starting ZED Fusion: {ZED_FUSION}")
    _fusion_proc = _run("ZED Fusion", f'python3 "{ZED_FUSION}"')

    time.sleep(1)

    print(f"  Starting position tracking: {SAVE_POSITION}")
    _position_proc = _run(
        "Position tracking",
        f'bash -c "{FULL_SETUP} && python3 \\"{SAVE_POSITION}\\""'
    )

    if _fusion_proc and _position_proc:
        print("  ✓ 3D Fusion + position tracking active")
        send_status(STATUS_3D_FUSION_STARTED)
    else:
        print("  ERROR: Failed to start one or both processes")


def handle_stop_3d_fusion():
    """Stops ZED Fusion recording and position tracking."""
    global _fusion_proc, _position_proc
    print("  STOP 3D FUSION — terminating processes")

    stopped = False

    if _fusion_proc and _fusion_proc.poll() is None:
        print("  Stopping ZED Fusion...")
        _fusion_proc.terminate()
        _fusion_proc.wait(timeout=5)
        _fusion_proc = None
        stopped = True

    if _position_proc and _position_proc.poll() is None:
        print("  Stopping position tracking...")
        _position_proc.terminate()
        _position_proc.wait(timeout=5)
        _position_proc = None
        stopped = True

    if stopped:
        print("  ✓ 3D Fusion recording saved")
    else:
        print("  No fusion processes were running")

    send_status(STATUS_3D_FUSION_STOPPED)


def handle_start_photo(quality="HIGH"):
    """Starts photo capture at the specified quality setting."""
    global _photo_proc
    print(f"  START PHOTO CAPTURE — Quality: {quality}")

    if not _tmux_running():
        print("  ERROR: tmux session not running — launch first!")
        return

    os.makedirs(CAPTURES_DIR, exist_ok=True)

    if not os.path.exists(ZED_PHOTO):
        print(f"  ERROR: {ZED_PHOTO} not found!")
        return

    _photo_proc = _run(
        f"Photo capture ({quality})",
        f'python3 "{ZED_PHOTO}" {quality}'
    )

    if _photo_proc:
        print(f"  ✓ Photo capture active ({quality})")
        send_status(STATUS_PHOTO_STARTED)
    else:
        print("  ERROR: Failed to start photo capture")


def handle_start_video(quality="HIGH"):
    """Starts video recording at the specified quality setting."""
    global _video_proc
    print(f"  START VIDEO RECORDING — Quality: {quality}")

    if not _tmux_running():
        print("  ERROR: tmux session not running — launch first!")
        return

    os.makedirs(CAPTURES_DIR, exist_ok=True)

    if not os.path.exists(ZED_VIDEO):
        print(f"  ERROR: {ZED_VIDEO} not found!")
        return

    _video_proc = _run(
        f"Video recording ({quality})",
        f'python3 "{ZED_VIDEO}" {quality}'
    )

    if _video_proc:
        print(f"  ✓ Video recording active ({quality})")
        send_status(STATUS_VIDEO_STARTED)
    else:
        print("  ERROR: Failed to start video recording")


def handle_stop_capture():
    """Stops photo or video capture and saves files."""
    global _photo_proc, _video_proc
    print("  STOP CAPTURE — terminating processes")

    stopped = False

    if _photo_proc and _photo_proc.poll() is None:
        print("  Stopping photo capture...")
        _photo_proc.terminate()
        _photo_proc.wait(timeout=5)
        _photo_proc = None
        stopped = True

    if _video_proc and _video_proc.poll() is None:
        print("  Stopping video recording...")
        _video_proc.terminate()
        _video_proc.wait(timeout=5)
        _video_proc = None
        stopped = True

    if stopped:
        print("  ✓ Capture stopped, files saved")
    else:
        print("  No capture processes were running")

    send_status(STATUS_CAPTURE_STOPPED)


def handle_set_gimbal(angle=0):
    """
    Sets gimbal pitch angle via serial to SimpleBGC controller.
    Jetson → /dev/ttyACM2 → SimpleBGC → gimbal motor.
    """
    print(f"  SET GIMBAL — Angle: {angle}°")

    try:
        import serial
        import struct

        gimbal = serial.Serial(GIMBAL_PORT, GIMBAL_BAUD, timeout=1)
        time.sleep(0.1)

        # CMD_CONTROL (id=67), Mode 2: absolute angle
        cmd_id = 67
        mode   = 2
        pitch  = int(angle / 0.02197265625)
        data   = struct.pack('<Bhhhhhh', mode, 0, 0, 300, pitch, 0, 0)

        size            = len(data)
        header_checksum = (cmd_id + size) & 0xFF
        body_checksum   = sum(data) & 0xFF
        packet = (bytes([0x3E, cmd_id, size, header_checksum])
                  + data + bytes([body_checksum]))

        gimbal.write(packet)
        time.sleep(0.5)

        # Mode 0: release control (allows drift correction)
        mode = 0
        data = struct.pack('<Bhhhhhh', mode, 0, 0, 0, 0, 0, 0)
        size            = len(data)
        header_checksum = (cmd_id + size) & 0xFF
        body_checksum   = sum(data) & 0xFF
        packet = (bytes([0x3E, cmd_id, size, header_checksum])
                  + data + bytes([body_checksum]))

        gimbal.write(packet)
        gimbal.close()

        print(f"  ✓ Gimbal set to {angle}°")
        send_status(STATUS_GIMBAL_SET)

    except Exception as e:
        print(f"  ERROR: Gimbal control failed: {e}")
        print(f"  Check that SimpleBGC is connected on {GIMBAL_PORT}")
        send_status(STATUS_GIMBAL_SET)


def handle_save_end():
    """
    Writes a trigger file that save_position.py watches for.
    When detected, save_position.py snapshots the current
    position as scan_end_pos.json and stops recording.
    """
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
    """
    Runs return_land.py in tmux window 4 and starts AprilTag.
    Requires origin_T0.json to exist (SET HOME must have run first).

    return_land.py phases:
      Phase 1 — ZED navigate back to origin_T0 position
      Phase 2 — AprilTag align at 90° above tag
      Phase 3 — Land on rover
    """
    print("  GO HOME — starting return_land.py + AprilTag")

    if not _tmux_running():
        print("  ERROR: tmux session not running — launch first!")
        return

    if not os.path.exists(ORIGIN_FILE):
        print(f"  ERROR: {ORIGIN_FILE} not found!")
        print("  Run SET HOME POINT first!")
        return

    # Start AprilTag in tmux window 1
    _tmux_send(1, (
        f'{FULL_SETUP} && ros2 run apriltag_ros apriltag_node --ros-args '
        f'-r image_rect:=/zed/zed_node/rgb/color/rect/image '
        f'-r camera_info:=/zed/zed_node/left/color/rect/camera_info '
        f'-r detections:=/detections '
        f'--params-file "{TAGS_CONFIG}"'
    ))
    print("  AprilTag started in tmux window 1")

    time.sleep(2)

    # Start return_land.py in tmux window 4
    _tmux_send(4, f'{FULL_SETUP} && python3 "{RETURN_LAND}"')
    print("  return_land.py started in tmux window 4")

    send_status(STATUS_GOING_HOME)


def handle_start_tag():
    """Start AprilTag node standalone."""
    print("  START TAG")
    _start_apriltag()


def handle_stop_tag():
    """Stop AprilTag node."""
    print("  STOP TAG")
    _stop_apriltag()


def handle_land():
    """
    Shortcut — starts AprilTag and return_land immediately.
    Skips ZED Phase 1 navigation, goes straight to AprilTag alignment.
    """
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
def dispatch(cmd_id):
    print(f"\n{'='*44}")
    print(f"  COMMAND: {cmd_id}")
    print(f"{'='*44}")

    # Run on main thread (fast, non-blocking)
    sync_map = {
        CMD_ID_PING:     handle_ping,
        CMD_ID_STOP_TAG: handle_stop_tag,
        CMD_ID_STOP_ALL: handle_stop_all,
        CMD_ID_SAVE_END: handle_save_end,
    }

    # Run in background thread (may take time)
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
        threading.Thread(
            target=lambda: handle_launch(sim=True),
            daemon=True).start()
    elif cmd_id in sync_map:
        sync_map[cmd_id]()
    elif cmd_id in thread_map:
        threading.Thread(
            target=thread_map[cmd_id],
            daemon=True).start()
    else:
        print(f"  Unknown command ID: {cmd_id}")


# ═══════════════════════════════════════════════════════════════
# NRF24 RECEIVE LOOP
# ═══════════════════════════════════════════════════════════════
def rf_listen_loop(radio):
    """
    Main loop — polls NRF24L01 for incoming packets.
    Packet format (2 bytes, matches clarq_rf_arduino.ino):
      byte 0: cmd_id
      byte 1: checksum (cmd_id XOR 0xAA)
    """
    print("[nRF] Listening for commands...")
    while True:
        try:
            if radio.available():
                payload = radio.read(2)

                if len(payload) < 2:
                    print("[nRF] Short packet — ignored")
                    continue

                cmd_id   = payload[0]
                checksum = payload[1]

                # Validate checksum
                if checksum != (cmd_id ^ 0xAA):
                    print(f"[nRF] Checksum fail — "
                          f"got {checksum:#x} "
                          f"expected {cmd_id ^ 0xAA:#x}")
                    continue

                if not 1 <= cmd_id <= 17:
                    print(f"[nRF] Out of range cmd_id: {cmd_id}")
                    continue

                dispatch(cmd_id)

            else:
                time.sleep(0.01)

        except Exception as e:
            print(f"[nRF] Receive error: {e}")
            time.sleep(0.5)


# ═══════════════════════════════════════════════════════════════
# ENTRY POINT
# ═══════════════════════════════════════════════════════════════
if __name__ == '__main__':
    print("=" * 44)
    print("  CLARQ Command Listener")
    print("  Transport: NRF24L01")
    print(f"  Base path: {BASE}")
    print(f"  Gimbal port: {GIMBAL_PORT}")
    print("=" * 44)

    if subprocess.run('which tmux', shell=True,
                      capture_output=True).returncode != 0:
        print("WARNING: tmux not found — install with: sudo apt install tmux")

    radio = init_radio()

    try:
        rf_listen_loop(radio)
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        radio.powerDown()
        print("Radio powered down.")
