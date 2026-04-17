#!/usr/bin/env python3
# ─────────────────────────────────────────────────────────────
# mavlink_trigger_listener.py  —  JETSON COMMAND LISTENER
#
# Intercepts MAVLink commands sent from the Windows GUI
# via two channels:
#
#   Channel 1 — NAMED_VALUE_INT (primary, most reliable)
#     /mavros/named_value/int
#     name = "CLARQ", value = command ID number
#     1 = PING
#     2 = LAND
#     3 = START_TAG
#     4 = STOP_TAG
#     5 = STOP_ALL
#     6 = LAUNCH (real hardware)
#     7 = LAUNCH_SIM
#
#   Channel 2 — STATUSTEXT (fallback)
#     /mavros/statustext/recv
#     text = "CLARQ_LAND", "CLARQ_PING" etc
#
# Signal route (both channels):
#   Windows CLARQ_GUI.py
#       ↓ MAVLink over UDP
#   Mission Planner
#       ↓ USB serial
#   Pixhawk
#       ↓ USB serial to Jetson
#   MAVROS
#       ↓
#   This file intercepts and runs bash commands
#
# Usage:
#   python3 mavlink_trigger_listener.py
# ─────────────────────────────────────────────────────────────

import os
import subprocess
import threading
import time

import rclpy
from rclpy.node import Node
from mavros_msgs.msg import StatusText
from mavros_msgs.msg import NamedValueInt

# ── Config ────────────────────────────────────────────────────
SCRIPT_DIR  = os.path.dirname(os.path.abspath(__file__))
WORKSPACE   = SCRIPT_DIR
FCU_URL     = '/dev/ttyACM0'   # Pixhawk USB — for direct MAVLink listener
FCU_BAUD    = 115200
TAGS_CONFIG = os.path.expanduser(
    '~/ZED Navigation/ZED-Camera-Navigation/tags_config.yaml')

ROS_SETUP  = 'source /opt/ros/humble/setup.bash'
ZED_SETUP  = 'source ~/ros2_ws/install/setup.bash'
FULL_SETUP = f'{ROS_SETUP} && {ZED_SETUP}'

# ── Command IDs — must match CLARQ_GUI.py ─────────────────────
CMD_ID_PING       = 1
CMD_ID_LAND       = 2
CMD_ID_START_TAG  = 3
CMD_ID_STOP_TAG   = 4
CMD_ID_STOP_ALL   = 5
CMD_ID_LAUNCH     = 6
CMD_ID_LAUNCH_SIM = 7

# ── Command strings — for STATUSTEXT fallback ─────────────────
CMD_PING       = "CLARQ_PING"
CMD_LAND       = "CLARQ_LAND"
CMD_START_TAG  = "CLARQ_START_TAG"
CMD_STOP_TAG   = "CLARQ_STOP_TAG"
CMD_STOP_ALL   = "CLARQ_STOP_ALL"
CMD_LAUNCH     = "CLARQ_LAUNCH"
CMD_LAUNCH_SIM = "CLARQ_LAUNCH_SIM"


class TriggerListener(Node):
    def __init__(self):
        super().__init__('mavlink_trigger_listener')

        self._apriltag_proc    = None
        self._return_land_proc = None

        # ── Channel 1: NAMED_VALUE_INT (primary) ──────────────
        # Most reliable — passes through Pixhawk guaranteed
        self.create_subscription(
            NamedValueInt,
            '/mavros/named_value/int',
            self._named_value_cb,
            10)

        # ── Channel 2: STATUSTEXT (fallback) ──────────────────
        self.create_subscription(
            StatusText,
            '/mavros/statustext/recv',
            self._statustext_cb,
            10)

        # Publisher to send status back to GUI
        self._status_pub = self.create_publisher(
            StatusText,
            '/mavros/statustext/send',
            10)

        # ── Channel 0: Direct pymavlink on serial port ────────
        # Catches COMMAND_LONG MAV_CMD_USER_1 and NAMED_VALUE_INT
        # directly from Pixhawk USB — does not depend on MAVROS
        self._fcu_mav = None
        threading.Thread(
            target=self._pymavlink_listener,
            daemon=True).start()

        self.get_logger().info('=' * 44)
        self.get_logger().info('  MAVLink Trigger Listener started')
        self.get_logger().info('=' * 44)
        self.get_logger().info(
            'Channel 0: pymavlink direct on ' + FCU_URL)
        self.get_logger().info(
            'Channel 1: /mavros/named_value/int (MAVROS)')
        self.get_logger().info(
            'Channel 2: /mavros/statustext/recv (MAVROS)')
        self.get_logger().info(
            'Workspace: ' + WORKSPACE)
        self.get_logger().info(
            'Waiting for GUI commands...')

    # ── Channel 0: Direct pymavlink listener ─────────────────
    def _pymavlink_listener(self):
        """
        Direct MAVLink connection to Pixhawk on USB serial.
        Catches COMMAND_LONG MAV_CMD_USER_1 sent from GUI.
        Runs in a background thread independently of MAVROS.
        """
        import time as _time
        while True:
            try:
                self.get_logger().info(
                    f'Connecting pymavlink on {FCU_URL}:{FCU_BAUD}')
                self._fcu_mav = mavutil.mavlink_connection(
                    FCU_URL, baud=FCU_BAUD)
                self._fcu_mav.wait_heartbeat(timeout=10)
                self.get_logger().info(
                    'pymavlink connected to Pixhawk!')

                while True:
                    msg = self._fcu_mav.recv_match(
                        type=['COMMAND_LONG', 'NAMED_VALUE_INT'],
                        blocking=True,
                        timeout=2)
                    if msg is None:
                        continue

                    t = msg.get_type()

                    if t == 'COMMAND_LONG':
                        # MAV_CMD_USER_1 = 31010
                        if msg.command == 31010:
                            cmd_id = int(msg.param1)
                            self.get_logger().info(
                                f'COMMAND_LONG MAV_CMD_USER_1 '
                                f'cmd_id={cmd_id}')
                            self._dispatch_cmd_id(cmd_id)

                    elif t == 'NAMED_VALUE_INT':
                        name = msg.name.decode(
                            'utf-8', errors='ignore'
                        ).strip().rstrip('\x00')
                        if name == 'CLARQ':
                            cmd_id = msg.value
                            self.get_logger().info(
                                f'pymavlink NAMED_VALUE_INT '
                                f'CLARQ={cmd_id}')
                            self._dispatch_cmd_id(cmd_id)

            except Exception as e:
                self.get_logger().warn(
                    f'pymavlink error: {e} — retrying in 5s')
                _time.sleep(5)

    def _dispatch_cmd_id(self, cmd_id):
        """Dispatch a command ID to the correct handler."""
        if cmd_id == CMD_ID_PING:
            self._handle_ping()
        elif cmd_id == CMD_ID_LAND:
            threading.Thread(
                target=self._handle_land,
                daemon=True).start()
        elif cmd_id == CMD_ID_START_TAG:
            threading.Thread(
                target=self._start_apriltag,
                daemon=True).start()
        elif cmd_id == CMD_ID_STOP_TAG:
            self._stop_apriltag()
        elif cmd_id == CMD_ID_STOP_ALL:
            self._stop_all()
        elif cmd_id == CMD_ID_LAUNCH:
            threading.Thread(
                target=self._launch_drone,
                daemon=True).start()
        elif cmd_id == CMD_ID_LAUNCH_SIM:
            threading.Thread(
                target=lambda: self._launch_drone(sim=True),
                daemon=True).start()
        else:
            self.get_logger().warn(
                f'Unknown command ID: {cmd_id}')

    # ── Channel 1: NAMED_VALUE_INT callback ───────────────────
    def _named_value_cb(self, msg):
        """
        Primary channel — intercepts NAMED_VALUE_INT messages.
        GUI sends name='CLARQ', value=command ID.
        """
        name = msg.name.strip().rstrip('\x00')

        if name != 'CLARQ':
            return

        cmd_id = msg.value
        self.get_logger().info(
            f'NAMED_VALUE_INT received: CLARQ={cmd_id}')

        if cmd_id == CMD_ID_PING:
            self._handle_ping()
        elif cmd_id == CMD_ID_LAND:
            threading.Thread(
                target=self._handle_land,
                daemon=True).start()
        elif cmd_id == CMD_ID_START_TAG:
            threading.Thread(
                target=self._start_apriltag,
                daemon=True).start()
        elif cmd_id == CMD_ID_STOP_TAG:
            self._stop_apriltag()
        elif cmd_id == CMD_ID_STOP_ALL:
            self._stop_all()
        elif cmd_id == CMD_ID_LAUNCH:
            threading.Thread(
                target=self._launch_drone,
                daemon=True).start()
        elif cmd_id == CMD_ID_LAUNCH_SIM:
            threading.Thread(
                target=lambda: self._launch_drone(sim=True),
                daemon=True).start()
        else:
            self.get_logger().warn(
                f'Unknown command ID: {cmd_id}')

    # ── Channel 2: STATUSTEXT callback ────────────────────────
    def _statustext_cb(self, msg):
        """
        Fallback channel — intercepts STATUSTEXT messages.
        GUI sends text='CLARQ_LAND' etc.
        """
        text = msg.text.strip()

        if not text.startswith('CLARQ'):
            return

        self.get_logger().info(
            f'STATUSTEXT received: {text}')

        if text == CMD_PING:
            self._handle_ping()
        elif text == CMD_LAND:
            threading.Thread(
                target=self._handle_land,
                daemon=True).start()
        elif text == CMD_START_TAG:
            threading.Thread(
                target=self._start_apriltag,
                daemon=True).start()
        elif text == CMD_STOP_TAG:
            self._stop_apriltag()
        elif text == CMD_STOP_ALL:
            self._stop_all()
        elif text == CMD_LAUNCH:
            threading.Thread(
                target=self._launch_drone,
                daemon=True).start()
        elif text == CMD_LAUNCH_SIM:
            threading.Thread(
                target=lambda: self._launch_drone(sim=True),
                daemon=True).start()

    # ── Ping ──────────────────────────────────────────────────
    def _handle_ping(self):
        self.get_logger().info('Ping — sending pong')
        self._send_status('CLARQ_PONG')

    # ── Full landing sequence ─────────────────────────────────
    def _handle_land(self):
        """Start apriltag then return_land.py"""
        self.get_logger().info(
            'CLARQ_LAND — starting landing sequence')
        self._start_apriltag()
        time.sleep(3)
        self._start_return_land()

    # ── Start apriltag_ros ────────────────────────────────────
    def _start_apriltag(self):
        if (self._apriltag_proc is not None and
                self._apriltag_proc.poll() is None):
            self.get_logger().info('AprilTag already running')
            self._send_status('CLARQ_TAG_ALREADY_RUNNING')
            return

        self.get_logger().info('Starting apriltag_ros...')
        self._send_status('CLARQ_TAG_STARTING')

        cmd = (
            f'{FULL_SETUP} && '
            f'ros2 run apriltag_ros apriltag_node --ros-args '
            f'-r image_rect:='
            f'/zed/zed_node/rgb/color/rect/image '
            f'-r camera_info:='
            f'/zed/zed_node/rgb/color/rect/camera_info '
            f'--params-file {TAGS_CONFIG}'
        )

        self._apriltag_proc = subprocess.Popen(
            ['bash', '-c', cmd],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE
        )

        time.sleep(2)
        if self._apriltag_proc.poll() is None:
            self.get_logger().info('AprilTag running!')
            self._send_status('CLARQ_TAG_RUNNING')
        else:
            err = self._apriltag_proc.stderr.read().decode()
            self.get_logger().error(f'AprilTag failed: {err}')
            self._send_status('CLARQ_TAG_FAILED')

    # ── Stop apriltag_ros ─────────────────────────────────────
    def _stop_apriltag(self):
        if (self._apriltag_proc is not None and
                self._apriltag_proc.poll() is None):
            self._apriltag_proc.terminate()
            self._apriltag_proc = None
            self.get_logger().info('AprilTag stopped')
            self._send_status('CLARQ_TAG_STOPPED')
        else:
            self.get_logger().warn('AprilTag not running')

    # ── Start return_land.py ──────────────────────────────────
    def _start_return_land(self):
        if (self._return_land_proc is not None and
                self._return_land_proc.poll() is None):
            self.get_logger().info('return_land.py already running')
            self._send_status('CLARQ_LAND_ALREADY_RUNNING')
            return

        self.get_logger().info('Starting return_land.py...')
        self._send_status('CLARQ_LAND_STARTING')

        script = os.path.join(WORKSPACE, 'return_land.py')
        if not os.path.exists(script):
            self.get_logger().error(
                f'return_land.py not found at {script}')
            self._send_status('CLARQ_LAND_NOT_FOUND')
            return

        cmd = (
            f'{FULL_SETUP} && '
            f'cd "{WORKSPACE}" && '
            f'python3 return_land.py'
        )

        self._return_land_proc = subprocess.Popen(
            ['bash', '-c', cmd],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE
        )

        time.sleep(2)
        if self._return_land_proc.poll() is None:
            self.get_logger().info('return_land.py running!')
            self._send_status('CLARQ_LAND_RUNNING')
        else:
            err = self._return_land_proc.stderr.read().decode()
            self.get_logger().error(
                f'return_land.py failed: {err}')
            self._send_status('CLARQ_LAND_FAILED')

    # ── Stop everything ───────────────────────────────────────
    def _stop_all(self):
        self.get_logger().info('Stopping all...')
        self._stop_apriltag()
        if (self._return_land_proc is not None and
                self._return_land_proc.poll() is None):
            self._return_land_proc.terminate()
            self._return_land_proc = None
            self.get_logger().info('return_land.py stopped')
        self._send_status('CLARQ_ALL_STOPPED')

    # ── Launch drone stack ────────────────────────────────────
    def _launch_drone(self, sim=False):
        """
        Run launch_drone.sh on the Jetson.
        Called when GUI sends CLARQ_LAUNCH or CLARQ_LAUNCH_SIM.
        Kills any existing tmux session first then starts fresh.
        """
        flag = '--sim' if sim else ''
        mode = 'SIM' if sim else 'REAL'
        self.get_logger().info(
            f'Launching drone stack — {mode} mode...')
        self._send_status(
            'CLARQ_LAUNCHING_SIM' if sim
            else 'CLARQ_LAUNCHING')

        script = os.path.join(WORKSPACE, 'launch_drone.sh')
        if not os.path.exists(script):
            self.get_logger().error(
                f'launch_drone.sh not found at {script}')
            self._send_status('CLARQ_LAUNCH_NOT_FOUND')
            return

        cmd = (
            f'cd "{WORKSPACE}" && '
            f'tmux kill-session -t drone 2>/dev/null; '
            f'sleep 0.5 && '
            f'bash launch_drone.sh {flag}'
        )

        subprocess.Popen(
            ['bash', '-c', cmd],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL
        )

        time.sleep(2)
        self.get_logger().info(
            f'launch_drone.sh started — {mode} mode')
        self._send_status(
            'CLARQ_LAUNCHED_SIM' if sim
            else 'CLARQ_LAUNCHED')

    # ── Send status back to GUI ───────────────────────────────
    def _send_status(self, text):
        msg          = StatusText()
        msg.severity = 6
        msg.text     = text
        self._status_pub.publish(msg)
        self.get_logger().info(f'Status sent: {text}')

    # ── Cleanup ───────────────────────────────────────────────
    def destroy_node(self):
        self.get_logger().info('Shutting down...')
        self._stop_all()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TriggerListener()

    print('')
    print('=' * 44)
    print('  MAVLink Trigger Listener running')
    print('  Channel 1: NAMED_VALUE_INT (primary)')
    print('  Channel 2: STATUSTEXT (fallback)')
    print('  Waiting for commands from GUI...')
    print('=' * 44)
    print('')

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\nStopped by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
