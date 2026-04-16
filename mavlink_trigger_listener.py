#!/usr/bin/env python3
# ─────────────────────────────────────────────────────────────
# mavlink_trigger_listener.py  —  JETSON COMMAND LISTENER
#
# Listens for trigger commands sent from the Windows GUI
# via MAVLink STATUSTEXT messages.
#
# Signal route:
#   Windows CLARQ_GUI.py
#       ↓ MAVLink STATUSTEXT over UDP
#   Mission Planner
#       ↓ forwards to Pixhawk via USB
#   Pixhawk
#       ↓ USB serial to Jetson
#   MAVROS → /mavros/statustext/recv
#       ↓
#   This file receives and acts on the command
#
# Commands recognised:
#   CLARQ_PING        — responds with CLARQ_PONG (connection test)
#   CLARQ_LAND        — starts full landing sequence:
#                         1. starts apriltag_ros node
#                         2. starts return_land.py
#   CLARQ_START_TAG   — starts apriltag_ros node only
#   CLARQ_STOP_TAG    — stops apriltag_ros node
#   CLARQ_STOP_ALL    — stops apriltag and return_land
#
# Usage:
#   python3 mavlink_trigger_listener.py
#
# Add to launch_drone.sh window 6 so it starts automatically.
# Requires MAVROS to be running first.
# ─────────────────────────────────────────────────────────────

import os
import subprocess
import threading
import time

import rclpy
from rclpy.node import Node
from mavros_msgs.msg import StatusText

# ── Config ────────────────────────────────────────────────────
SCRIPT_DIR  = os.path.dirname(os.path.abspath(__file__))
WORKSPACE   = SCRIPT_DIR
TAGS_CONFIG = os.path.expanduser(
    '~/ZED Navigation/ZED-Camera-Navigation/tags_config.yaml')

ROS_SETUP   = 'source /opt/ros/humble/setup.bash'
ZED_SETUP   = 'source ~/ros2_ws/install/setup.bash'
FULL_SETUP  = f'{ROS_SETUP} && {ZED_SETUP}'

# ── Command keywords — must match CLARQ_GUI.py ────────────────
CMD_PING       = "CLARQ_PING"
CMD_LAND       = "CLARQ_LAND"
CMD_START_TAG  = "CLARQ_START_TAG"
CMD_STOP_TAG   = "CLARQ_STOP_TAG"
CMD_STOP_ALL   = "CLARQ_STOP_ALL"

class TriggerListener(Node):
    def __init__(self):
        super().__init__('mavlink_trigger_listener')

        # Track running subprocesses
        self._apriltag_proc   = None
        self._return_land_proc = None

        # Subscribe to MAVROS statustext
        # Messages arrive here after travelling:
        # GUI → Mission Planner → Pixhawk → MAVROS
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

        self.get_logger().info('=' * 44)
        self.get_logger().info('  MAVLink Trigger Listener started')
        self.get_logger().info('=' * 44)
        self.get_logger().info(
            'Listening on /mavros/statustext/recv')
        self.get_logger().info(
            f'Commands: {CMD_PING} | {CMD_LAND} | '
            f'{CMD_START_TAG} | {CMD_STOP_TAG} | '
            f'{CMD_STOP_ALL}')
        self.get_logger().info(
            'Workspace: ' + WORKSPACE)

    # ── Statustext callback ───────────────────────────────────
    def _statustext_cb(self, msg):
        text = msg.text.strip()

        # Filter out non-CLARQ messages to avoid noise
        if not text.startswith("CLARQ"):
            return

        self.get_logger().info(f'Command received: {text}')

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

    # ── Ping handler ──────────────────────────────────────────
    def _handle_ping(self):
        self.get_logger().info('Ping received — sending pong')
        self._send_status("CLARQ_PONG")

    # ── Full landing sequence ─────────────────────────────────
    def _handle_land(self):
        """
        Full landing sequence triggered by GUI START PREC LAND:
        1. Start apriltag_ros node
        2. Start return_land.py
        Both run as background processes.
        """
        self.get_logger().info(
            'CLARQ_LAND received — starting landing sequence')

        # Step 1 — start apriltag node
        self._start_apriltag()

        # Wait for apriltag to initialize
        time.sleep(3)

        # Step 2 — start return_land.py
        self._start_return_land()

    # ── Start apriltag_ros ────────────────────────────────────
    def _start_apriltag(self):
        """Start apriltag_ros node if not already running"""
        if (self._apriltag_proc is not None and
                self._apriltag_proc.poll() is None):
            self.get_logger().info(
                'AprilTag node already running')
            self._send_status("CLARQ_TAG_ALREADY_RUNNING")
            return

        self.get_logger().info('Starting apriltag_ros node...')
        self._send_status("CLARQ_TAG_STARTING")

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

        # Wait and confirm
        time.sleep(2)
        if self._apriltag_proc.poll() is None:
            self.get_logger().info(
                'AprilTag node running!')
            self._send_status("CLARQ_TAG_RUNNING")
        else:
            stderr = self._apriltag_proc.stderr.read().decode()
            self.get_logger().error(
                f'AprilTag failed to start: {stderr}')
            self._send_status("CLARQ_TAG_FAILED")

    # ── Stop apriltag_ros ─────────────────────────────────────
    def _stop_apriltag(self):
        """Stop apriltag_ros node"""
        if (self._apriltag_proc is not None and
                self._apriltag_proc.poll() is None):
            self._apriltag_proc.terminate()
            self._apriltag_proc = None
            self.get_logger().info('AprilTag node stopped')
            self._send_status("CLARQ_TAG_STOPPED")
        else:
            self.get_logger().warn(
                'AprilTag not running')

    # ── Start return_land.py ──────────────────────────────────
    def _start_return_land(self):
        """Start return_land.py if not already running"""
        if (self._return_land_proc is not None and
                self._return_land_proc.poll() is None):
            self.get_logger().info(
                'return_land.py already running')
            self._send_status("CLARQ_LAND_ALREADY_RUNNING")
            return

        self.get_logger().info('Starting return_land.py...')
        self._send_status("CLARQ_LAND_STARTING")

        script = os.path.join(WORKSPACE, 'return_land.py')

        if not os.path.exists(script):
            self.get_logger().error(
                f'return_land.py not found at {script}')
            self._send_status("CLARQ_LAND_NOT_FOUND")
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

        # Wait and confirm
        time.sleep(2)
        if self._return_land_proc.poll() is None:
            self.get_logger().info(
                'return_land.py running!')
            self._send_status("CLARQ_LAND_RUNNING")
        else:
            stderr = self._return_land_proc.stderr.read().decode()
            self.get_logger().error(
                f'return_land.py failed: {stderr}')
            self._send_status("CLARQ_LAND_FAILED")

    # ── Stop everything ───────────────────────────────────────
    def _stop_all(self):
        """Stop both apriltag and return_land"""
        self.get_logger().info('Stopping all processes...')
        self._stop_apriltag()

        if (self._return_land_proc is not None and
                self._return_land_proc.poll() is None):
            self._return_land_proc.terminate()
            self._return_land_proc = None
            self.get_logger().info('return_land.py stopped')

        self._send_status("CLARQ_ALL_STOPPED")

    # ── Send status back to GUI ───────────────────────────────
    def _send_status(self, text):
        """Send status message back via MAVROS to GUI"""
        msg          = StatusText()
        msg.severity = 6  # INFO
        msg.text     = text
        self._status_pub.publish(msg)
        self.get_logger().info(f'Status sent: {text}')

    # ── Cleanup on shutdown ───────────────────────────────────
    def destroy_node(self):
        self.get_logger().info(
            'Shutting down — stopping all processes...')
        self._stop_all()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TriggerListener()

    print("")
    print("=" * 44)
    print("  MAVLink Trigger Listener running")
    print("  Waiting for commands from GUI...")
    print("=" * 44)
    print("")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nStopped by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
