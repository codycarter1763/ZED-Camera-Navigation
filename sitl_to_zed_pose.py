#!/usr/bin/env python3
# ─────────────────────────────────────────────────────────────
# sitl_to_zed_pose.py  —  SIMULATION HELPER
# Bridges ArduPilot SITL LOCAL_POSITION_NED messages into the
# /zed/zed_node/pose ROS2 topic so all three Jetson programs
# work identically in simulation as in real flight.
#
# Run this instead of the real ZED wrapper during SITL testing.
#
# Usage:
#   python3 sitl_to_zed_pose.py
#   python3 sitl_to_zed_pose.py --sitl udp:192.168.1.100:14550
# ─────────────────────────────────────────────────────────────

import argparse, sys, threading, time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from pymavlink import mavutil

parser = argparse.ArgumentParser()
parser.add_argument('--sitl', default='udp:127.0.0.1:14550',
                    help='MAVLink connection string for SITL')
args = parser.parse_args()

print(f"[sitl_bridge] Connecting to SITL → {args.sitl}")
try:
    mav = mavutil.mavlink_connection(args.sitl)
    mav.wait_heartbeat(timeout=10)
    print("[sitl_bridge] SITL connected ✓")
except Exception as e:
    print(f"[sitl_bridge] ERROR: {e}")
    sys.exit(1)

# Shared position
pos_lock = threading.Lock()
pos = {"x": 0.0, "y": 0.0, "z": 0.0, "fresh": False}

def _mav_read():
    """Read LOCAL_POSITION_NED from SITL continuously."""
    while True:
        msg = mav.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=1)
        if msg:
            with pos_lock:
                # SITL NED → ZED frame
                # SITL: x=North  y=East  z=Down
                # ZED:  x=Right  y=Up    z=Toward viewer (right-handed)
                pos["x"]     =  msg.y    # East  → ZED X
                pos["y"]     = -msg.z    # Up    → ZED Y  (flip sign)
                pos["z"]     =  msg.x    # North → ZED Z
                pos["fresh"] =  True

threading.Thread(target=_mav_read, daemon=True).start()
time.sleep(0.5)

class BridgeNode(Node):
    def __init__(self):
        super().__init__('sitl_zed_bridge')
        self.pub = self.create_publisher(
            PoseStamped, '/zed/zed_node/pose', 10)
        self.create_timer(0.033, self._publish)   # 30 Hz
        self.get_logger().info(
            "Publishing SITL position → /zed/zed_node/pose")

    def _publish(self):
        with pos_lock:
            x, y, z, fresh = pos["x"], pos["y"], pos["z"], pos["fresh"]

        if not fresh:
            return

        msg = PoseStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        self.pub.publish(msg)

rclpy.init()
node = BridgeNode()
print("[sitl_bridge] Running — Ctrl+C to stop")
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass
node.destroy_node()
rclpy.shutdown()
