#!/usr/bin/env python3
# ─────────────────────────────────────────────────────────────
# set_origin.py  —  PROGRAM 1
# Run on Jetson BEFORE arming the drone.
# Subscribes to the real ZED ROS2 wrapper pose topic and records
# the landing pad position (T0) to origin_T0.json.
#
# ZED topic used:
#   /zed/zed_node/pose  (geometry_msgs/PoseStamped)
#
# Trigger methods (any of these saves the origin):
#   1. Write trigger file: .set_home_trigger  (from clarq_rf_listener)
#   2. Press SPACE in terminal (stdin)
#   3. Press Q or Ctrl+C to quit
#
# Usage:   python3 set_origin.py
# Output:  origin_T0.json  (same directory as this script)
# ─────────────────────────────────────────────────────────────

import json
import os
import sys
import threading
import time
import select

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

SCRIPT_DIR    = os.path.dirname(os.path.abspath(__file__))
ORIGIN_FILE   = os.path.join(SCRIPT_DIR, "origin_T0.json")
TRIGGER_FILE  = os.path.join(SCRIPT_DIR, ".set_home_trigger")

# ── ZED topic ─────────────────────────────────────────────────
ZED_POSE_TOPIC = '/zed/zed_node/pose'

# ── Shared state ──────────────────────────────────────────────
class State:
    def __init__(self):
        self.lock       = threading.Lock()
        self.pos        = [0.0, 0.0, 0.0]
        self.pose_ok    = False
        self.origin_set = False
        self.pose_count = 0

S = State()

# ── ROS2 node ─────────────────────────────────────────────────
class OriginNode(Node):
    def __init__(self):
        super().__init__('set_origin')
        self.create_subscription(
            PoseStamped, ZED_POSE_TOPIC, self._cb, 10)
        self.get_logger().info(
            f"Subscribed to {ZED_POSE_TOPIC}")

    def _cb(self, msg):
        p = msg.pose.position
        with S.lock:
            S.pos        = [p.x, p.y, p.z]
            S.pose_ok    = True
            S.pose_count += 1

def _ros_spin():
    rclpy.init()
    node = OriginNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

threading.Thread(target=_ros_spin, daemon=True).start()
time.sleep(1.0)

# ── Save function ─────────────────────────────────────────────
def save_origin():
    with S.lock:
        if not S.pose_ok:
            print("[set_origin] ERROR: No ZED pose received yet!")
            return False
        data = {"x": S.pos[0], "y": S.pos[1], "z": S.pos[2]}

    with open(ORIGIN_FILE, "w") as f:
        json.dump(data, f, indent=2)

    with S.lock:
        S.origin_set = True

    print(f"[set_origin] ✓ Origin saved → {ORIGIN_FILE}")
    print(f"[set_origin]   X: {data['x']:+.4f}")
    print(f"[set_origin]   Y: {data['y']:+.4f}")
    print(f"[set_origin]   Z: {data['z']:+.4f}")
    return True

# ── Clean up old trigger file on start ────────────────────────
if os.path.exists(TRIGGER_FILE):
    os.remove(TRIGGER_FILE)

# ── Main loop ─────────────────────────────────────────────────
print("=" * 44)
print("  SET ORIGIN — waiting for trigger")
print(f"  Pose topic: {ZED_POSE_TOPIC}")
print(f"  Output:     {ORIGIN_FILE}")
print("  Trigger:    GUI SET HOME button")
print("  Manual:     press SPACE + Enter")
print("=" * 44)

try:
    while True:
        with S.lock:
            ok    = S.pose_ok
            count = S.pose_count
            pos   = S.pos.copy()

        # Print live pose every 2 seconds
        if ok and count % 30 == 0:
            print(f"[set_origin] Live pose — "
                  f"X:{pos[0]:+.4f} Y:{pos[1]:+.4f} Z:{pos[2]:+.4f}")

        # Method 1 — check trigger file
        if os.path.exists(TRIGGER_FILE):
            print("[set_origin] Trigger file detected — saving origin...")
            try:
                os.remove(TRIGGER_FILE)
            except Exception:
                pass
            save_origin()

        # Method 2 — check stdin for SPACE keypress (non-blocking)
        if select.select([sys.stdin], [], [], 0)[0]:
            key = sys.stdin.read(1)
            if key == ' ':
                print("[set_origin] SPACE pressed — saving origin...")
                save_origin()
            elif key in ('q', 'Q'):
                print("[set_origin] Quit")
                break

        time.sleep(0.1)

except KeyboardInterrupt:
    print("\n[set_origin] Stopped")
