#!/usr/bin/env python3
# ─────────────────────────────────────────────────────────────
# fake_apriltag.py  —  SIMULATION HELPER
# Publishes fake AprilTag detections on /detections once the
# simulated drone is within DETECT_RADIUS metres of origin.
# Simulates slight offsets that converge toward zero as the
# drone gets closer, mimicking real tag behaviour.
#
# Usage:   cd ~/ZED\ Navigation && python3 fake_apriltag.py
# ─────────────────────────────────────────────────────────────

import json, math, os, sys, threading, time
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from apriltag_msgs.msg import AprilTagDetectionArray, AprilTagDetection, Point

SCRIPT_DIR    = os.path.dirname(os.path.abspath(__file__))
ORIGIN_FILE   = os.path.join(SCRIPT_DIR, "origin_T0.json")
DETECT_RADIUS = 3.0    # metres — tag becomes visible within this range
TAG_ID        = 0
IMG_W, IMG_H  = 960, 540

# Load origin
if not os.path.exists(ORIGIN_FILE):
    print("ERROR: origin_T0.json not found — run set_origin.py first")
    sys.exit(1)

with open(ORIGIN_FILE) as f:
    _o = json.load(f)
T0 = np.array([_o["x"], _o["y"], _o["z"]])
print(f"[fake_tag] Origin loaded: {T0}")

# Shared pose
pose_lock = threading.Lock()
cur_pos   = np.zeros(3)
pose_ok   = False

class FakeTagNode(Node):
    def __init__(self):
        super().__init__('fake_apriltag')
        self.pub = self.create_publisher(
            AprilTagDetectionArray, '/detections', 10)
        self.create_subscription(
            PoseStamped, '/zed/zed_node/pose', self._pose_cb, 10)
        self.create_timer(0.1, self._publish)   # 10 Hz
        self.get_logger().info(
            f"Fake tag active — visible within {DETECT_RADIUS}m of origin")

    def _pose_cb(self, msg):
        global cur_pos, pose_ok
        with pose_lock:
            p       = msg.pose.position
            cur_pos = np.array([p.x, p.y, p.z])
            pose_ok = True

    def _publish(self):
        global cur_pos, pose_ok
        with pose_lock:
            if not pose_ok:
                return
            cp   = cur_pos.copy()

        dist  = float(np.linalg.norm(cp - T0))
        arr   = AprilTagDetectionArray()
        arr.header.stamp    = self.get_clock().now().to_msg()
        arr.header.frame_id = "camera"

        if dist > DETECT_RADIUS:
            # No detection — publish empty array
            self.pub.publish(arr)
            return

        # Simulate tag pixel offset proportional to distance from origin
        # As dist → 0, offsets → 0 (perfectly centred)
        scale  = dist / DETECT_RADIUS
        off_x  = (cp[0] - T0[0]) * 200 * scale   # pixels
        off_y  = (cp[2] - T0[2]) * 200 * scale   # pixels

        # Tag pixel size grows as drone descends (closer = bigger)
        altitude   = max(cp[1], 0.1)
        tag_px_w   = int(700.0 * 0.15 / max(altitude, 0.1))
        tag_px_w   = max(20, min(tag_px_w, 400))

        cx = IMG_W / 2.0 + off_x
        cy = IMG_H / 2.0 + off_y

        det        = AprilTagDetection()
        det.id     = TAG_ID

        det.centre      = Point()
        det.centre.x    = cx
        det.centre.y    = cy

        # Corners: top-left, top-right, bottom-right, bottom-left
        half = tag_px_w / 2.0
        corners = [
            (cx - half, cy - half),
            (cx + half, cy - half),
            (cx + half, cy + half),
            (cx - half, cy + half),
        ]
        det.corners = []
        for (px, py) in corners:
            pt   = Point()
            pt.x = px
            pt.y = py
            det.corners.append(pt)

        arr.detections.append(det)
        self.pub.publish(arr)

rclpy.init()
node = FakeTagNode()
print("[fake_tag] Running — Ctrl+C to stop")
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass
node.destroy_node()
rclpy.shutdown()
