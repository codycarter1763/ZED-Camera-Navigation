# landing_controller.py
# ROS2 node that handles precision landing
# Phase 1 — navigates to home using ZED pose
# Phase 2 — switches to AprilTag when land command received and tag visible
# Phase 3 — aligned and ready to land
#
# Activated by pressing L in desktop_return_sim.py
# which publishes /land_command topic

import rclpy
from rclpy.node import Node
from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
import numpy as np
import json
import os
import math

class LandingController(Node):

    def __init__(self):
        super().__init__('landing_controller')

        # ── Load home position ────────────────────────────────
        origin_file = os.path.expanduser('~/origin_T0.json')
        if os.path.exists(origin_file):
            with open(origin_file) as f:
                origin = json.load(f)
            self.home = np.array([
                origin['x'],
                origin['y'],
                origin['z']
            ])
            self.get_logger().info(
                f'Home loaded: '
                f'X={self.home[0]:.3f} '
                f'Y={self.home[1]:.3f} '
                f'Z={self.home[2]:.3f}')
        else:
            self.get_logger().warn(
                'origin_T0.json not found — home position not set')
            self.home = None

        # ── Config ────────────────────────────────────────────
        self.TAG_ID       = 0       # AprilTag ID to look for
        self.TAG_SIZE     = 0.15    # tag size in meters (15cm)
        self.DEADBAND     = 0.05    # 5cm deadband
        self.LAND_DIST    = 0.30    # trigger land when this close in Z

        # ── State ─────────────────────────────────────────────
        self.phase          = 1     # start in phase 1
        self.land_activated = False # waiting for L key press
        self.tag_visible    = False
        self.current_pos    = None
        self.tag_x_offset   = 0.0
        self.tag_y_offset   = 0.0
        self.tag_distance   = 0.0

        # ── Subscribers ───────────────────────────────────────

        # ZED pose — always listening
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/zed/zed_node/pose',
            self.pose_callback,
            10)

        # AprilTag detections — always listening
        # but only acts when land_activated is True
        self.detection_sub = self.create_subscription(
            AprilTagDetectionArray,
            '/detections',
            self.detection_callback,
            10)

        # Land command from desktop_return_sim.py L key
        self.land_sub = self.create_subscription(
            Bool,
            '/land_command',
            self.land_callback,
            10)

        # ── Timer — runs at 10Hz ──────────────────────────────
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('Landing controller started!')
        self.get_logger().info(
            'Waiting for land command — press L in return sim')

    # ── Land command callback ─────────────────────────────────
    def land_callback(self, msg):
        if msg.data and not self.land_activated:
            self.land_activated = True
            self.get_logger().info('')
            self.get_logger().info('============================================')
            self.get_logger().info('LAND COMMAND RECEIVED')
            self.get_logger().info('AprilTag guidance activated')
            self.get_logger().info('Watching for AprilTag ID: '
                                   f'{self.TAG_ID}')
            self.get_logger().info('============================================')
            self.get_logger().info('')

    # ── ZED pose callback ─────────────────────────────────────
    def pose_callback(self, msg):
        pos              = msg.pose.position
        self.current_pos = np.array([pos.x, pos.y, pos.z])

    # ── AprilTag detection callback ───────────────────────────
    def detection_callback(self, msg):

        # Ignore tags until land command received
        if not self.land_activated:
            return

        self.tag_visible = False

        for detection in msg.detections:
            if detection.id != self.TAG_ID:
                continue

            self.tag_visible = True

            # Get tag center in image pixels
            cx     = detection.centre.x
            cy     = detection.centre.y
            img_cx = 960 / 2.0
            img_cy = 540 / 2.0
            px_x   = cx - img_cx
            px_y   = cy - img_cy

            # Calculate tag width in pixels from corners
            corners      = detection.corners
            tag_px_width = math.sqrt(
                (corners[1].x - corners[0].x)**2 +
                (corners[1].y - corners[0].y)**2
            )

            if tag_px_width > 0:
                px_per_meter       = tag_px_width / self.TAG_SIZE
                self.tag_x_offset  = px_x / px_per_meter
                self.tag_y_offset  = px_y / px_per_meter
                focal_length_px    = 700.0
                self.tag_distance  = (
                    (self.TAG_SIZE * focal_length_px) / tag_px_width)
            else:
                self.tag_x_offset = 0.0
                self.tag_y_offset = 0.0
                self.tag_distance = 0.0

            # Switch to phase 2 when tag first seen
            if self.phase == 1:
                self.phase = 2
                self.get_logger().info(
                    'AprilTag detected! '
                    'Switching to Phase 2 — precision approach')

    # ── Main control loop ─────────────────────────────────────
    def control_loop(self):

        print('---')

        # ── Phase 1 — ZED pose navigation ────────────────────
        if self.phase == 1:
            if self.current_pos is not None and self.home is not None:
                dist  = float(np.linalg.norm(self.current_pos - self.home))
                delta = self.home - self.current_pos

                print(f'PHASE 1 — Navigating to home using ZED pose')
                print(f'Current  : '
                      f'X={self.current_pos[0]:.3f} '
                      f'Y={self.current_pos[1]:.3f} '
                      f'Z={self.current_pos[2]:.3f}')
                print(f'Home     : '
                      f'X={self.home[0]:.3f} '
                      f'Y={self.home[1]:.3f} '
                      f'Z={self.home[2]:.3f}')
                print(f'Distance : {dist:.3f}m')
                print(f'Move     : '
                      f'X={delta[0]:+.3f} '
                      f'Y={delta[1]:+.3f} '
                      f'Z={delta[2]:+.3f}')

                if not self.land_activated:
                    print('Waiting for land command — press L in sim')
                elif dist < 2.0:
                    print('Close to home — watching for AprilTag...')
                else:
                    print('Navigating toward home...')
            else:
                if not self.land_activated:
                    print('PHASE 1 — Waiting for land command...')
                else:
                    print('PHASE 1 — Waiting for ZED pose...')

        # ── Phase 2 — AprilTag precision approach ─────────────
        elif self.phase == 2:
            if not self.tag_visible:
                print('PHASE 2 — Searching for AprilTag...')
                print('Make sure tag is visible to camera')
                return

            x_err = self.tag_x_offset
            y_err = self.tag_y_offset
            z_err = self.tag_distance - self.LAND_DIST

            x_action = self.get_action(x_err, 'RIGHT',   'LEFT')
            y_action = self.get_action(y_err, 'DOWN',     'UP')
            z_action = self.get_action(z_err, 'DESCEND',  'ASCEND')

            print(f'PHASE 2 — AprilTag precision approach')
            print(f'Tag offset : '
                  f'X={x_err:+.3f}m  '
                  f'Y={y_err:+.3f}m  '
                  f'dist={self.tag_distance:.3f}m')
            print(f'X : {x_action}')
            print(f'Y : {y_action}')
            print(f'Z : {z_action}')

            # Check alignment
            if (abs(x_err) < self.DEADBAND and
                    abs(y_err) < self.DEADBAND):
                if abs(z_err) < self.DEADBAND:
                    self.phase = 3
                    print('ALIGNED AND AT LANDING HEIGHT — switching to Phase 3')
                else:
                    print('ALIGNED — DESCENDING')
            else:
                print('NOT ALIGNED — CORRECTING')

        # ── Phase 3 — Land ────────────────────────────────────
        elif self.phase == 3:
            print('PHASE 3 — LAND NOW')
            print('Send land command to Pixhawk')
            print('Drone should descend and land on rover')

    # ── Helper: error to direction string ────────────────────
    def get_action(self, error, positive_label, negative_label):
        if abs(error) < self.DEADBAND:
            return 'OK'
        elif error > 0:
            return f'move {positive_label} ({abs(error):.3f}m)'
        else:
            return f'move {negative_label} ({abs(error):.3f}m)'


def main(args=None):
    rclpy.init(args=args)
    node = LandingController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()