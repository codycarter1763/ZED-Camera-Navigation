# desktop_return_sim.py
# Rewritten for Jetson — uses ROS2 topics instead of opening ZED camera directly
# Subscribes to:
#   /zed/zed_node/pose   — current position from ZED wrapper
#   /detections          — AprilTag detections
#
# Controls:
#   R — simulate return path
#   L — activate landing sequence (starts AprilTag node)
#   C — clear trail
#   Q — quit

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from apriltag_msgs.msg import AprilTagDetectionArray
from std_msgs.msg import Bool

import json
import numpy as np
import pygame
import sys
import os
import threading
import math
import subprocess
from pymavlink import mavutil

# ── Check files exist ─────────────────────────────────────────
SCRIPT_DIR  = os.path.dirname(os.path.abspath(__file__))
ORIGIN_FILE = os.path.join(SCRIPT_DIR, "origin_T0.json")

if not os.path.exists(ORIGIN_FILE):
    print(f"ERROR: origin_T0.json not found at {ORIGIN_FILE}")
    sys.exit()

# ── Load origin ───────────────────────────────────────────────
with open(ORIGIN_FILE) as f:
    origin = json.load(f)
T0 = np.array([origin["x"], origin["y"], origin["z"]])

print("Origin loaded:")
print(f"  X: {T0[0]:.3f}  Y: {T0[1]:.3f}  Z: {T0[2]:.3f}")

# ── MAVLink connection to Pixhawk via UART ────────────────────
# Jetson UART ports (pick the one wired to Pixhawk TELEM2):
#   /dev/ttyTHS0  — UART0  (40-pin header, pins 8/10)
#   /dev/ttyTHS1  — UART1
#   /dev/ttyTHS2  — UART2  (most common for Pixhawk TELEM2)
# Pixhawk TELEM2 default baud is 921600.
# Match SERIAL2_BAUD in ArduPilot params to this value.
MAV_CONNECT = '/dev/ttyTHS2'
MAV_BAUD    = 921600

print(f"Connecting to Pixhawk via UART {MAV_CONNECT} @ {MAV_BAUD}...")
try:
    mav = mavutil.mavlink_connection(MAV_CONNECT, baud=MAV_BAUD)
    mav.wait_heartbeat(timeout=5)
    print(f"Pixhawk connected — system {mav.target_system} "
          f"component {mav.target_component}")
except Exception as e:
    print(f"WARNING: Pixhawk connection failed: {e}")
    print("Continuing without MAVLink — velocity commands will be skipped")
    mav = None

current_flight_mode = "UNKNOWN"
last_heartbeat_time = time.time()

# ── Shared state (thread safe) ────────────────────────────────
class SharedState:
    def __init__(self):
        self.lock              = threading.Lock()
        self.current_pos       = np.array([0.0, 0.0, 0.0])
        self.tag_detected      = False
        self.tag_x_offset      = 0.0
        self.tag_y_offset      = 0.0
        self.tag_distance      = 0.0
        self.phase             = 1
        self.pose_received     = False
        self.land_activated    = False
        self.apriltag_process  = None
        self.last_pose_time = 0.0

state = SharedState()

# ── ROS2 Node ─────────────────────────────────────────────────
class ReturnSimNode(Node):
    def __init__(self):
        super().__init__('return_sim')

        # Subscribe to ZED pose
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/zed/zed_node/pose',
            self.pose_callback,
            10)

        # Subscribe to AprilTag detections
        self.detection_sub = self.create_subscription(
            AprilTagDetectionArray,
            '/detections',
            self.detection_callback,
            10)

        # Publisher for land command
        self.land_pub = self.create_publisher(
            Bool,
            '/land_command',
            10)

        self.get_logger().info('Return sim node started')
        self.get_logger().info('Waiting for ZED pose...')

    def pose_callback(self, msg):
        pos = msg.pose.position
        state.last_pose_time = time.time()
        with state.lock:
            state.current_pos   = np.array([pos.x, pos.y, pos.z])
            state.pose_received = True

            
            # Auto switch phase when close to home
            # and land has been activated
            dist = float(np.linalg.norm(state.current_pos - T0))
            if dist < 2.0 and state.land_activated and state.phase == 1:
                state.phase = 2

    def detection_callback(self, msg):
        TAG_ID   = 0
        TAG_SIZE = 0.15

        with state.lock:
            # Only process detections after land activated
            if not state.land_activated:
                state.tag_detected = False
                return

            state.tag_detected = False

            for detection in msg.detections:
                if detection.id != TAG_ID:
                    continue

                state.tag_detected = True

                cx     = detection.centre.x
                cy     = detection.centre.y
                img_cx = 960 / 2.0
                img_cy = 540 / 2.0
                px_x   = cx - img_cx
                px_y   = cy - img_cy

                corners      = detection.corners
                tag_px_width = math.sqrt(
                    (corners[1].x - corners[0].x)**2 +
                    (corners[1].y - corners[0].y)**2
                )

                if tag_px_width > 0:
                    px_per_meter       = tag_px_width / TAG_SIZE
                    state.tag_x_offset = px_x / px_per_meter
                    state.tag_y_offset = px_y / px_per_meter
                    focal_length_px    = 700.0
                    state.tag_distance = (TAG_SIZE * focal_length_px) / tag_px_width
                else:
                    state.tag_x_offset = 0.0
                    state.tag_y_offset = 0.0
                    state.tag_distance = 0.0

                # Switch to phase 3 if aligned
                if (abs(state.tag_x_offset) < 0.05 and
                    abs(state.tag_y_offset) < 0.05 and
                    state.tag_distance < 0.35):
                    state.phase = 3

    def publish_land_command(self):
        msg      = Bool()
        msg.data = True
        self.land_pub.publish(msg)
        self.get_logger().info('Land command published!')


# ── ROS2 spin in background thread ───────────────────────────
ros_node = None

def ros2_thread():
    global ros_node
    rclpy.init()
    ros_node = ReturnSimNode()
    rclpy.spin(ros_node)
    ros_node.destroy_node()
    rclpy.shutdown()

ros_thread = threading.Thread(target=ros2_thread, daemon=True)
ros_thread.start()

# Wait for node to initialize
import time
time.sleep(1.0)

# ── Pygame setup ──────────────────────────────────────────────
pygame.init()
W, H   = 900, 650
screen = pygame.display.set_mode((W, H))
pygame.display.set_caption(
    "ZED Return Simulator  |  R=return  L=land  C=clear  Q=quit")
font_sm = pygame.font.SysFont("consolas", 14)
font_md = pygame.font.SysFont("consolas", 17, bold=True)
font_lg = pygame.font.SysFont("consolas", 22, bold=True)
clock   = pygame.time.Clock()

# ── Colors ────────────────────────────────────────────────────
BG          = ( 15,  15,  22)
GRID        = ( 35,  35,  50)
TRAIL       = ( 60, 120, 180)
HOME_COL    = ( 30, 200, 130)
CURRENT_COL = (240, 160,  40)
RETURN_COL  = ( 50, 210, 120)
TAG_COL     = (220,  80, 220)
TEXT_COL    = (210, 210, 220)
DIM_COL     = (100, 100, 120)
WARN_COL    = (220, 100,  60)
PANEL_COL   = ( 25,  25,  38)

# ── Map settings ──────────────────────────────────────────────
MAP_X  = 250
MAP_Y  = 50
MAP_W  = 620
MAP_H  = 560
MAP_CX = MAP_X + MAP_W // 2
MAP_CY = MAP_Y + MAP_H // 2
SCALE  = 12

def world_to_screen(x, z):
    sx = int(MAP_CX + x * SCALE)
    sy = int(MAP_CY - z * SCALE)
    return sx, sy

# ── Sim state ─────────────────────────────────────────────────
trail       = []
simulating  = False
return_path = []
return_idx  = 0
sim_done    = False

def compute_return_path(start, end, steps=80):
    return [start + (end - start) * (i / steps) for i in range(steps + 1)]

def activate_landing():
    """Start apriltag node and publish land command"""
    with state.lock:
        if state.land_activated:
            print("  Landing already activated!")
            return
        state.land_activated = True

    print("")
    print("  ============================================")
    print("  LAND COMMAND ACTIVATED")
    print("  Starting AprilTag node...")
    print("  ============================================")
    print("")

    # Launch apriltag node as subprocess
    state.apriltag_process = subprocess.Popen(
        ['bash', '-c',
         'source ~/ros2_ws/install/setup.bash && '
         'ros2 run apriltag_ros apriltag_node --ros-args '
         '-r image_rect:=/zed/zed_node/rgb/color/rect/image '
         '-r camera_info:=/zed/zed_node/rgb/color/rect/camera_info '
         f'--params-file {os.path.expanduser("~/tags_config.yaml")}']
    )

    # Publish land command to landing_controller
    if ros_node is not None:
        ros_node.publish_land_command()

# ── MAVLink helpers ───────────────────────────────────────────
def send_velocity_ned(vx, vy, vz, yaw_rate=0.0):
    """
    Send body-frame velocity setpoint to Pixhawk over UART.
    vx = forward/North (m/s)
    vy = right/East    (m/s)
    vz = down          (m/s, positive = descend)
    Must be called at ~10Hz to keep ArduPilot GUIDED mode active.
    """
    if mav is None:
        return
    mav.mav.set_position_target_local_ned_send(
        0,                              # time_boot_ms (unused)
        mav.target_system,
        mav.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        0b0000_1111_1100_0111,          # typemask: velocity only
        0, 0, 0,                        # x, y, z position (ignored)
        vx, vy, vz,                     # velocity setpoints (m/s)
        0, 0, 0,                        # acceleration (ignored)
        0, yaw_rate                     # yaw, yaw_rate
    )

def send_land_command():
    """Tell Pixhawk to execute LAND mode via MAVLink over UART."""
    if mav is None:
        print("WARNING: MAVLink not connected — cannot send land command")
        return
    mav.mav.command_long_send(
        mav.target_system,
        mav.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0,          # confirmation
        0, 0, 0, 0, # params 1-4 unused
        0, 0, 0     # lat, lon, alt (0 = land at current position)
    )
    print("MAV_CMD_NAV_LAND sent to Pixhawk via UART")

# ── MAVLink command thread ────────────────────────────────────
# Runs at 10Hz — sends velocity commands to Pixhawk based on phase.
# ArduPilot GUIDED mode times out after ~1s without a fresh command,
# so this thread must keep refreshing at 10Hz while flying.
SPEED     = 1.5   # m/s cruise speed toward home (Phase 1)
TAG_GAIN  = 0.8   # proportional gain for AprilTag corrections (Phase 2)
DEADBAND  = 0.05  # 5cm — matches landing_controller.py
LAND_DIST = 0.30  # begin descent when tag is closer than this (m)

def clamp(v, max_v):
    return max(-max_v, min(max_v, v))

def mavlink_thread():
    global current_flight_mode, last_heartbeat_time

    land_sent = False

    def clamp(v, max_v):
        return max(-max_v, min(max_v, v))

    while True:
        time.sleep(0.1)  # 10 Hz

        # ── MAVLink heartbeat (non-blocking) ─────────────────
        msg = mav.recv_match(type='HEARTBEAT', blocking=False) if mav else None
        if msg:
            current_flight_mode = mav.flightmode
            last_heartbeat_time = time.time()

        # ── Heartbeat failsafe ───────────────────────────────
        if mav and (time.time() - last_heartbeat_time > 1.0):
            print("WARNING: MAVLink heartbeat lost — stopping control")
            continue

        # ── Respect operator control (Mission Planner) ───────
        if mav and current_flight_mode != "GUIDED":
            # Operator switched to RTL, LOITER, etc.
            continue

        # ── Read shared state safely ─────────────────────────
        with state.lock:
            phase        = state.phase
            land_active  = state.land_activated
            tag_visible  = state.tag_detected
            x_off        = state.tag_x_offset
            y_off        = state.tag_y_offset
            tag_dist     = state.tag_distance
            cur          = state.current_pos.copy()
            pose_ok      = state.pose_received
            last_pose_ts = getattr(state, "last_pose_time", 0.0)

        # ── Pose failsafe (timeout-based) ────────────────────
        if not pose_ok or (time.time() - last_pose_ts > 0.5):
            print("WARNING: Pose lost/stale — holding position")
            send_velocity_ned(0.0, 0.0, 0.0)
            continue

        # ── Do nothing until landing is activated ────────────
        if not land_active:
            continue

        # ── Phase 1 — Navigate toward home ───────────────────
        if phase == 1:
            delta = T0 - cur
            horiz = math.sqrt(delta[0]**2 + delta[2]**2)

            if horiz > 0.1:
                vx = clamp(SPEED * delta[2] / horiz, 1.0)  # North
                vy = clamp(SPEED * delta[0] / horiz, 1.0)  # East
                vz = 0.0
            else:
                vx, vy, vz = 0.0, 0.0, 0.0

            send_velocity_ned(vx, vy, vz)

        # ── Phase 2 — AprilTag precision landing ─────────────
        elif phase == 2:
            if tag_visible:
                vx = clamp(TAG_GAIN * y_off, 0.5)
                vy = clamp(TAG_GAIN * x_off, 0.5)

                z_err = tag_dist - LAND_DIST
                vz = clamp(TAG_GAIN * z_err, 0.3) if abs(z_err) > DEADBAND else 0.0

                send_velocity_ned(vx, vy, vz)
            else:
                # Tag lost → hold position
                send_velocity_ned(0.0, 0.0, 0.0)

        # ── Phase 3 — Land and release control ───────────────
        elif phase == 3:
            if not land_sent:
                send_land_command()
                land_sent = True
                print("Landing initiated — releasing control to autopilot")

            # STOP sending velocity commands after LAND
            continue

mav_thread = threading.Thread(target=mavlink_thread, daemon=True)
mav_thread.start()

# ── Main loop ─────────────────────────────────────────────────
running = True
while running:

    # ── Get state thread safely ───────────────────────────────
    with state.lock:
        phase       = state.phase
        land_active = state.land_activated
        tag_visible = state.tag_detected
        x_off       = state.tag_x_offset
        y_off       = state.tag_y_offset
        tag_dist    = state.tag_distance
        cur         = state.current_pos.copy()
        pose_ok     = state.pose_received

    # ── Pose failsafe ──
    if not pose_ok:
        print("WARNING: No pose — holding position")
        send_velocity_ned(0.0, 0.0, 0.0)
        continue

    # ── Events ────────────────────────────────────────────────
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

        if event.type == pygame.KEYDOWN:

            # Q — quit
            if event.key == pygame.K_q:
                running = False

            # R — simulate return path
            if event.key == pygame.K_r:
                if pose_received:
                    return_path = compute_return_path(
                        current_pos.copy(), T0)
                    return_idx  = 0
                    simulating  = True
                    sim_done    = False
                    print("Simulating return path...")
                else:
                    print("Waiting for ZED pose...")

            # L — activate landing sequence
            if event.key == pygame.K_l:
                if pose_received:
                    activate_landing()
                else:
                    print("Waiting for ZED pose first...")

            # C — clear trail
            if event.key == pygame.K_c:
                trail.clear()
                simulating = False
                sim_done   = False

    # ── Update trail ──────────────────────────────────────────
    if pose_received:
        trail.append((current_pos[0], current_pos[2]))
        if len(trail) > 800:
            trail.pop(0)

    # ── Advance simulation ────────────────────────────────────
    if simulating and not sim_done:
        return_idx += 2
        if return_idx >= len(return_path):
            return_idx = len(return_path) - 1
            sim_done   = True

    # ── Draw ──────────────────────────────────────────────────
    screen.fill(BG)
    pygame.draw.rect(screen, PANEL_COL, (0, 0, 245, H))
    pygame.draw.rect(screen, GRID, (MAP_X, MAP_Y, MAP_W, MAP_H), 1)

    # Grid dots
    for gx in range(-40, 41):
        for gz in range(-40, 41):
            sx, sy = world_to_screen(gx, gz)
            if MAP_X < sx < MAP_X+MAP_W and MAP_Y < sy < MAP_Y+MAP_H:
                pygame.draw.circle(screen, GRID, (sx, sy), 1)

    # Meter labels
    for m in range(-40, 41, 5):
        sx, sy = world_to_screen(m, 0)
        if MAP_X < sx < MAP_X+MAP_W:
            lbl = font_sm.render(f"{m}m", True, GRID)
            screen.blit(lbl, (sx - 10, MAP_CY + 6))

    # Trail
    if len(trail) > 1:
        pts = [world_to_screen(x, z) for x, z in trail]
        pts = [(x, y) for x, y in pts
               if MAP_X < x < MAP_X+MAP_W and MAP_Y < y < MAP_Y+MAP_H]
        if len(pts) > 1:
            pygame.draw.lines(screen, TRAIL, False, pts, 2)

    # Return path
    if simulating and len(return_path) > 1:
        rpts = [world_to_screen(p[0], p[2])
                for p in return_path[:return_idx+1]]
        rpts = [(x, y) for x, y in rpts
                if MAP_X < x < MAP_X+MAP_W and MAP_Y < y < MAP_Y+MAP_H]
        if len(rpts) > 1:
            pygame.draw.lines(screen, RETURN_COL, False, rpts, 3)
        if not sim_done:
            ap = return_path[return_idx]
            ax, ay = world_to_screen(ap[0], ap[2])
            pygame.draw.circle(screen, RETURN_COL, (ax, ay), 9)
            pygame.draw.circle(screen, (255, 255, 255), (ax, ay), 9, 2)

    # Home marker
    dist = float(np.linalg.norm(current_pos - T0))
    hx, hy = world_to_screen(T0[0], T0[2])
    if MAP_X < hx < MAP_X+MAP_W and MAP_Y < hy < MAP_Y+MAP_H:
        pygame.draw.circle(screen, HOME_COL, (hx, hy), 14)
        pygame.draw.circle(screen, (255, 255, 255), (hx, hy), 14, 2)
        lbl = font_sm.render("HOME", True, (255, 255, 255))
        screen.blit(lbl, (hx - 18, hy - 28))

    # AprilTag ring around home when tag visible
    if tag_detected and land_active:
        pygame.draw.circle(screen, TAG_COL, (hx, hy), 22, 3)
        lbl = font_sm.render("TAG", True, TAG_COL)
        screen.blit(lbl, (hx - 12, hy + 26))

    # Current position dot
    if pose_received:
        cx, cy = world_to_screen(current_pos[0], current_pos[2])
        if MAP_X < cx < MAP_X+MAP_W and MAP_Y < cy < MAP_Y+MAP_H:
            col = TAG_COL if land_active else CURRENT_COL
            pygame.draw.circle(screen, col, (cx, cy), 11)
            pygame.draw.circle(screen, (255, 255, 255), (cx, cy), 11, 2)
            lbl = font_sm.render("YOU", True, (255, 255, 255))
            screen.blit(lbl, (cx - 10, cy - 26))
    else:
        msg1 = font_md.render("Waiting for ZED pose...", True, WARN_COL)
        msg2 = font_sm.render(
            "Make sure ZED wrapper is running", True, DIM_COL)
        screen.blit(msg1, (MAP_X + 20, MAP_CY - 20))
        screen.blit(msg2, (MAP_X + 20, MAP_CY + 15))

    # Sim done message
    if sim_done:
        msg = font_md.render("ARRIVED AT HOME!", True, HOME_COL)
        screen.blit(msg, (MAP_X + MAP_W//2 - 110, MAP_Y + MAP_H - 40))

    # Phase 3 landed message
    if phase == 3:
        msg = font_lg.render("LAND NOW!", True, TAG_COL)
        screen.blit(msg, (MAP_X + MAP_W//2 - 70, MAP_Y + 20))

    # MAVLink status indicator (bottom of left panel)
    mav_text  = "UART OK ✓" if mav is not None else "UART DISCONNECTED"
    mav_color = HOME_COL    if mav is not None else WARN_COL
    surf = font_sm.render(mav_text, True, mav_color)
    screen.blit(surf, (15, H - 25))

    # ── Left panel HUD ────────────────────────────────────────
    phase_color = (TAG_COL    if phase == 3
                   else TAG_COL    if phase == 2
                   else TEXT_COL)
    phase_text  = ("PHASE 3 — LAND NOW" if phase == 3
                   else "PHASE 2 — APRILTAG"  if phase == 2
                   else "PHASE 1 — NAVIGATE")

    hud = [
        ("ZED RETURN SIM",        font_lg, TEXT_COL,     20),
        ("",                      font_sm, DIM_COL,      52),
        ("PHASE",                 font_sm, DIM_COL,      65),
        (phase_text,              font_md, phase_color,   83),
        ("",                      font_sm, DIM_COL,     108),
        ("POSITION (m)",          font_sm, DIM_COL,     118),
        (f"X: {current_pos[0]:+.3f}", font_sm, TEXT_COL, 136),
        (f"Y: {current_pos[1]:+.3f}", font_sm, TEXT_COL, 152),
        (f"Z: {current_pos[2]:+.3f}", font_sm, TEXT_COL, 168),
        ("",                      font_sm, DIM_COL,     188),
        ("HOME (m)",              font_sm, DIM_COL,     198),
        (f"X: {T0[0]:+.3f}",     font_sm, HOME_COL,    216),
        (f"Y: {T0[1]:+.3f}",     font_sm, HOME_COL,    232),
        (f"Z: {T0[2]:+.3f}",     font_sm, HOME_COL,    248),
        ("",                      font_sm, DIM_COL,     268),
        ("DIST TO HOME",          font_sm, DIM_COL,     278),
        (f"{dist:.2f} m",         font_lg, TEXT_COL,    296),
        ("",                      font_sm, DIM_COL,     325),
        ("LANDING",               font_sm, DIM_COL,     338),
        ("ACTIVE ✓" if land_active else "press L to activate",
                                  font_md,
                                  TAG_COL if land_active else DIM_COL,
                                  356),
        ("",                      font_sm, DIM_COL,     378),
        ("APRILTAG",              font_sm, DIM_COL,     391),
        ("VISIBLE ✓" if tag_detected else "not visible",
                                  font_md,
                                  TAG_COL if tag_detected else DIM_COL,
                                  409),
        ("",                      font_sm, DIM_COL,     431),
        ("TAG OFFSET",            font_sm, DIM_COL,     444),
        (f"X: {tag_x_offset:+.3f}m" if tag_detected else "X: --",
                                  font_sm, TEXT_COL,    462),
        (f"Y: {tag_y_offset:+.3f}m" if tag_detected else "Y: --",
                                  font_sm, TEXT_COL,    478),
        (f"dist:{tag_distance:.2f}m" if tag_detected else "dist: --",
                                  font_sm, TEXT_COL,    494),
        ("",                      font_sm, DIM_COL,     514),
        ("CONTROLS",              font_sm, DIM_COL,     527),
        ("[R] Simulate return",   font_sm, TEXT_COL,    545),
        ("[L] Activate landing",  font_sm,
                                  TAG_COL if not land_active else DIM_COL,
                                  561),
        ("[C] Clear trail",       font_sm, TEXT_COL,    577),
        ("[Q] Quit",              font_sm, TEXT_COL,    593),
    ]

    for text, fnt, col, y in hud:
        surf = fnt.render(text, True, col)
        screen.blit(surf, (15, y))

    # Distance bar
    bar_y = 313
    pygame.draw.rect(screen, GRID, (15, bar_y, 215, 8), border_radius=4)
    bar_w = int(215 * min(dist, 10.0) / 10.0)
    bar_c = (HOME_COL if dist < 2.0
             else WARN_COL if dist < 5.0
             else (180, 60, 60))
    pygame.draw.rect(screen, bar_c, (15, bar_y, bar_w, 8), border_radius=4)

    pygame.display.flip()
    clock.tick(30)

# ── Cleanup ───────────────────────────────────────────────────
if state.apriltag_process is not None:
    state.apriltag_process.terminate()

pygame.quit()
sys.exit()