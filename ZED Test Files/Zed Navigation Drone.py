# zed_apriltag_land.py
# Minimized ZED + AprilTag precision landing controller
# Jetson deployment — ROS2 real topics
#
# Subscribes:
#   /zed/zed_node/pose        — ZED camera pose
#   /detections               — AprilTag detections
# Publishes:
#   /land_command             — triggers landing_controller
#
# MAVLink: UDP (change to /dev/ttyTHS2 for UART on Jetson)
# Phases:
#   1 — Navigate toward home (GPS/ZED guided)
#   2 — AprilTag precision alignment
#   3 — Land via MAV_CMD_NAV_LAND
#
# Keys: L=activate landing  C=clear trail  Q=quit
 
import json, math, os, sys, threading, time
import numpy as np
import pygame
from pymavlink import mavutil
 
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from apriltag_msgs.msg import AprilTagDetectionArray
from std_msgs.msg import Bool
 
# ── Config ────────────────────────────────────────────────────
SCRIPT_DIR  = os.path.dirname(os.path.abspath(__file__))
ORIGIN_FILE = os.path.join(SCRIPT_DIR, "origin_T0.json")
 
MAV_CONNECT = '/dev/ttyACM0'
MAV_BAUD    = 115200
 
TAG_ID      = 0
TAG_SIZE    = 0.15   # metres, physical tag size
IMG_W, IMG_H = 960, 540
FOCAL_LEN   = 700.0  # pixels — tune to your ZED config
 
NAV_SPEED   = 1.5    # m/s cruise toward home
TAG_GAIN    = 0.8    # proportional gain for tag corrections
DEADBAND    = 0.05   # metres
LAND_DIST   = 0.30   # descend when tag closer than this (m)
HOME_RADIUS = 2.0    # metres — switch Phase 1→2 when within this range
 
# ── Load origin ───────────────────────────────────────────────
if not os.path.exists(ORIGIN_FILE):
    print(f"ERROR: {ORIGIN_FILE} not found"); sys.exit(1)
 
with open(ORIGIN_FILE) as f:
    _o = json.load(f)
T0 = np.array([_o["x"], _o["y"], _o["z"]])
print(f"Origin loaded — X:{T0[0]:.3f} Y:{T0[1]:.3f} Z:{T0[2]:.3f}")
 
# ── MAVLink ───────────────────────────────────────────────────
print(f"Connecting MAVLink → {MAV_CONNECT} ...")
try:
    mav = mavutil.mavlink_connection(MAV_CONNECT, baud=MAV_BAUD)
    mav.wait_heartbeat(timeout=5)
    print(f"MAVLink OK — sys {mav.target_system} comp {mav.target_component}")
except Exception as e:
    print(f"WARNING: MAVLink failed ({e}) — velocity commands disabled")
    mav = None
 
# ── Shared state ──────────────────────────────────────────────
class State:
    def __init__(self):
        self.lock           = threading.Lock()
        self.pos            = np.zeros(3)       # current XYZ from ZED
        self.pose_ok        = False
        self.pose_time      = 0.0
        self.tag_visible    = False
        self.tag_x          = 0.0               # metres offset from centre
        self.tag_y          = 0.0
        self.tag_dist       = 0.0               # metres to tag
        self.phase          = 1
        self.land_active    = False
        self.flight_mode    = "UNKNOWN"
        self.hb_time        = time.time()
 
S = State()
 
# ── MAVLink helpers ───────────────────────────────────────────
def _clamp(v, lim): return max(-lim, min(lim, v))
 
def send_velocity(vx, vy, vz):
    """Body-NED velocity setpoint — call at 10 Hz."""
    if mav is None: return
    mav.mav.set_position_target_local_ned_send(
        0,
        mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        0b0000_1111_1100_0111,
        0, 0, 0,
        vx, vy, vz,
        0, 0, 0,
        0, 0
    )
 
def send_land():
    if mav is None:
        print("WARNING: no MAVLink — land skipped"); return
    mav.mav.command_long_send(
        mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0, 0, 0, 0, 0, 0, 0, 0
    )
    print("MAV_CMD_NAV_LAND sent")
 
# ── ROS2 Node ─────────────────────────────────────────────────
class LandNode(Node):
    def __init__(self):
        super().__init__('zed_land')
        self.create_subscription(PoseStamped,
            '/zed/zed_node/pose', self._pose_cb, 10)
        self.create_subscription(AprilTagDetectionArray,
            '/detections', self._tag_cb, 10)
        self._land_pub = self.create_publisher(Bool, '/land_command', 10)
        self.get_logger().info('LandNode ready')
 
    def _pose_cb(self, msg):
        p = msg.pose.position
        with S.lock:
            S.pos       = np.array([p.x, p.y, p.z])
            S.pose_ok   = True
            S.pose_time = time.time()
            dist = float(np.linalg.norm(S.pos - T0))
            # Phase 1→2: close enough and landing activated
            if S.phase == 1 and S.land_active and dist < HOME_RADIUS:
                S.phase = 2
 
    def _tag_cb(self, msg):
        with S.lock:
            if not S.land_active:
                S.tag_visible = False
                return
            S.tag_visible = False
            for det in msg.detections:
                if det.id != TAG_ID:
                    continue
                S.tag_visible = True
                px_x = det.centre.x - IMG_W / 2.0
                px_y = det.centre.y - IMG_H / 2.0
                c    = det.corners
                w_px = math.hypot(c[1].x - c[0].x, c[1].y - c[0].y)
                if w_px > 0:
                    ppm          = w_px / TAG_SIZE
                    S.tag_x      = px_x / ppm
                    S.tag_y      = px_y / ppm
                    S.tag_dist   = (TAG_SIZE * FOCAL_LEN) / w_px
                    print(f"DEBUG tag_x:{S.tag_x:.3f} tag_y:{S.tag_y:.3f} dist:{S.tag_dist:.3f} w_px:{w_px:.1f}")
                    print(f"DEBUG need x<{DEADBAND} y<{DEADBAND} dist<{LAND_DIST}")
                    print(f"DEBUG pass: x={abs(S.tag_x)<DEADBAND} y={abs(S.tag_y)<DEADBAND} dist={S.tag_dist<LAND_DIST}")
                else:
                    S.tag_x = S.tag_y = S.tag_dist = 0.0
                # Phase 2→3: aligned and close
                if (abs(S.tag_x) < DEADBAND and
                    abs(S.tag_y) < DEADBAND and
                    S.tag_dist < LAND_DIST):
                    S.phase = 3
                break
 
    def publish_land(self):
        msg = Bool(); msg.data = True
        self._land_pub.publish(msg)
        self.get_logger().info('Land command published')
 
ros_node: LandNode = None
 
def _ros_spin():
    global ros_node
    rclpy.init()
    ros_node = LandNode()
    rclpy.spin(ros_node)
    ros_node.destroy_node()
    rclpy.shutdown()
 
threading.Thread(target=_ros_spin, daemon=True).start()
time.sleep(1.0)   # wait for node
 
# ── MAVLink control thread (10 Hz) ────────────────────────────
def _mav_thread():
    land_sent = False
    while True:
        time.sleep(0.1)
 
        # Heartbeat
        if mav:
            hb = mav.recv_match(type='HEARTBEAT', blocking=False)
            if hb:
                with S.lock:
                    S.flight_mode = mav.flightmode
                    S.hb_time     = time.time()
            with S.lock:
                stale_hb = (time.time() - S.hb_time) > 1.0
                mode     = S.flight_mode
            if stale_hb or mode != "GUIDED":
                continue
 
        with S.lock:
            phase     = S.phase
            active    = S.land_active
            pose_ok   = S.pose_ok
            pose_age  = time.time() - S.pose_time
            tag_vis   = S.tag_visible
            tx, ty    = S.tag_x, S.tag_y
            tdist     = S.tag_dist
            cur       = S.pos.copy()
 
        if not active:
            continue
        if not pose_ok or pose_age > 0.5:
            send_velocity(0, 0, 0); continue
 
        if phase == 1:
            d     = T0 - cur
            horiz = math.hypot(d[0], d[2])
            if horiz > 0.1:
                send_velocity(
                    _clamp(NAV_SPEED * d[2] / horiz, 1.0),
                    _clamp(NAV_SPEED * d[0] / horiz, 1.0),
                    0.0)
            else:
                send_velocity(0, 0, 0)
 
        elif phase == 2:
            if tag_vis:
                z_err = tdist - LAND_DIST
                send_velocity(
                    _clamp(TAG_GAIN * ty,   0.5),
                    _clamp(TAG_GAIN * tx,   0.5),
                    _clamp(TAG_GAIN * z_err, 0.3)
                        if abs(z_err) > DEADBAND else 0.0)
            else:
                send_velocity(0, 0, 0)
 
        elif phase == 3:
            if not land_sent:
                send_land()
                land_sent = True
 
threading.Thread(target=_mav_thread, daemon=True).start()
 
# ── Pygame HUD ────────────────────────────────────────────────
pygame.init()
W, H   = 480, 420
screen = pygame.display.set_mode((W, H))
pygame.display.set_caption("ZED Landing  |  L=land  C=clear  Q=quit")
F_SM = pygame.font.SysFont("consolas", 13)
F_MD = pygame.font.SysFont("consolas", 15, bold=True)
F_LG = pygame.font.SysFont("consolas", 20, bold=True)
clock = pygame.time.Clock()
 
# Colours
BG    = ( 15,  15,  22)
PANEL = ( 25,  25,  38)
GR    = ( 35,  35,  50)
C_OK  = ( 30, 200, 130)
C_WRN = (220, 100,  60)
C_TAG = (220,  80, 220)
C_TXT = (210, 210, 220)
C_DIM = (100, 100, 120)
C_YOU = (240, 160,  40)
C_TRL = ( 60, 120, 180)
 
# Map pane
MX, MY, MW, MH = 210, 40, 255, 360
MCX, MCY = MX + MW//2, MY + MH//2
SCALE = 15
 
def w2s(x, z):
    return int(MCX + x*SCALE), int(MCY - z*SCALE)
 
trail = []
 
# ── Main loop ─────────────────────────────────────────────────
running = True
while running:
 
    with S.lock:
        phase    = S.phase
        active   = S.land_active
        pose_ok  = S.pose_ok
        tag_vis  = S.tag_visible
        tx, ty   = S.tag_x, S.tag_y
        tdist    = S.tag_dist
        cur      = S.pos.copy()
        fmode    = S.flight_mode
 
    dist = float(np.linalg.norm(cur - T0))
 
    # Events
    for ev in pygame.event.get():
        if ev.type == pygame.QUIT:
            running = False
        if ev.type == pygame.KEYDOWN:
            if ev.key == pygame.K_q:
                running = False
            elif ev.key == pygame.K_l:
                with S.lock:
                    if not S.land_active and S.pose_ok:
                        S.land_active = True
                        print("Landing activated")
                        if ros_node: ros_node.publish_land()
                    elif not S.pose_ok:
                        print("No pose yet — waiting for ZED")
            elif ev.key == pygame.K_c:
                trail.clear()
 
    # Trail
    if pose_ok:
        trail.append((cur[0], cur[2]))
        if len(trail) > 600: trail.pop(0)
 
    # ── Draw ──────────────────────────────────────────────────
    screen.fill(BG)
 
    # Left panel background
    pygame.draw.rect(screen, PANEL, (0, 0, 205, H))
 
    # Map border + grid
    pygame.draw.rect(screen, GR, (MX, MY, MW, MH), 1)
    for g in range(-20, 21, 5):
        sx, sy = w2s(g, 0)
        if MX < sx < MX+MW:
            pygame.draw.line(screen, GR, (sx, MY), (sx, MY+MH))
        sx, sy = w2s(0, g)
        if MY < sy < MY+MH:
            pygame.draw.line(screen, GR, (MX, sy), (MX+MW, sy))
 
    # Trail
    if len(trail) > 1:
        pts = [w2s(x,z) for x,z in trail]
        pts = [(x,y) for x,y in pts if MX<x<MX+MW and MY<y<MY+MH]
        if len(pts) > 1:
            pygame.draw.lines(screen, C_TRL, False, pts, 2)
 
    # Home marker
    hx, hy = w2s(T0[0], T0[2])
    if MX < hx < MX+MW and MY < hy < MY+MH:
        pygame.draw.circle(screen, C_OK,  (hx, hy), 11)
        pygame.draw.circle(screen, (255,255,255), (hx, hy), 11, 2)
        screen.blit(F_SM.render("HOME", True, (255,255,255)), (hx-16, hy-24))
        if tag_vis and active:
            pygame.draw.circle(screen, C_TAG, (hx, hy), 18, 2)
 
    # Drone marker
    if pose_ok:
        dx, dy = w2s(cur[0], cur[2])
        if MX < dx < MX+MW and MY < dy < MY+MH:
            col = C_TAG if active else C_YOU
            pygame.draw.circle(screen, col, (dx, dy), 9)
            pygame.draw.circle(screen, (255,255,255), (dx, dy), 9, 2)
            screen.blit(F_SM.render("UAV", True, (255,255,255)), (dx-10, dy-22))
    else:
        screen.blit(F_MD.render("No ZED pose", True, C_WRN), (MX+10, MCY-10))
 
    # Phase banner at bottom of map
    ph_txt   = ["", "PH1: NAVIGATE", "PH2: APRILTAG", "PH3: LANDING"][phase]
    ph_col   = [C_TXT, C_TXT, C_TAG, C_TAG][phase]
    screen.blit(F_MD.render(ph_txt, True, ph_col), (MX+5, MY+MH+6))
 
    # ── Left panel HUD ────────────────────────────────────────
    def row(text, y, fnt=F_SM, col=C_TXT):
        screen.blit(fnt.render(text, True, col), (10, y))
 
    row("ZED LAND CTRL", 10, F_LG, C_TXT)
    row("─"*22,           32, F_SM, GR)
 
    row("POSITION (m)",   44, F_SM, C_DIM)
    row(f"X {cur[0]:+.3f}", 59, F_SM, C_TXT)
    row(f"Y {cur[1]:+.3f}", 73, F_SM, C_TXT)
    row(f"Z {cur[2]:+.3f}", 87, F_SM, C_TXT)
 
    row("HOME (m)",      104, F_SM, C_DIM)
    row(f"X {T0[0]:+.3f}", 119, F_SM, C_OK)
    row(f"Y {T0[1]:+.3f}", 133, F_SM, C_OK)
    row(f"Z {T0[2]:+.3f}", 147, F_SM, C_OK)
 
    row("DIST TO HOME",  163, F_SM, C_DIM)
    d_col = C_OK if dist < 2 else C_WRN if dist < 5 else (180,60,60)
    row(f"{dist:.2f} m",  178, F_MD, d_col)
 
    row("APRILTAG",      202, F_SM, C_DIM)
    if tag_vis:
        row(f"X {tx:+.3f}m",  217, F_SM, C_TAG)
        row(f"Y {ty:+.3f}m",  231, F_SM, C_TAG)
        row(f"D {tdist:.2f}m",245, F_SM, C_TAG)
    else:
        row("not visible",    217, F_SM, C_DIM)
 
    row("MAVLink",        265, F_SM, C_DIM)
    row(fmode if mav else "DISCONNECTED",
                          280, F_SM, C_OK if mav else C_WRN)
 
    row("LAND",           300, F_SM, C_DIM)
    row("ACTIVE ✓" if active else "press L",
                          315, F_MD, C_TAG if active else C_DIM)
 
    row("─"*22,           338, F_SM, GR)
    row("[L] Land  [C] Clear  [Q] Quit", 350, F_SM, C_DIM)
 
    pygame.display.flip()
    clock.tick(30)
 
pygame.quit()
sys.exit()