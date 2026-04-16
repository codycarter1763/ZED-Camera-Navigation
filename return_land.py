# #!/usr/bin/env python3
# # ─────────────────────────────────────────────────────────────
# # return_land.py  —  PROGRAM 3
# # Full real-hardware version using ZED ROS2 wrapper topics.
# #
# # ZED topics subscribed:
# #   /zed/zed_node/pose                        — position tracking (SLAM)
# #   /zed/zed_node/depth/depth_registered      — real depth map (float32, metres)
# #
# # AprilTag topic subscribed:
# #   /detections  (apriltag_msgs/AprilTagDetectionArray)
# #
# # MAVLink topics published:
# #   velocity setpoints (BODY_NED) at 10 Hz in GUIDED mode
# #   MAV_CMD_NAV_LAND when aligned on Phase 3
# #   MAV_CMD_DO_SET_MODE → GUIDED on auto-activate
# #
# # Fully automatic — no key presses needed during normal flight.
# #
# # Usage:   python3 "return_land.py"
# # Controls: L = force activate    C = clear trail    Q = quit
# # Requires: origin_T0.json written by set_origin.py
# # Folder:   ~/ZED Navigation/
# # ─────────────────────────────────────────────────────────────

# import json, math, os, sys, threading, time
# import numpy as np
# import pygame
# from pymavlink import mavutil

# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import PoseStamped, PointStamped
# from apriltag_msgs.msg import AprilTagDetectionArray
# from sensor_msgs.msg import Image
# from std_msgs.msg import Bool

# # ══════════════════════════════════════════════════════════════
# # CONFIGURATION
# # ══════════════════════════════════════════════════════════════
# SCRIPT_DIR  = os.path.dirname(os.path.abspath(__file__))
# ORIGIN_FILE = os.path.join(SCRIPT_DIR, "origin_T0.json")

# # ── MAVLink ───────────────────────────────────────────────────
# MAV_CONNECT  = '/dev/ttyTHS2'    # Jetson Orin Nano UART → Pixhawk TELEM2
# MAV_BAUD     = 921600            # Match SERIAL2_BAUD in ArduPilot params
# GUIDED_MODE  = 4                 # ArduCopter GUIDED mode number

# # ── ZED ROS2 wrapper topic names ─────────────────────────────
# # These match the default zed_wrapper namespace: /zed/zed_node/
# ZED_POSE_TOPIC  = '/zed/zed_node/pose'
# ZED_DEPTH_TOPIC = '/zed/zed_node/depth/depth_registered'
# APRILTAG_TOPIC  = '/detections'

# # ── Tag config — must match tags_config.yaml ─────────────────
# TAG_ID       = 0
# TAG_SIZE     = 0.15      # metres — must match standalone_tags size in yaml

# # ── ZED image dimensions ─────────────────────────────────────
# # MUST match general.grab_resolution in zed_apriltag.launch.py
# # ZED actual resolutions (landing_controller.py used 960x540
# # which does NOT match any real ZED mode — HD720 is correct):
# #
# #   Resolution  IMG_W  IMG_H
# #   HD1080      1920   1080
# #   HD720       1280    720   ← DEFAULT (set in zed_apriltag.launch.py)
# #   VGA          672    376
# #
# # Update both values if you change grab_resolution in the launch file.
# IMG_W        = 1280
# IMG_H        =  720

# # ── Flight control ────────────────────────────────────────────
# NAV_SPEED    = 1.0      # m/s  Phase 1 cruise speed toward T0
# TAG_GAIN     = 0.8      # P-gain Phase 2 lateral alignment
# DEADBAND     = 0.05     # metres alignment deadband
# LAND_DIST    = 0.30     # metres — commit to land below this altitude
# HOME_RADIUS  = 5.0      # metres — auto-activate within this dist of T0
# ALT_FLOOR    = 0.5      # metres — never descend below this in Phase 2
# # ══════════════════════════════════════════════════════════════

# # ── Load origin ───────────────────────────────────────────────
# if not os.path.exists(ORIGIN_FILE):
#     print(f"ERROR: {ORIGIN_FILE} not found.")
#     print("Run set_origin.py first to record the landing pad position.")
#     sys.exit(1)

# with open(ORIGIN_FILE) as f:
#     _o = json.load(f)
# T0 = np.array([_o["x"], _o["y"], _o["z"]])
# print(f"[return_land] Origin loaded — X:{T0[0]:.3f} Y:{T0[1]:.3f} Z:{T0[2]:.3f}")

# # ── MAVLink connection ────────────────────────────────────────
# print(f"[return_land] Connecting MAVLink → {MAV_CONNECT} @ {MAV_BAUD}...")
# try:
#     mav = mavutil.mavlink_connection(MAV_CONNECT, baud=MAV_BAUD)
#     mav.wait_heartbeat(timeout=5)
#     print(f"[return_land] MAVLink OK — "
#           f"sys:{mav.target_system} comp:{mav.target_component}")
# except Exception as e:
#     print(f"[return_land] WARNING: MAVLink failed ({e})")
#     print("Velocity/land commands disabled — running in display-only mode.")
#     mav = None

# # ── Shared state ──────────────────────────────────────────────
# class State:
#     def __init__(self):
#         self.lock            = threading.Lock()
#         # ZED pose
#         self.pos             = np.zeros(3)
#         self.pose_ok         = False
#         self.pose_time       = 0.0
#         # AprilTag
#         self.tag_visible     = False
#         self.tag_x           = 0.0    # metres lateral offset (camera frame)
#         self.tag_y           = 0.0    # metres lateral offset (camera frame)
#         self.tag_dist_visual = 0.0    # distance from focal length formula
#         self.tag_dist_depth  = 0.0    # distance from ZED depth sensor
#         self.tag_dist        = 0.0    # whichever is active (depth preferred)
#         self.tag_cx          = 0      # tag centre pixel X
#         self.tag_cy          = 0      # tag centre pixel Y
#         self.depth_available = False
#         # Flight
#         self.phase           = 1
#         self.land_active     = False
#         self.auto_activated  = False
#         self.guided_ok       = False
#         self.flight_mode     = "UNKNOWN"
#         self.hb_time         = time.time()
#         self.battery_pct     = 100
# S = State()

# # ── MAVLink helpers ───────────────────────────────────────────
# def _clamp(v, lim): return max(-lim, min(lim, v))

# def switch_to_guided():
#     """Send GUIDED mode command and verify ACK."""
#     if mav is None: return False
#     print("[return_land] Sending GUIDED mode command...")
#     mav.mav.command_long_send(
#         mav.target_system, mav.target_component,
#         mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
#         mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
#         GUIDED_MODE, 0, 0, 0, 0, 0)
#     ack = mav.recv_match(type='COMMAND_ACK', blocking=True, timeout=2)
#     if ack and ack.result == 0:
#         print("[return_land] GUIDED mode confirmed ✓")
#         with S.lock:
#             S.guided_ok = True
#         return True
#     # Retry once
#     print("[return_land] ACK failed — retrying GUIDED command...")
#     time.sleep(0.3)
#     mav.mav.command_long_send(
#         mav.target_system, mav.target_component,
#         mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
#         mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
#         GUIDED_MODE, 0, 0, 0, 0, 0)
#     return False

# def send_velocity(vx, vy, vz):
#     """Body-NED velocity setpoint. Call at 10 Hz."""
#     if mav is None: return
#     mav.mav.set_position_target_local_ned_send(
#         0,
#         mav.target_system, mav.target_component,
#         mavutil.mavlink.MAV_FRAME_BODY_NED,
#         0b0000_1111_1100_0111,
#         0, 0, 0,
#         vx, vy, vz,
#         0, 0, 0, 0, 0)

# def send_land():
#     """Send MAV_CMD_NAV_LAND."""
#     if mav is None:
#         print("[return_land] WARNING: No MAVLink — land skipped"); return
#     mav.mav.command_long_send(
#         mav.target_system, mav.target_component,
#         mavutil.mavlink.MAV_CMD_NAV_LAND,
#         0, 0, 0, 0, 0, 0, 0, 0)
#     print("[return_land] MAV_CMD_NAV_LAND sent ✓")

# # ── ROS2 Node ─────────────────────────────────────────────────
# class LandNode(Node):
#     def __init__(self):
#         super().__init__('return_land')

#         # ZED pose — position tracking / SLAM output
#         self.create_subscription(
#             PoseStamped, ZED_POSE_TOPIC, self._pose_cb, 10)

#         # ZED depth map — float32 metres, same resolution as RGB
#         self.create_subscription(
#             Image, ZED_DEPTH_TOPIC, self._depth_cb, 10)

#         # AprilTag detections from apriltag_ros
#         self.create_subscription(
#             AprilTagDetectionArray, APRILTAG_TOPIC, self._tag_cb, 10)

#         # Publishers
#         self._land_pub    = self.create_publisher(Bool,         '/land_command',        10)
#         self._tag_pos_pub = self.create_publisher(PointStamped, '/apriltag_global_pos',  10)

#         self.get_logger().info('LandNode ready')
#         self.get_logger().info(f'Pose:    {ZED_POSE_TOPIC}')
#         self.get_logger().info(f'Depth:   {ZED_DEPTH_TOPIC}')
#         self.get_logger().info(f'AprilTag:{APRILTAG_TOPIC}')

#     # ── Pose callback ─────────────────────────────────────────
#     def _pose_cb(self, msg):
#         p = msg.pose.position
#         with S.lock:
#             S.pos       = np.array([p.x, p.y, p.z])
#             S.pose_ok   = True
#             S.pose_time = time.time()
#             dist        = float(np.linalg.norm(S.pos - T0))

#             # Auto-activate when drone enters HOME_RADIUS
#             if not S.land_active and dist < HOME_RADIUS:
#                 S.land_active    = True
#                 S.auto_activated = True
#                 print(f"[return_land] AUTO-ACTIVATED — dist={dist:.2f}m")
#                 threading.Thread(
#                     target=switch_to_guided, daemon=True).start()

#             # Phase 1 → 2 when close enough for tag search
#             if S.phase == 1 and S.land_active and dist < (HOME_RADIUS * 0.4):
#                 S.phase = 2
#                 print("[return_land] Phase 1 → 2: AprilTag search active")

#     # ── Depth callback ────────────────────────────────────────
#     def _depth_cb(self, msg):
#         """
#         Read ZED depth image (32-bit float, metres).
#         Samples depth at the last known tag centre pixel.
#         Only runs when tag is visible to avoid wasted work.
#         """
#         with S.lock:
#             if not S.tag_visible:
#                 return
#             cx = S.tag_cx
#             cy = S.tag_cy

#         # Bounds check
#         if not (0 < cx < msg.width and 0 < cy < msg.height):
#             return

#         try:
#             # ZED depth is 32FC1 — 4 bytes per pixel
#             row_offset  = cy * msg.step
#             col_offset  = cx * 4
#             raw         = msg.data[row_offset + col_offset:
#                                    row_offset + col_offset + 4]
#             depth_m     = float(np.frombuffer(bytes(raw), dtype=np.float32)[0])

#             # ZED returns nan/inf for invalid measurements
#             if math.isfinite(depth_m) and 0.05 < depth_m < 20.0:
#                 with S.lock:
#                     S.tag_dist_depth  = depth_m
#                     S.tag_dist        = depth_m   # prefer real depth
#                     S.depth_available = True
#         except Exception:
#             pass   # depth read failed — fall back to visual estimate

#     # ── AprilTag callback ─────────────────────────────────────
#     def _tag_cb(self, msg):
#         with S.lock:
#             if not S.land_active:
#                 S.tag_visible = False
#                 return
#             S.tag_visible = False

#             for det in msg.detections:
#                 if det.id != TAG_ID:
#                     continue

#                 S.tag_visible = True

#                 # Early Phase 1 → 2 transition if tag already visible
#                 # Mirrors landing_controller.py behaviour — don't wait for
#                 # HOME_RADIUS * 0.4 distance if the tag is already in frame
#                 if S.phase == 1 and S.land_active:
#                     S.phase = 2
#                     print("[return_land] Tag visible in Phase 1 — early jump to Phase 2")

#                 # Pixel offset from image centre
#                 px_x = det.centre.x - IMG_W / 2.0
#                 px_y = det.centre.y - IMG_H / 2.0

#                 # Save pixel centre for depth lookup
#                 S.tag_cx = int(det.centre.x)
#                 S.tag_cy = int(det.centre.y)

#                 # Tag pixel width → metric offset + visual distance estimate
#                 c    = det.corners
#                 w_px = math.hypot(c[1].x - c[0].x, c[1].y - c[0].y)

#                 if w_px > 0:
#                     ppm         = w_px / TAG_SIZE
#                     S.tag_x     = px_x / ppm
#                     S.tag_y     = px_y / ppm
#                     # Visual distance estimate (fallback if depth unavailable)
#                     # fx ≈ IMG_W * 0.8 is a reasonable approximation for HD720
#                     fx_approx           = IMG_W * 0.8
#                     S.tag_dist_visual   = (TAG_SIZE * fx_approx) / w_px
#                     # Only use visual if depth isn't fresh
#                     if not S.depth_available:
#                         S.tag_dist = S.tag_dist_visual

#                     # Debug output — mirrors landing_controller.py control_loop prints
#                     x_action = ("OK" if abs(S.tag_x) < DEADBAND
#                                 else f"move {'RIGHT' if S.tag_x > 0 else 'LEFT'} ({abs(S.tag_x):.3f}m)")
#                     y_action = ("OK" if abs(S.tag_y) < DEADBAND
#                                 else f"move {'DOWN' if S.tag_y > 0 else 'UP'} ({abs(S.tag_y):.3f}m)")
#                     z_err    = S.tag_dist - LAND_DIST
#                     z_action = ("OK" if abs(z_err) < DEADBAND
#                                 else f"{'DESCEND' if z_err > 0 else 'ASCEND'} ({abs(z_err):.3f}m)")
#                     print(f"[tag] X:{x_action}  Y:{y_action}  Z:{z_action}  "
#                           f"dist:{S.tag_dist:.3f}m  w_px:{w_px:.1f}")
#                 else:
#                     S.tag_x = S.tag_y = 0.0

#                 # Publish global tag position for logging
#                 tg_x = S.pos[0] + S.tag_x
#                 tg_z = S.pos[2] + S.tag_y
#                 threading.Thread(
#                     target=self._pub_tag_pos,
#                     args=(tg_x, tg_z), daemon=True).start()

#                 # Phase 2 → 3: aligned and close enough
#                 if (S.phase == 2 and
#                         abs(S.tag_x) < DEADBAND and
#                         abs(S.tag_y) < DEADBAND and
#                         S.tag_dist   < LAND_DIST):
#                     S.phase = 3
#                     print("[return_land] Phase 2 → 3: LANDING")
#                 break

#     def _pub_tag_pos(self, x, z):
#         pt = PointStamped()
#         pt.header.frame_id = "map"
#         pt.header.stamp    = self.get_clock().now().to_msg()
#         pt.point.x = x
#         pt.point.y = 0.0
#         pt.point.z = z
#         self._tag_pos_pub.publish(pt)

#     def publish_land_cmd(self):
#         msg = Bool(); msg.data = True
#         self._land_pub.publish(msg)
#         self.get_logger().info('Land command published on /land_command')

# ros_node: LandNode = None

# def _ros_spin():
#     global ros_node
#     rclpy.init()
#     ros_node = LandNode()
#     rclpy.spin(ros_node)
#     ros_node.destroy_node()
#     rclpy.shutdown()

# threading.Thread(target=_ros_spin, daemon=True).start()
# time.sleep(1.0)

# # ── MAVLink control thread — 10 Hz ───────────────────────────
# def _mav_thread():
#     land_sent = False
#     while True:
#         time.sleep(0.1)

#         if mav:
#             hb = mav.recv_match(type='HEARTBEAT', blocking=False)
#             if hb:
#                 with S.lock:
#                     S.flight_mode = mav.flightmode
#                     S.hb_time     = time.time()

#             bat = mav.recv_match(type='SYS_STATUS', blocking=False)
#             if bat:
#                 with S.lock:
#                     S.battery_pct = bat.battery_remaining

#             with S.lock:
#                 stale_hb = (time.time() - S.hb_time) > 1.0
#                 mode     = S.flight_mode
#                 batt     = S.battery_pct

#             if batt < 20:
#                 print("[return_land] LOW BATTERY — forcing land")
#                 send_land()
#                 continue

#             if stale_hb or mode != "GUIDED":
#                 continue

#         with S.lock:
#             phase    = S.phase
#             active   = S.land_active
#             pose_ok  = S.pose_ok
#             pose_age = time.time() - S.pose_time
#             tag_vis  = S.tag_visible
#             tx, ty   = S.tag_x, S.tag_y
#             tdist    = S.tag_dist
#             cur      = S.pos.copy()
#             alt      = cur[1]

#         if not active: continue

#         if not pose_ok or pose_age > 0.5:
#             send_velocity(0, 0, 0)
#             continue

#         # Phase 1 — navigate toward T0 using ZED odometry
#         if phase == 1:
#             d     = T0 - cur
#             horiz = math.hypot(d[0], d[2])
#             if horiz > 0.1:
#                 send_velocity(
#                     _clamp(NAV_SPEED * d[2] / horiz, 1.0),
#                     _clamp(NAV_SPEED * d[0] / horiz, 1.0),
#                     0.0)
#             else:
#                 send_velocity(0, 0, 0)

#         # Phase 2 — AprilTag alignment using real depth distance
#         elif phase == 2:
#             if tag_vis:
#                 z_err = tdist - LAND_DIST
#                 vz    = 0.0
#                 if abs(z_err) > DEADBAND:
#                     vz = _clamp(TAG_GAIN * z_err, 0.3)
#                     if alt < ALT_FLOOR and vz > 0:
#                         vz = 0.0
#                 send_velocity(
#                     _clamp(TAG_GAIN * ty, 0.5),
#                     _clamp(TAG_GAIN * tx, 0.5),
#                     vz)
#             else:
#                 send_velocity(0, 0, 0)

#         # Phase 3 — land
#         elif phase == 3:
#             if not land_sent:
#                 send_land()
#                 if ros_node: ros_node.publish_land_cmd()
#                 land_sent = True

# threading.Thread(target=_mav_thread, daemon=True).start()

# # ── Pygame HUD ────────────────────────────────────────────────
# pygame.init()
# W, H   = 560, 480
# screen = pygame.display.set_mode((W, H))
# pygame.display.set_caption("Program 3 — Return & Land  |  L=force  C=clear  Q=quit")
# FSM = pygame.font.SysFont("consolas", 13)
# FMD = pygame.font.SysFont("consolas", 15, bold=True)
# FLG = pygame.font.SysFont("consolas", 20, bold=True)
# clk = pygame.time.Clock()

# BG   = ( 15,  15,  22);  PANEL = ( 25,  25,  38);  GR   = ( 35,  35,  50)
# C_OK = ( 30, 200, 130);  C_WR  = (220, 100,  60);  C_TX = (210, 210, 220)
# C_DM = (100, 100, 120);  C_TG  = (220,  80, 220);  C_YO = (240, 160,  40)
# C_TR = ( 60, 120, 180);  C_BT  = (255, 200,  40);  C_BL = ( 80, 140, 220)

# MX, MY, MW, MH = 240, 40, 305, 390
# MCX = MX + MW // 2
# MCY = MY + MH // 2
# SCALE = 12

# def w2s(x, z):
#     return int(MCX + x * SCALE), int(MCY - z * SCALE)

# trail   = []
# running = True

# while running:
#     with S.lock:
#         phase      = S.phase
#         active     = S.land_active
#         auto_act   = S.auto_activated
#         guided_ok  = S.guided_ok
#         pose_ok    = S.pose_ok
#         tag_vis    = S.tag_visible
#         tx, ty     = S.tag_x, S.tag_y
#         tdist      = S.tag_dist
#         tdist_vis  = S.tag_dist_visual
#         tdist_dep  = S.tag_dist_depth
#         depth_avail= S.depth_available
#         cur        = S.pos.copy()
#         fmode      = S.flight_mode
#         battery    = S.battery_pct

#     dist = float(np.linalg.norm(cur - T0))

#     for ev in pygame.event.get():
#         if ev.type == pygame.QUIT: running = False
#         if ev.type == pygame.KEYDOWN:
#             if ev.key == pygame.K_q: running = False
#             elif ev.key == pygame.K_c: trail.clear()
#             elif ev.key == pygame.K_l:
#                 with S.lock:
#                     if not S.land_active and S.pose_ok:
#                         S.land_active    = True
#                         S.auto_activated = False
#                         print("[return_land] Manual activation")
#                         threading.Thread(
#                             target=switch_to_guided, daemon=True).start()
#                     elif not S.pose_ok:
#                         print("[return_land] No ZED pose yet")

#     if pose_ok:
#         trail.append((cur[0], cur[2]))
#         if len(trail) > 600: trail.pop(0)

#     # ── Draw ──────────────────────────────────────────────────
#     screen.fill(BG)
#     pygame.draw.rect(screen, PANEL, (0, 0, 235, H))

#     # Map grid
#     pygame.draw.rect(screen, GR, (MX, MY, MW, MH), 1)
#     for g in range(-20, 21, 5):
#         sx, _ = w2s(g, 0)
#         if MX < sx < MX+MW:
#             pygame.draw.line(screen, GR, (sx, MY), (sx, MY+MH))
#         _, sy = w2s(0, g)
#         if MY < sy < MY+MH:
#             pygame.draw.line(screen, GR, (MX, sy), (MX+MW, sy))

#     # Trail
#     if len(trail) > 1:
#         pts = [w2s(x, z) for x, z in trail]
#         pts = [(x, y) for x, y in pts if MX < x < MX+MW and MY < y < MY+MH]
#         if len(pts) > 1:
#             pygame.draw.lines(screen, C_TR, False, pts, 2)

#     # HOME_RADIUS circle
#     r_px = int(HOME_RADIUS * SCALE)
#     pygame.draw.circle(screen, GR, (MCX, MCY), r_px, 1)

#     # Home marker
#     hx, hy = w2s(T0[0], T0[2])
#     if MX < hx < MX+MW and MY < hy < MY+MH:
#         pygame.draw.circle(screen, C_OK, (hx, hy), 11)
#         pygame.draw.circle(screen, (255, 255, 255), (hx, hy), 11, 2)
#         screen.blit(FSM.render("HOME", True, (255,255,255)), (hx-16, hy-24))
#         if tag_vis and active:
#             pygame.draw.circle(screen, C_TG, (hx, hy), 22, 2)

#     # Drone marker
#     if pose_ok:
#         dx, dy = w2s(cur[0], cur[2])
#         if MX < dx < MX+MW and MY < dy < MY+MH:
#             col = C_TG if active else C_YO
#             pygame.draw.circle(screen, col, (dx, dy), 9)
#             pygame.draw.circle(screen, (255,255,255), (dx, dy), 9, 2)
#             screen.blit(FSM.render("UAV", True, (255,255,255)), (dx-10, dy-22))
#     else:
#         screen.blit(FMD.render("No ZED pose", True, C_WR), (MX+10, MCY-10))

#     # Phase banner
#     ph_txt = ["", "PH1: NAVIGATE", "PH2: APRILTAG", "PH3: LANDING"][phase]
#     ph_col = [C_TX, C_TX, C_TG, C_TG][phase]
#     screen.blit(FMD.render(ph_txt, True, ph_col), (MX+5, MY+MH+6))

#     # ── Left HUD panel ────────────────────────────────────────
#     def row(t, y, f=FSM, c=C_TX):
#         screen.blit(f.render(t, True, c), (8, y))

#     row("RETURN & LAND",    8,  FLG, C_TX)
#     row("─"*26,            30,  FSM, GR)

#     row("ZED POSE",         42, FSM, C_DM)
#     pc = C_OK if pose_ok else C_WR
#     row("LIVE ✓" if pose_ok else "NO SIGNAL", 57, FMD, pc)
#     row(f"X {cur[0]:+.3f}", 74, FSM, C_TX)
#     row(f"Y {cur[1]:+.3f}", 88, FSM, C_TX)
#     row(f"Z {cur[2]:+.3f}",102, FSM, C_TX)

#     row("ORIGIN",          118, FSM, C_DM)
#     row(f"X {T0[0]:+.3f}", 133, FSM, C_OK)
#     row(f"Z {T0[2]:+.3f}", 147, FSM, C_OK)

#     row("DIST TO HOME",    163, FSM, C_DM)
#     dc = C_OK if dist < 2 else C_WR if dist < 5 else (180,60,60)
#     row(f"{dist:.2f} m",   178, FMD, dc)

#     row("─"*26,            196, FSM, GR)
#     row("APRILTAG",        206, FSM, C_DM)
#     if tag_vis:
#         row(f"X {tx:+.3f}m", 221, FSM, C_TG)
#         row(f"Y {ty:+.3f}m", 235, FSM, C_TG)
#         # Show which distance source is active
#         dep_lbl = "ZED depth" if depth_avail else "visual est"
#         row(f"D {tdist:.3f}m ({dep_lbl})", 249, FSM, C_BL if depth_avail else C_TG)
#         if depth_avail:
#             row(f"  vis:{tdist_vis:.3f} dep:{tdist_dep:.3f}", 263, FSM, C_DM)
#     else:
#         row("not visible",   221, FSM, C_DM)

#     row("─"*26,            283, FSM, GR)
#     row("MAVLink",         293, FSM, C_DM)
#     mc = C_OK if (mav and fmode == "GUIDED") else C_WR
#     row(fmode if mav else "DISCONNECTED", 308, FSM, mc)

#     row("GUIDED CMD",      324, FSM, C_DM)
#     row("SENT ✓" if guided_ok else "Pending...",
#                            339, FMD, C_OK if guided_ok else C_DM)

#     row("BATTERY",         355, FSM, C_DM)
#     bc = C_OK if battery > 50 else C_BT if battery > 20 else (180,60,60)
#     row(f"{battery}%",     370, FMD, bc)

#     row("STATUS",          386, FSM, C_DM)
#     lbl = ("AUTO ✓" if auto_act else "MANUAL ✓") if active else "Standby"
#     row(lbl,               401, FMD, C_TG if active else C_DM)

#     row("─"*26,            420, FSM, GR)
#     row("[L] Force  [C] Clear  [Q] Quit", 432, FSM, C_DM)

#     pygame.display.flip()
#     clk.tick(30)

# pygame.quit()
# sys.exit()

#!/usr/bin/env python3
# ─────────────────────────────────────────────────────────────
# return_land.py  —  PROGRAM 3
# Full real-hardware version using ZED ROS2 wrapper topics.
#
# ZED topics subscribed:
#   /zed/zed_node/pose                        — position tracking (SLAM)
#   /zed/zed_node/depth/depth_registered      — real depth map (float32, metres)
#
# AprilTag topic subscribed:
#   /detections  (apriltag_msgs/AprilTagDetectionArray)
#
# MAVLink topics published:
#   velocity setpoints (BODY_NED) at 10 Hz in GUIDED mode
#   MAV_CMD_NAV_LAND when aligned on Phase 3
#   MAV_CMD_DO_SET_MODE → GUIDED on auto-activate
#
# Fully automatic — no key presses needed during normal flight.
#
# Usage:   python3 "return_land.py"
# Controls: L = force activate    C = clear trail    Q = quit
# Requires: origin_T0.json written by set_origin.py
# Folder:   ~/ZED Navigation/
# ─────────────────────────────────────────────────────────────

import json, math, os, sys, threading, time
import struct
import numpy as np
import pygame
from pymavlink import mavutil
import serial

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PointStamped
from apriltag_msgs.msg import AprilTagDetectionArray
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

# ══════════════════════════════════════════════════════════════
# CONFIGURATION
# ══════════════════════════════════════════════════════════════
SCRIPT_DIR  = os.path.dirname(os.path.abspath(__file__))
ORIGIN_FILE = os.path.join(SCRIPT_DIR, "origin_T0.json")

# ── MAVLink ───────────────────────────────────────────────────
MAV_CONNECT  = '/dev/ttyTHS2'    # Jetson Orin Nano UART → Pixhawk TELEM2
MAV_BAUD     = 921600            # Match SERIAL2_BAUD in ArduPilot params
GUIDED_MODE  = 4                 # ArduCopter GUIDED mode number

# ── ZED ROS2 wrapper topic names ─────────────────────────────
# These match the default zed_wrapper namespace: /zed/zed_node/
ZED_POSE_TOPIC  = '/zed/zed_node/pose'
ZED_DEPTH_TOPIC = '/zed/zed_node/depth/depth_registered'
APRILTAG_TOPIC  = '/detections'

# ── Tag config — must match tags_config.yaml ─────────────────
TAG_ID       = 0
TAG_SIZE     = 0.15      # metres — must match standalone_tags size in yaml

# ── ZED image dimensions ─────────────────────────────────────
# MUST match general.grab_resolution in zed_apriltag.launch.py
# ZED actual resolutions (landing_controller.py used 960x540
# which does NOT match any real ZED mode — HD720 is correct):
#
#   Resolution  IMG_W  IMG_H
#   HD1080      1920   1080
#   HD720       1280    720   ← DEFAULT (set in zed_apriltag.launch.py)
#   VGA          672    376
#
# Update both values if you change grab_resolution in the launch file.
IMG_W        = 1280
IMG_H        =  720

# ── Flight control ────────────────────────────────────────────
NAV_SPEED    = 1.0      # m/s  Phase 1 cruise speed toward T0
TAG_GAIN     = 0.8      # P-gain Phase 2 lateral alignment
DEADBAND     = 0.05     # metres alignment deadband
LAND_DIST    = 0.30     # metres — commit to land below this altitude
HOME_RADIUS  = 5.0      # metres — auto-activate within this dist of T0
ALT_FLOOR    = 0.5      # metres — never descend below this in Phase 2

# ── Gimbal config ─────────────────────────────────────────────
GIMBAL_PORT      = '/dev/ttyACM0'   # SimpleBGC serial port
GIMBAL_BAUD      = 115200
GIMBAL_FORWARD   =   0.0            # degrees — camera faces forward
GIMBAL_SEARCH    =  45.0            # degrees — angled down for approach
GIMBAL_NADIR     =  90.0            # degrees — straight down for AprilTag
# ══════════════════════════════════════════════════════════════

# ── Load origin ───────────────────────────────────────────────
if not os.path.exists(ORIGIN_FILE):
    print(f"ERROR: {ORIGIN_FILE} not found.")
    print("Run set_origin.py first to record the landing pad position.")
    sys.exit(1)

with open(ORIGIN_FILE) as f:
    _o = json.load(f)
T0 = np.array([_o["x"], _o["y"], _o["z"]])
print(f"[return_land] Origin loaded — X:{T0[0]:.3f} Y:{T0[1]:.3f} Z:{T0[2]:.3f}")

# ── MAVLink connection ────────────────────────────────────────
print(f"[return_land] Connecting MAVLink → {MAV_CONNECT} @ {MAV_BAUD}...")
try:
    mav = mavutil.mavlink_connection(MAV_CONNECT, baud=MAV_BAUD)
    mav.wait_heartbeat(timeout=5)
    print(f"[return_land] MAVLink OK — "
          f"sys:{mav.target_system} comp:{mav.target_component}")
except Exception as e:
    print(f"[return_land] WARNING: MAVLink failed ({e})")
    print("Velocity/land commands disabled — running in display-only mode.")
    mav = None

# ── Gimbal connection ─────────────────────────────────────────
print(f"[gimbal] Connecting → {GIMBAL_PORT} @ {GIMBAL_BAUD}...")
try:
    gimbal_ser = serial.Serial(GIMBAL_PORT, GIMBAL_BAUD, timeout=1)
    time.sleep(2)
    print("[gimbal] Connected ✓")
except Exception as e:
    print(f"[gimbal] WARNING: Gimbal failed ({e}) — gimbal control disabled")
    gimbal_ser = None

def _gimbal_send_cmd(cmd, data=bytes([])):
    """Send a SimpleBGC serial command."""
    if gimbal_ser is None: return
    size            = len(data)
    header_checksum = (cmd + size) & 0xFF
    body_checksum   = 0
    for b in data:
        body_checksum = (body_checksum + b) & 0xFF
    packet = bytes([0x3E, cmd, size, header_checksum]) + data + bytes([body_checksum])
    gimbal_ser.write(packet)
    time.sleep(0.1)

def gimbal_set_pitch(angle_deg):
    """
    Set gimbal pitch angle in degrees.
      0   = forward (level)
      45  = angled down (search)
      90  = straight down (nadir — AprilTag detection)
    """
    if gimbal_ser is None:
        print(f"[gimbal] Skipped — no connection (would set {angle_deg}°)")
        return
    try:
        # Mode 2 = angle mode, send target angle
        mode  = 2
        pitch = int(angle_deg / 0.02197265625)
        data  = struct.pack('<Bhhhhhh', mode, 0, 0, 300, pitch, 0, 0)
        _gimbal_send_cmd(67, data)
        time.sleep(0.5)
        # Release to hold position
        mode = 0
        data = struct.pack('<Bhhhhhh', mode, 0, 0, 0, 0, 0, 0)
        _gimbal_send_cmd(67, data)
        print(f"[gimbal] Pitch set to {angle_deg}°")
    except Exception as e:
        print(f"[gimbal] ERROR setting pitch: {e}")

def gimbal_motors_on():
    """Turn gimbal motors on."""
    if gimbal_ser is None: return
    _gimbal_send_cmd(77)
    time.sleep(1)
    print("[gimbal] Motors on")

# Initialise gimbal on startup
if gimbal_ser:
    threading.Thread(target=lambda: (
        gimbal_motors_on(),
        gimbal_set_pitch(GIMBAL_FORWARD)
    ), daemon=True).start()

# ── Shared state ──────────────────────────────────────────────
class State:
    def __init__(self):
        self.lock            = threading.Lock()
        # ZED pose
        self.pos             = np.zeros(3)
        self.pose_ok         = False
        self.pose_time       = 0.0
        # AprilTag
        self.tag_visible     = False
        self.tag_x           = 0.0    # metres lateral offset (camera frame)
        self.tag_y           = 0.0    # metres lateral offset (camera frame)
        self.tag_dist_visual = 0.0    # distance from focal length formula
        self.tag_dist_depth  = 0.0    # distance from ZED depth sensor
        self.tag_dist        = 0.0    # whichever is active (depth preferred)
        self.tag_cx          = 0      # tag centre pixel X
        self.tag_cy          = 0      # tag centre pixel Y
        self.depth_available = False
        # Flight
        self.phase           = 1
        self.land_active     = False
        self.auto_activated  = False
        self.guided_ok       = False
        self.flight_mode     = "UNKNOWN"
        self.hb_time         = time.time()
        self.battery_pct     = 100
        # Gimbal
        self.gimbal_angle    = 0.0      # current gimbal pitch degrees
        self.gimbal_ok       = gimbal_ser is not None
S = State()

# ── MAVLink helpers ───────────────────────────────────────────
def _clamp(v, lim): return max(-lim, min(lim, v))

def switch_to_guided():
    """Send GUIDED mode command and verify ACK."""
    if mav is None: return False
    print("[return_land] Sending GUIDED mode command...")
    mav.mav.command_long_send(
        mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        GUIDED_MODE, 0, 0, 0, 0, 0)
    ack = mav.recv_match(type='COMMAND_ACK', blocking=True, timeout=2)
    if ack and ack.result == 0:
        print("[return_land] GUIDED mode confirmed ✓")
        with S.lock:
            S.guided_ok = True
        return True
    # Retry once
    print("[return_land] ACK failed — retrying GUIDED command...")
    time.sleep(0.3)
    mav.mav.command_long_send(
        mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        GUIDED_MODE, 0, 0, 0, 0, 0)
    return False

def send_velocity(vx, vy, vz):
    """Body-NED velocity setpoint. Call at 10 Hz."""
    if mav is None: return
    mav.mav.set_position_target_local_ned_send(
        0,
        mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        0b0000_1111_1100_0111,
        0, 0, 0,
        vx, vy, vz,
        0, 0, 0, 0, 0)

def send_land():
    """Send MAV_CMD_NAV_LAND."""
    if mav is None:
        print("[return_land] WARNING: No MAVLink — land skipped"); return
    mav.mav.command_long_send(
        mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0, 0, 0, 0, 0, 0, 0, 0)
    print("[return_land] MAV_CMD_NAV_LAND sent ✓")

# ── ROS2 Node ─────────────────────────────────────────────────
class LandNode(Node):
    def __init__(self):
        super().__init__('return_land')

        # ZED pose — position tracking / SLAM output
        self.create_subscription(
            PoseStamped, ZED_POSE_TOPIC, self._pose_cb, 10)

        # ZED depth map — float32 metres, same resolution as RGB
        self.create_subscription(
            Image, ZED_DEPTH_TOPIC, self._depth_cb, 10)

        # AprilTag detections from apriltag_ros
        self.create_subscription(
            AprilTagDetectionArray, APRILTAG_TOPIC, self._tag_cb, 10)

        # Publishers
        self._land_pub    = self.create_publisher(Bool,         '/land_command',        10)
        self._tag_pos_pub = self.create_publisher(PointStamped, '/apriltag_global_pos',  10)

        self.get_logger().info('LandNode ready')
        self.get_logger().info(f'Pose:    {ZED_POSE_TOPIC}')
        self.get_logger().info(f'Depth:   {ZED_DEPTH_TOPIC}')
        self.get_logger().info(f'AprilTag:{APRILTAG_TOPIC}')

    # ── Pose callback ─────────────────────────────────────────
    def _pose_cb(self, msg):
        p = msg.pose.position
        with S.lock:
            S.pos       = np.array([p.x, p.y, p.z])
            S.pose_ok   = True
            S.pose_time = time.time()
            dist        = float(np.linalg.norm(S.pos - T0))

            # Auto-activate when drone enters HOME_RADIUS
            if not S.land_active and dist < HOME_RADIUS:
                S.land_active    = True
                S.auto_activated = True
                print(f"[return_land] AUTO-ACTIVATED — dist={dist:.2f}m")
                threading.Thread(target=switch_to_guided, daemon=True).start()
                # Tilt gimbal to search angle as drone approaches
                threading.Thread(
                    target=lambda: (
                        gimbal_set_pitch(GIMBAL_SEARCH),
                        setattr(S, 'gimbal_angle', GIMBAL_SEARCH)
                    ), daemon=True).start()

            # Phase 1 → 2 when close enough for tag search
            if S.phase == 1 and S.land_active and dist < (HOME_RADIUS * 0.4):
                S.phase = 2
                print("[return_land] Phase 1 → 2: AprilTag search active")
                # Point gimbal straight down for AprilTag detection
                threading.Thread(
                    target=lambda: (
                        gimbal_set_pitch(GIMBAL_NADIR),
                        setattr(S, 'gimbal_angle', GIMBAL_NADIR)
                    ), daemon=True).start()

    # ── Depth callback ────────────────────────────────────────
    def _depth_cb(self, msg):
        """
        Read ZED depth image (32-bit float, metres).
        Samples depth at the last known tag centre pixel.
        Only runs when tag is visible to avoid wasted work.
        """
        with S.lock:
            if not S.tag_visible:
                return
            cx = S.tag_cx
            cy = S.tag_cy

        # Bounds check
        if not (0 < cx < msg.width and 0 < cy < msg.height):
            return

        try:
            # ZED depth is 32FC1 — 4 bytes per pixel
            row_offset  = cy * msg.step
            col_offset  = cx * 4
            raw         = msg.data[row_offset + col_offset:
                                   row_offset + col_offset + 4]
            depth_m     = float(np.frombuffer(bytes(raw), dtype=np.float32)[0])

            # ZED returns nan/inf for invalid measurements
            if math.isfinite(depth_m) and 0.05 < depth_m < 20.0:
                with S.lock:
                    S.tag_dist_depth  = depth_m
                    S.tag_dist        = depth_m   # prefer real depth
                    S.depth_available = True
        except Exception:
            pass   # depth read failed — fall back to visual estimate

    # ── AprilTag callback ─────────────────────────────────────
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

                # Early Phase 1 → 2 transition if tag already visible
                # Mirrors landing_controller.py behaviour — don't wait for
                # HOME_RADIUS * 0.4 distance if the tag is already in frame
                if S.phase == 1 and S.land_active:
                    S.phase = 2
                    print("[return_land] Tag visible in Phase 1 — early jump to Phase 2")
                    threading.Thread(
                        target=lambda: (
                            gimbal_set_pitch(GIMBAL_NADIR),
                            setattr(S, 'gimbal_angle', GIMBAL_NADIR)
                        ), daemon=True).start()

                # Pixel offset from image centre
                px_x = det.centre.x - IMG_W / 2.0
                px_y = det.centre.y - IMG_H / 2.0

                # Save pixel centre for depth lookup
                S.tag_cx = int(det.centre.x)
                S.tag_cy = int(det.centre.y)

                # Tag pixel width → metric offset + visual distance estimate
                c    = det.corners
                w_px = math.hypot(c[1].x - c[0].x, c[1].y - c[0].y)

                if w_px > 0:
                    ppm         = w_px / TAG_SIZE
                    S.tag_x     = px_x / ppm
                    S.tag_y     = px_y / ppm
                    # Visual distance estimate (fallback if depth unavailable)
                    # fx ≈ IMG_W * 0.8 is a reasonable approximation for HD720
                    fx_approx           = IMG_W * 0.8
                    S.tag_dist_visual   = (TAG_SIZE * fx_approx) / w_px
                    # Only use visual if depth isn't fresh
                    if not S.depth_available:
                        S.tag_dist = S.tag_dist_visual

                    # Debug output — mirrors landing_controller.py control_loop prints
                    x_action = ("OK" if abs(S.tag_x) < DEADBAND
                                else f"move {'RIGHT' if S.tag_x > 0 else 'LEFT'} ({abs(S.tag_x):.3f}m)")
                    y_action = ("OK" if abs(S.tag_y) < DEADBAND
                                else f"move {'DOWN' if S.tag_y > 0 else 'UP'} ({abs(S.tag_y):.3f}m)")
                    z_err    = S.tag_dist - LAND_DIST
                    z_action = ("OK" if abs(z_err) < DEADBAND
                                else f"{'DESCEND' if z_err > 0 else 'ASCEND'} ({abs(z_err):.3f}m)")
                    print(f"[tag] X:{x_action}  Y:{y_action}  Z:{z_action}  "
                          f"dist:{S.tag_dist:.3f}m  w_px:{w_px:.1f}")
                else:
                    S.tag_x = S.tag_y = 0.0

                # Publish global tag position for logging
                tg_x = S.pos[0] + S.tag_x
                tg_z = S.pos[2] + S.tag_y
                threading.Thread(
                    target=self._pub_tag_pos,
                    args=(tg_x, tg_z), daemon=True).start()

                # Phase 2 → 3: aligned and close enough
                if (S.phase == 2 and
                        abs(S.tag_x) < DEADBAND and
                        abs(S.tag_y) < DEADBAND and
                        S.tag_dist   < LAND_DIST):
                    S.phase = 3
                    print("[return_land] Phase 2 → 3: LANDING")
                break

    def _pub_tag_pos(self, x, z):
        pt = PointStamped()
        pt.header.frame_id = "map"
        pt.header.stamp    = self.get_clock().now().to_msg()
        pt.point.x = x
        pt.point.y = 0.0
        pt.point.z = z
        self._tag_pos_pub.publish(pt)

    def publish_land_cmd(self):
        msg = Bool(); msg.data = True
        self._land_pub.publish(msg)
        self.get_logger().info('Land command published on /land_command')

ros_node: LandNode = None

def _ros_spin():
    global ros_node
    rclpy.init()
    ros_node = LandNode()
    rclpy.spin(ros_node)
    ros_node.destroy_node()
    rclpy.shutdown()

threading.Thread(target=_ros_spin, daemon=True).start()
time.sleep(1.0)

# ── MAVLink control thread — 10 Hz ───────────────────────────
def _mav_thread():
    land_sent = False
    while True:
        time.sleep(0.1)

        if mav:
            hb = mav.recv_match(type='HEARTBEAT', blocking=False)
            if hb:
                with S.lock:
                    S.flight_mode = mav.flightmode
                    S.hb_time     = time.time()

            bat = mav.recv_match(type='SYS_STATUS', blocking=False)
            if bat:
                with S.lock:
                    S.battery_pct = bat.battery_remaining

            with S.lock:
                stale_hb = (time.time() - S.hb_time) > 1.0
                mode     = S.flight_mode
                batt     = S.battery_pct

            if batt < 20:
                print("[return_land] LOW BATTERY — forcing land")
                send_land()
                continue

            if stale_hb or mode != "GUIDED":
                continue

        with S.lock:
            phase    = S.phase
            active   = S.land_active
            pose_ok  = S.pose_ok
            pose_age = time.time() - S.pose_time
            tag_vis  = S.tag_visible
            tx, ty   = S.tag_x, S.tag_y
            tdist    = S.tag_dist
            cur      = S.pos.copy()
            alt      = cur[1]

        if not active: continue

        if not pose_ok or pose_age > 0.5:
            send_velocity(0, 0, 0)
            continue

        # Phase 1 — navigate toward T0 using ZED odometry
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

        # Phase 2 — AprilTag alignment using real depth distance
        elif phase == 2:
            if tag_vis:
                z_err = tdist - LAND_DIST
                vz    = 0.0
                if abs(z_err) > DEADBAND:
                    vz = _clamp(TAG_GAIN * z_err, 0.3)
                    if alt < ALT_FLOOR and vz > 0:
                        vz = 0.0
                send_velocity(
                    _clamp(TAG_GAIN * ty, 0.5),
                    _clamp(TAG_GAIN * tx, 0.5),
                    vz)
            else:
                send_velocity(0, 0, 0)

        # Phase 3 — land
        elif phase == 3:
            if not land_sent:
                send_land()
                if ros_node: ros_node.publish_land_cmd()
                land_sent = True
                # Return gimbal to forward-facing after landing initiated
                threading.Thread(
                    target=lambda: (
                        time.sleep(2),
                        gimbal_set_pitch(GIMBAL_FORWARD),
                        setattr(S, 'gimbal_angle', GIMBAL_FORWARD)
                    ), daemon=True).start()

threading.Thread(target=_mav_thread, daemon=True).start()

# ── Pygame HUD ────────────────────────────────────────────────
pygame.init()
W, H   = 560, 480
screen = pygame.display.set_mode((W, H))
pygame.display.set_caption("Program 3 — Return & Land  |  L=force  C=clear  Q=quit")
FSM = pygame.font.SysFont("consolas", 13)
FMD = pygame.font.SysFont("consolas", 15, bold=True)
FLG = pygame.font.SysFont("consolas", 20, bold=True)
clk = pygame.time.Clock()

BG   = ( 15,  15,  22);  PANEL = ( 25,  25,  38);  GR   = ( 35,  35,  50)
C_OK = ( 30, 200, 130);  C_WR  = (220, 100,  60);  C_TX = (210, 210, 220)
C_DM = (100, 100, 120);  C_TG  = (220,  80, 220);  C_YO = (240, 160,  40)
C_TR = ( 60, 120, 180);  C_BT  = (255, 200,  40);  C_BL = ( 80, 140, 220)

MX, MY, MW, MH = 240, 40, 305, 390
MCX = MX + MW // 2
MCY = MY + MH // 2
SCALE = 12

def w2s(x, z):
    return int(MCX + x * SCALE), int(MCY - z * SCALE)

trail   = []
running = True

while running:
    with S.lock:
        phase       = S.phase
        active      = S.land_active
        auto_act    = S.auto_activated
        guided_ok   = S.guided_ok
        pose_ok     = S.pose_ok
        tag_vis     = S.tag_visible
        tx, ty      = S.tag_x, S.tag_y
        tdist       = S.tag_dist
        tdist_vis   = S.tag_dist_visual
        tdist_dep   = S.tag_dist_depth
        depth_avail = S.depth_available
        cur         = S.pos.copy()
        fmode       = S.flight_mode
        battery     = S.battery_pct
        gimbal_ang  = S.gimbal_angle
        gimbal_ok   = S.gimbal_ok

    dist = float(np.linalg.norm(cur - T0))

    for ev in pygame.event.get():
        if ev.type == pygame.QUIT: running = False
        if ev.type == pygame.KEYDOWN:
            if ev.key == pygame.K_q: running = False
            elif ev.key == pygame.K_c: trail.clear()
            elif ev.key == pygame.K_l:
                with S.lock:
                    if not S.land_active and S.pose_ok:
                        S.land_active    = True
                        S.auto_activated = False
                        print("[return_land] Manual activation")
                        threading.Thread(
                            target=switch_to_guided, daemon=True).start()
                    elif not S.pose_ok:
                        print("[return_land] No ZED pose yet")

    if pose_ok:
        trail.append((cur[0], cur[2]))
        if len(trail) > 600: trail.pop(0)

    # ── Draw ──────────────────────────────────────────────────
    screen.fill(BG)
    pygame.draw.rect(screen, PANEL, (0, 0, 235, H))

    # Map grid
    pygame.draw.rect(screen, GR, (MX, MY, MW, MH), 1)
    for g in range(-20, 21, 5):
        sx, _ = w2s(g, 0)
        if MX < sx < MX+MW:
            pygame.draw.line(screen, GR, (sx, MY), (sx, MY+MH))
        _, sy = w2s(0, g)
        if MY < sy < MY+MH:
            pygame.draw.line(screen, GR, (MX, sy), (MX+MW, sy))

    # Trail
    if len(trail) > 1:
        pts = [w2s(x, z) for x, z in trail]
        pts = [(x, y) for x, y in pts if MX < x < MX+MW and MY < y < MY+MH]
        if len(pts) > 1:
            pygame.draw.lines(screen, C_TR, False, pts, 2)

    # HOME_RADIUS circle
    r_px = int(HOME_RADIUS * SCALE)
    pygame.draw.circle(screen, GR, (MCX, MCY), r_px, 1)

    # Home marker
    hx, hy = w2s(T0[0], T0[2])
    if MX < hx < MX+MW and MY < hy < MY+MH:
        pygame.draw.circle(screen, C_OK, (hx, hy), 11)
        pygame.draw.circle(screen, (255, 255, 255), (hx, hy), 11, 2)
        screen.blit(FSM.render("HOME", True, (255,255,255)), (hx-16, hy-24))
        if tag_vis and active:
            pygame.draw.circle(screen, C_TG, (hx, hy), 22, 2)

    # Drone marker
    if pose_ok:
        dx, dy = w2s(cur[0], cur[2])
        if MX < dx < MX+MW and MY < dy < MY+MH:
            col = C_TG if active else C_YO
            pygame.draw.circle(screen, col, (dx, dy), 9)
            pygame.draw.circle(screen, (255,255,255), (dx, dy), 9, 2)
            screen.blit(FSM.render("UAV", True, (255,255,255)), (dx-10, dy-22))
    else:
        screen.blit(FMD.render("No ZED pose", True, C_WR), (MX+10, MCY-10))

    # Phase banner
    ph_txt = ["", "PH1: NAVIGATE", "PH2: APRILTAG", "PH3: LANDING"][phase]
    ph_col = [C_TX, C_TX, C_TG, C_TG][phase]
    screen.blit(FMD.render(ph_txt, True, ph_col), (MX+5, MY+MH+6))

    # ── Left HUD panel ────────────────────────────────────────
    def row(t, y, f=FSM, c=C_TX):
        screen.blit(f.render(t, True, c), (8, y))

    row("RETURN & LAND",    8,  FLG, C_TX)
    row("─"*26,            30,  FSM, GR)

    row("ZED POSE",         42, FSM, C_DM)
    pc = C_OK if pose_ok else C_WR
    row("LIVE ✓" if pose_ok else "NO SIGNAL", 57, FMD, pc)
    row(f"X {cur[0]:+.3f}", 74, FSM, C_TX)
    row(f"Y {cur[1]:+.3f}", 88, FSM, C_TX)
    row(f"Z {cur[2]:+.3f}",102, FSM, C_TX)

    row("ORIGIN",          118, FSM, C_DM)
    row(f"X {T0[0]:+.3f}", 133, FSM, C_OK)
    row(f"Z {T0[2]:+.3f}", 147, FSM, C_OK)

    row("DIST TO HOME",    163, FSM, C_DM)
    dc = C_OK if dist < 2 else C_WR if dist < 5 else (180,60,60)
    row(f"{dist:.2f} m",   178, FMD, dc)

    row("─"*26,            196, FSM, GR)
    row("APRILTAG",        206, FSM, C_DM)
    if tag_vis:
        row(f"X {tx:+.3f}m", 221, FSM, C_TG)
        row(f"Y {ty:+.3f}m", 235, FSM, C_TG)
        # Show which distance source is active
        dep_lbl = "ZED depth" if depth_avail else "visual est"
        row(f"D {tdist:.3f}m ({dep_lbl})", 249, FSM, C_BL if depth_avail else C_TG)
        if depth_avail:
            row(f"  vis:{tdist_vis:.3f} dep:{tdist_dep:.3f}", 263, FSM, C_DM)
    else:
        row("not visible",   221, FSM, C_DM)

    row("─"*26,            283, FSM, GR)
    row("MAVLink",         293, FSM, C_DM)
    mc = C_OK if (mav and fmode == "GUIDED") else C_WR
    row(fmode if mav else "DISCONNECTED", 308, FSM, mc)

    row("GUIDED CMD",      324, FSM, C_DM)
    row("SENT ✓" if guided_ok else "Pending...",
                           339, FMD, C_OK if guided_ok else C_DM)

    row("BATTERY",         355, FSM, C_DM)
    bc = C_OK if battery > 50 else C_BT if battery > 20 else (180,60,60)
    row(f"{battery}%",     370, FMD, bc)

    row("GIMBAL",          386, FSM, C_DM)
    g_lbl = f"{gimbal_ang:.0f} deg"
    if   gimbal_ang == GIMBAL_NADIR:   g_lbl += " (NADIR)"
    elif gimbal_ang == GIMBAL_SEARCH:  g_lbl += " (SEARCH)"
    elif gimbal_ang == GIMBAL_FORWARD: g_lbl += " (FORWARD)"
    gc = C_OK if gimbal_ok else C_DM
    row(g_lbl,             401, FMD, gc)

    row("STATUS",          417, FSM, C_DM)
    lbl = ("AUTO ✓" if auto_act else "MANUAL ✓") if active else "Standby"
    row(lbl,               432, FMD, C_TG if active else C_DM)

    row("─"*26,            420, FSM, GR)
    row("[L] Force  [C] Clear  [Q] Quit", 432, FSM, C_DM)

    pygame.display.flip()
    clk.tick(30)

pygame.quit()
sys.exit()