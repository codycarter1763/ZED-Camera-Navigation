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
# Usage:   python3 "set_origin.py"
# Controls: SPACE = save origin    Q = quit
# Output:   ~/ZED Navigation/origin_T0.json
# ─────────────────────────────────────────────────────────────

import json, os, sys, threading, time
import pygame
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

SCRIPT_DIR  = os.path.dirname(os.path.abspath(__file__))
ORIGIN_FILE = os.path.join(SCRIPT_DIR, "origin_T0.json")

# ── ZED topic — matches zed_wrapper default namespace ─────────
ZED_POSE_TOPIC = '/zed/zed_node/pose'

# ── Shared state ──────────────────────────────────────────────
class State:
    def __init__(self):
        self.lock        = threading.Lock()
        self.pos         = [0.0, 0.0, 0.0]
        self.pose_ok     = False
        self.origin_set  = False
        self.status_msg  = "Waiting for ZED pose..."
        self.pose_count  = 0
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

def save_origin():
    with S.lock:
        if not S.pose_ok:
            S.status_msg = "ERROR: No ZED pose — is zed_apriltag.launch.py running?"
            return
        data = {"x": S.pos[0], "y": S.pos[1], "z": S.pos[2]}
        with open(ORIGIN_FILE, "w") as f:
            json.dump(data, f, indent=2)
        S.origin_set = True
        S.status_msg = (f"SAVED  X:{S.pos[0]:.3f} "
                        f"Y:{S.pos[1]:.3f}  Z:{S.pos[2]:.3f}")
        print(f"[set_origin] Origin saved → {ORIGIN_FILE}")
        print(f"[set_origin] Values: {data}")

# ── Pygame HUD ────────────────────────────────────────────────
pygame.init()
W, H   = 460, 320
screen = pygame.display.set_mode((W, H))
pygame.display.set_caption("Program 1 — Set Origin  |  SPACE=save  Q=quit")
FSM = pygame.font.SysFont("consolas", 13)
FMD = pygame.font.SysFont("consolas", 15, bold=True)
FLG = pygame.font.SysFont("consolas", 20, bold=True)
clk = pygame.time.Clock()
BG   = ( 15,  15,  22);  GR   = ( 35,  35,  50)
C_OK = ( 30, 200, 130);  C_WR = (220, 100,  60)
C_TX = (210, 210, 220);  C_DM = (100, 100, 120)

running = True
while running:
    for ev in pygame.event.get():
        if ev.type == pygame.QUIT: running = False
        if ev.type == pygame.KEYDOWN:
            if ev.key == pygame.K_q:       running = False
            elif ev.key == pygame.K_SPACE: save_origin()

    with S.lock:
        pos, ok, orig = S.pos.copy(), S.pose_ok, S.origin_set
        status, cnt   = S.status_msg, S.pose_count

    screen.fill(BG)
    def row(t, y, f=FSM, c=C_TX): screen.blit(f.render(t, True, c), (15, y))

    row("SET ORIGIN — PROGRAM 1",         12, FLG, C_TX)
    pygame.draw.line(screen, GR, (15, 40), (W-15, 40), 1)

    row("ZED POSE TOPIC",                 52, FSM, C_DM)
    row(ZED_POSE_TOPIC,                   67, FSM, C_DM)
    row("LIVE ✓" if ok else "NO SIGNAL",  82, FMD, C_OK if ok else C_WR)
    row(f"Msgs received: {cnt}",          98, FSM, C_DM)

    pygame.draw.line(screen, GR, (15, 116), (W-15, 116), 1)
    row("CURRENT POSITION (m)",          126, FSM, C_DM)
    row(f"X {pos[0]:+.4f}  Y {pos[1]:+.4f}  Z {pos[2]:+.4f}",
                                         141, FSM, C_OK if ok else C_WR)

    pygame.draw.line(screen, GR, (15, 162), (W-15, 162), 1)
    row("ORIGIN FILE",                   172, FSM, C_DM)
    row("SAVED ✓" if orig else "Not saved yet",
                                         187, FMD, C_OK if orig else C_DM)
    row(ORIGIN_FILE,                     204, FSM, C_DM)

    pygame.draw.line(screen, GR, (15, 224), (W-15, 224), 1)
    row(status,                          236, FSM, C_OK if orig else C_WR)

    pygame.draw.line(screen, GR, (15, 258), (W-15, 258), 1)
    row("Hover drone over landing pad,", 270, FSM, C_DM)
    row("then SPACE to lock in origin.", 286, FSM, C_DM)
    row("[SPACE] Save    [Q] Quit",      302, FSM, C_DM)

    pygame.display.flip()
    clk.tick(30)

pygame.quit()
sys.exit()
