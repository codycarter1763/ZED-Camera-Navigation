#!/usr/bin/env python3
# ─────────────────────────────────────────────────────────────
# save_position.py  —  PROGRAM 2
# Run after the scan mission loiters at its final waypoint.
# Saves current ZED pose to scan_end_pos.json.
#
# ZED topic used:
#   /zed/zed_node/pose  (geometry_msgs/PoseStamped)
#
# Usage:   python3 "save_position.py"
# Controls: SPACE = save    Q = quit
# Output:   ~/ZED Navigation/scan_end_pos.json
# ─────────────────────────────────────────────────────────────

import json, os, sys, threading, time
import pygame
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

SCRIPT_DIR    = os.path.dirname(os.path.abspath(__file__))
SCAN_END_FILE = os.path.join(SCRIPT_DIR, "scan_end_pos.json")
ZED_POSE_TOPIC = '/zed/zed_node/pose'

class State:
    def __init__(self):
        self.lock    = threading.Lock()
        self.pos     = [0.0, 0.0, 0.0]
        self.pose_ok = False
        self.saved   = False
        self.status  = "Waiting for ZED pose..."
        self.cnt     = 0
S = State()

class SavePosNode(Node):
    def __init__(self):
        super().__init__('save_position')
        self.create_subscription(
            PoseStamped, ZED_POSE_TOPIC, self._cb, 10)

    def _cb(self, msg):
        p = msg.pose.position
        with S.lock:
            S.pos     = [p.x, p.y, p.z]
            S.pose_ok = True
            S.cnt    += 1

def _ros_spin():
    rclpy.init()
    node = SavePosNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

threading.Thread(target=_ros_spin, daemon=True).start()
time.sleep(1.0)

def save_position():
    with S.lock:
        if not S.pose_ok:
            S.status = "ERROR: No ZED pose!"
            return
        data = {"x": S.pos[0], "y": S.pos[1], "z": S.pos[2],
                "timestamp": time.time()}
        with open(SCAN_END_FILE, "w") as f:
            json.dump(data, f, indent=2)
        S.saved  = True
        S.status = (f"SAVED  X:{S.pos[0]:.3f} "
                    f"Y:{S.pos[1]:.3f}  Z:{S.pos[2]:.3f}")
        print(f"[save_position] Saved → {SCAN_END_FILE}")

# ── Pygame HUD ────────────────────────────────────────────────
pygame.init()
W, H   = 460, 300
screen = pygame.display.set_mode((W, H))
pygame.display.set_caption("Program 2 — Save Position  |  SPACE=save  Q=quit")
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
            elif ev.key == pygame.K_SPACE: save_position()

    with S.lock:
        pos, ok, saved = S.pos.copy(), S.pose_ok, S.saved
        status, cnt    = S.status, S.cnt

    screen.fill(BG)
    def row(t, y, f=FSM, c=C_TX): screen.blit(f.render(t, True, c), (15, y))

    row("SAVE POSITION — PROGRAM 2",      12, FLG, C_TX)
    pygame.draw.line(screen, GR, (15, 40), (W-15, 40), 1)
    row("ZED POSE",                       52, FSM, C_DM)
    row("LIVE ✓" if ok else "NO SIGNAL",  67, FMD, C_OK if ok else C_WR)
    row(f"Msgs received: {cnt}",          84, FSM, C_DM)
    pygame.draw.line(screen, GR, (15, 104), (W-15, 104), 1)
    row("CURRENT POSITION (m)",          114, FSM, C_DM)
    row(f"X {pos[0]:+.4f}  Y {pos[1]:+.4f}  Z {pos[2]:+.4f}",
                                         129, FSM, C_OK if ok else C_WR)
    pygame.draw.line(screen, GR, (15, 150), (W-15, 150), 1)
    row("SCAN END POSITION",             160, FSM, C_DM)
    row("SAVED ✓" if saved else "Not saved yet",
                                         175, FMD, C_OK if saved else C_DM)
    pygame.draw.line(screen, GR, (15, 200), (W-15, 200), 1)
    row(status,                          212, FSM, C_OK if saved else C_WR)
    pygame.draw.line(screen, GR, (15, 235), (W-15, 235), 1)
    row("Run after scan mission reaches loiter WP.", 247, FSM, C_DM)
    row("[SPACE] Save position    [Q] Quit",         263, FSM, C_DM)
    row("Output: " + SCAN_END_FILE,                  279, FSM, C_DM)

    pygame.display.flip()
    clk.tick(30)

pygame.quit()
sys.exit()
