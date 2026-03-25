# desktop_return_sim.py
# Run this AFTER you have:
#   1. Saved origin_T0.json with save_origin.py
#   2. Finished mapping and saved rover_home.area from ZEDfu
#
# Hold the camera at the FAR END of the room
# Watch your position on screen, then press R to simulate return

import pyzed.sl as sl
import json
import numpy as np
import pygame
import sys
import os

# ── Check files exist before starting ────────────────────────
if not os.path.exists("origin_T0.json"):
    print("ERROR: origin_T0.json not found.")
    print("Run save_origin.py first.")
    input("Press Enter to close...")
    sys.exit()

# Get the folder where this script lives
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

AREA_FILE   = os.path.join(SCRIPT_DIR, "rover_home.area")
ORIGIN_FILE = os.path.join(SCRIPT_DIR, "origin_T0.json")

if not os.path.exists(ORIGIN_FILE):
    print(f"ERROR: origin_T0.json not found at {ORIGIN_FILE}")
    input("Press Enter to close...")
    sys.exit()

if not os.path.exists(AREA_FILE):
    print(f"ERROR: rover_home.area not found at {AREA_FILE}")
    input("Press Enter to close...")
    sys.exit()

# ── Load origin ───────────────────────────────────────────────
with open("origin_T0.json") as f:
    origin = json.load(f)
T0 = np.array([origin["x"], origin["y"], origin["z"]])

print("Origin loaded:")
print(f"  X: {T0[0]:.3f}  Y: {T0[1]:.3f}  Z: {T0[2]:.3f}")

# ── Init ZED ─────────────────────────────────────────────────
zed = sl.Camera()
init = sl.InitParameters()
init.coordinate_units = sl.UNIT.METER
init.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP

status = zed.open(init)
if status != sl.ERROR_CODE.SUCCESS:
    print(f"ERROR: Could not open ZED camera: {status}")
    input("Press Enter to close...")
    sys.exit()

# Load area memory so relocalization can happen
tracking_params = sl.PositionalTrackingParameters()
tracking_params.area_file_path = "rover_home.area"
zed.enable_positional_tracking(tracking_params)

print("ZED camera opened.")
print("Loading area memory... this may take a few seconds.")

# ── Pygame setup ──────────────────────────────────────────────
pygame.init()
W, H = 900, 650
screen = pygame.display.set_mode((W, H))
pygame.display.set_caption("ZED Return Simulator  |  Press R to simulate return  |  Press Q to quit")
font_sm = pygame.font.SysFont("consolas", 14)
font_md = pygame.font.SysFont("consolas", 17, bold=True)
font_lg = pygame.font.SysFont("consolas", 22, bold=True)
clock = pygame.time.Clock()

# ── Colors ────────────────────────────────────────────────────
BG          = ( 15,  15,  22)
GRID        = ( 35,  35,  50)
TRAIL       = ( 60, 120, 180)
HOME_COL    = ( 30, 200, 130)
CURRENT_COL = (240, 160,  40)
RETURN_COL  = ( 50, 210, 120)
TEXT_COL    = (210, 210, 220)
DIM_COL     = (100, 100, 120)
WARN_COL    = (220, 100,  60)
PANEL_COL   = ( 25,  25,  38)

# ── Map view settings ─────────────────────────────────────────
# Top-down view — X axis = left/right, Z axis = forward/back
MAP_X      = 250   # left edge of map panel
MAP_Y      = 50    # top edge of map panel
MAP_W      = 620
MAP_H      = 560
MAP_CX     = MAP_X + MAP_W // 2   # center of map panel
MAP_CY     = MAP_Y + MAP_H // 2
SCALE      = 25   # pixels per meter

def world_to_screen(x, z):
    sx = int(MAP_CX + x * SCALE)
    sy = int(MAP_CY - z * SCALE)  # flip Z so forward = up
    return sx, sy

# ── State ─────────────────────────────────────────────────────
pose = sl.Pose()
trail = []
current_pos = np.array([0.0, 0.0, 0.0])
confidence  = 0
simulating  = False
return_path = []
return_idx  = 0
sim_done    = False

def compute_return_path(start, end, steps=80):
    return [start + (end - start) * (i / steps) for i in range(steps + 1)]

# ── Main loop ─────────────────────────────────────────────────
running = True
while running:

    # ── Events ────────────────────────────────────────────────
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_q:
                running = False

            if event.key == pygame.K_r:
                if confidence >= 70:
                    return_path = compute_return_path(current_pos.copy(), T0)
                    return_idx  = 0
                    simulating  = True
                    sim_done    = False
                    print("Simulating return path...")
                else:
                    print(f"Confidence too low ({confidence}%) — wait for relocalization")

            if event.key == pygame.K_c:
                # C to clear trail
                trail.clear()
                simulating = False
                sim_done   = False

    # ── Grab ZED frame ────────────────────────────────────────
    if zed.grab() == sl.ERROR_CODE.SUCCESS:
        zed.get_position(pose)
        confidence = pose.pose_confidence
        t = pose.get_translation().get()
        current_pos = np.array([float(t[0]), float(t[1]), float(t[2])])
        trail.append((current_pos[0], current_pos[2]))
        if len(trail) > 800:
            trail.pop(0)

    # ── Advance simulation ────────────────────────────────────
    if simulating and not sim_done:
        return_idx += 2
        if return_idx >= len(return_path):
            return_idx = len(return_path) - 1
            sim_done   = True

    # ── Draw background ───────────────────────────────────────
    screen.fill(BG)

    # Left panel background
    pygame.draw.rect(screen, PANEL_COL, (0, 0, 245, H))

    # Map panel border
    pygame.draw.rect(screen, GRID, (MAP_X, MAP_Y, MAP_W, MAP_H), 1)
    
    # ── Grid dots ─────────────────────────────────────────────
    for gx in range(-20, 21):
        for gz in range(-20, 20):
            sx, sy = world_to_screen(gx, gz)
            if MAP_X < sx < MAP_X + MAP_W and MAP_Y < sy < MAP_Y + MAP_H:
                pygame.draw.circle(screen, GRID, (sx, sy), 2)

    # Axis labels (meter marks)
    for m in range(-20, 21, 2):
        sx, sy = world_to_screen(m, 0)
        if MAP_X < sx < MAP_X + MAP_W:
            lbl = font_sm.render(f"{m}m", True, GRID)
            screen.blit(lbl, (sx - 10, MAP_CY + 6))

    # ── Trail ─────────────────────────────────────────────────
    if len(trail) > 1:
        pts = [world_to_screen(x, z) for x, z in trail]
        # Only draw points inside the map panel
        pts = [(x, y) for x, y in pts if MAP_X < x < MAP_X+MAP_W and MAP_Y < y < MAP_Y+MAP_H]
        if len(pts) > 1:
            pygame.draw.lines(screen, TRAIL, False, pts, 2)

    # ── Return path ───────────────────────────────────────────
    if simulating and len(return_path) > 1:
        rpts = [world_to_screen(p[0], p[2]) for p in return_path[:return_idx+1]]
        rpts = [(x, y) for x, y in rpts if MAP_X < x < MAP_X+MAP_W and MAP_Y < y < MAP_Y+MAP_H]
        if len(rpts) > 1:
            pygame.draw.lines(screen, RETURN_COL, False, rpts, 3)

        # Animated dot along return path
        if not sim_done:
            ap = return_path[return_idx]
            ax, ay = world_to_screen(ap[0], ap[2])
            pygame.draw.circle(screen, RETURN_COL, (ax, ay), 9)
            pygame.draw.circle(screen, (255, 255, 255), (ax, ay), 9, 2)

    # ── Home marker (T0) ──────────────────────────────────────
    hx, hy = world_to_screen(T0[0], T0[2])
    pygame.draw.circle(screen, HOME_COL, (hx, hy), 14)
    pygame.draw.circle(screen, (255, 255, 255), (hx, hy), 14, 2)
    lbl = font_sm.render("HOME", True, (255, 255, 255))
    screen.blit(lbl, (hx - 18, hy - 28))

    # ── Current position marker ───────────────────────────────
    cx, cy = world_to_screen(current_pos[0], current_pos[2])
    if MAP_X < cx < MAP_X+MAP_W and MAP_Y < cy < MAP_Y+MAP_H:
        pygame.draw.circle(screen, CURRENT_COL, (cx, cy), 11)
        pygame.draw.circle(screen, (255, 255, 255), (cx, cy), 11, 2)
        lbl = font_sm.render("YOU", True, (255, 255, 255))
        screen.blit(lbl, (cx - 10, cy - 26))

    # ── Left panel — HUD ──────────────────────────────────────
    dist = float(np.linalg.norm(current_pos - T0))

    conf_color = HOME_COL if confidence >= 70 else (WARN_COL if confidence >= 40 else (180, 60, 60))
    conf_label = "LOCALIZED ✓" if confidence >= 70 else ("SEARCHING..." if confidence < 40 else "PARTIAL")

    hud = [
        ("ZED RETURN SIM",   font_lg, TEXT_COL,  20),
        ("",                 font_sm, DIM_COL,   52),
        ("CONFIDENCE",       font_sm, DIM_COL,   70),
        (f"{confidence}%  {conf_label}", font_md, conf_color, 88),
        ("",                 font_sm, DIM_COL,  110),
        ("POSITION (m)",     font_sm, DIM_COL,  120),
        (f"X: {current_pos[0]:+.3f}", font_sm, TEXT_COL, 138),
        (f"Y: {current_pos[1]:+.3f}", font_sm, TEXT_COL, 154),
        (f"Z: {current_pos[2]:+.3f}", font_sm, TEXT_COL, 170),
        ("",                 font_sm, DIM_COL,  190),
        ("HOME (m)",         font_sm, DIM_COL,  200),
        (f"X: {T0[0]:+.3f}", font_sm, HOME_COL, 218),
        (f"Y: {T0[1]:+.3f}", font_sm, HOME_COL, 234),
        (f"Z: {T0[2]:+.3f}", font_sm, HOME_COL, 250),
        ("",                 font_sm, DIM_COL,  270),
        ("DIST TO HOME",     font_sm, DIM_COL,  280),
        (f"{dist:.2f} m",    font_lg, TEXT_COL, 298),
        ("",                 font_sm, DIM_COL,  330),
        ("CONTROLS",         font_sm, DIM_COL,  345),
        ("[R] Simulate return", font_sm, TEXT_COL, 363),
        ("[C] Clear trail",    font_sm, TEXT_COL, 381),
        ("[Q] Quit",           font_sm, TEXT_COL, 399),
    ]

    for text, fnt, col, y in hud:
        surf = fnt.render(text, True, col)
        screen.blit(surf, (15, y))

    # Confidence bar
    bar_y = 105
    pygame.draw.rect(screen, GRID, (15, bar_y, 215, 10), border_radius=5)
    bar_w = int(215 * min(confidence, 100) / 100)
    pygame.draw.rect(screen, conf_color, (15, bar_y, bar_w, 10), border_radius=5)

    # Sim done message
    if sim_done:
        msg = font_md.render("ARRIVED AT HOME!", True, HOME_COL)
        screen.blit(msg, (MAP_X + MAP_W//2 - 100, MAP_Y + MAP_H - 40))

    # Low confidence warning when R pressed
    if not simulating and confidence < 70:
        warn = font_sm.render("Wait for confidence > 70% before simulating", True, WARN_COL)
        screen.blit(warn, (MAP_X + 10, MAP_Y + MAP_H - 30))

    pygame.display.flip()
    clock.tick(30)

zed.close()
pygame.quit()
sys.exit()