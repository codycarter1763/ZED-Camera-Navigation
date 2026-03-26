# desktop_return_sim.py
# Run this AFTER you have:
#   1. Saved origin_T0.json with save_origin.py
#   2. Finished mapping and saved rover_home.area from save_origin.py
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
SCRIPT_DIR  = os.path.dirname(os.path.abspath(__file__))
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
with open(ORIGIN_FILE) as f:
    origin = json.load(f)
T0 = np.array([origin["x"], origin["y"], origin["z"]])

print("Origin loaded:")
print(f"  X: {T0[0]:.3f}  Y: {T0[1]:.3f}  Z: {T0[2]:.3f}")

# ── Init ZED ─────────────────────────────────────────────────
zed = sl.Camera()
init = sl.InitParameters()
init.coordinate_units  = sl.UNIT.METER
init.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP

status = zed.open(init)
if status != sl.ERROR_CODE.SUCCESS:
    print(f"ERROR: Could not open ZED camera: {status}")
    input("Press Enter to close...")
    sys.exit()

# Load area memory so relocalization can happen
tracking_params = sl.PositionalTrackingParameters()
tracking_params.area_file_path     = AREA_FILE
tracking_params.enable_area_memory = True
zed.enable_positional_tracking(tracking_params)

print("ZED camera opened.")
print("Loading area memory... this may take a few seconds.")
print("Slowly pan camera around the room to relocalize...")
print("")

# ── Pygame setup ──────────────────────────────────────────────
pygame.init()
W, H = 900, 650
screen = pygame.display.set_mode((W, H))
pygame.display.set_caption("ZED Return Simulator  |  R = simulate return  |  Q = quit")
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
TEXT_COL    = (210, 210, 220)
DIM_COL     = (100, 100, 120)
WARN_COL    = (220, 100,  60)
PANEL_COL   = ( 25,  25,  38)

# ── Map view settings ─────────────────────────────────────────
MAP_X  = 250
MAP_Y  = 50
MAP_W  = 620
MAP_H  = 560
MAP_CX = MAP_X + MAP_W // 2
MAP_CY = MAP_Y + MAP_H // 2
SCALE  = 12    # pixels per meter — scaled for 40 meter range

def world_to_screen(x, z):
    sx = int(MAP_CX + x * SCALE)
    sy = int(MAP_CY - z * SCALE)
    return sx, sy

# ── State ─────────────────────────────────────────────────────
pose           = sl.Pose()
trail          = []
current_pos    = np.array([0.0, 0.0, 0.0])
confidence     = 0
simulating     = False
return_path    = []
return_idx     = 0
sim_done       = False
relocated      = False
frame_count    = 0
spatial_status = "UNKNOWN"

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
                if confidence >= 70 and relocated:
                    return_path = compute_return_path(current_pos.copy(), T0)
                    return_idx  = 0
                    simulating  = True
                    sim_done    = False
                    print("Simulating return path...")
                else:
                    if not relocated:
                        print(f"  Not relocalized yet — pan camera around room")
                        print(f"  Spatial memory: {spatial_status}")
                    else:
                        print(f"  Confidence too low ({confidence}%)")

            if event.key == pygame.K_c:
                trail.clear()
                simulating = False
                sim_done   = False

    # ── Grab ZED frame ────────────────────────────────────────
    if zed.grab() == sl.ERROR_CODE.SUCCESS:
        zed.get_position(pose)
        confidence  = pose.pose_confidence
        t           = pose.get_translation().get()
        current_pos = np.array([float(t[0]), float(t[1]), float(t[2])])
        frame_count += 1

        # ── Check spatial memory status ───────────────────────
        tracking_status = zed.get_positional_tracking_status()
        spatial_memory  = tracking_status.spatial_memory_status
        spatial_status  = str(spatial_memory)

        # Print status every 90 frames (~3 seconds) while waiting
        if not relocated and frame_count % 90 == 0:
            print(f"  Confidence: {confidence:3d}%  |  "
                  f"Spatial memory: {spatial_status}")

        # Only trust position when spatial memory has matched the saved map
        if confidence >= 70 and not relocated:
            if "LOOP_CLOSED" in spatial_status:
                relocated = True
                print("")
                print("  ============================================")
                print("  Spatial memory LOOP CLOSED — map recognized!")
                print("  ============================================")
                print(f"  Current pos : X={current_pos[0]:.3f} "
                      f"Y={current_pos[1]:.3f} "
                      f"Z={current_pos[2]:.3f}")
                print(f"  Home T0     : X={T0[0]:.3f} "
                      f"Y={T0[1]:.3f} "
                      f"Z={T0[2]:.3f}")
                dist = float(np.linalg.norm(current_pos - T0))
                print(f"  Distance    : {dist:.3f} m")
                print("")

        # Record trail only after proper relocalization
        if relocated:
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
    for gx in range(-40, 41):
        for gz in range(-40, 41):
            sx, sy = world_to_screen(gx, gz)
            if MAP_X < sx < MAP_X + MAP_W and MAP_Y < sy < MAP_Y + MAP_H:
                pygame.draw.circle(screen, GRID, (sx, sy), 1)

    # Meter labels every 5 meters
    for m in range(-40, 41, 5):
        sx, sy = world_to_screen(m, 0)
        if MAP_X < sx < MAP_X + MAP_W:
            lbl = font_sm.render(f"{m}m", True, GRID)
            screen.blit(lbl, (sx - 10, MAP_CY + 6))

    # ── Trail ─────────────────────────────────────────────────
    if len(trail) > 1:
        pts = [world_to_screen(x, z) for x, z in trail]
        pts = [(x, y) for x, y in pts
               if MAP_X < x < MAP_X + MAP_W and MAP_Y < y < MAP_Y + MAP_H]
        if len(pts) > 1:
            pygame.draw.lines(screen, TRAIL, False, pts, 2)

    # ── Return path ───────────────────────────────────────────
    if simulating and len(return_path) > 1:
        rpts = [world_to_screen(p[0], p[2]) for p in return_path[:return_idx+1]]
        rpts = [(x, y) for x, y in rpts
                if MAP_X < x < MAP_X + MAP_W and MAP_Y < y < MAP_Y + MAP_H]
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
    if MAP_X < hx < MAP_X + MAP_W and MAP_Y < hy < MAP_Y + MAP_H:
        pygame.draw.circle(screen, HOME_COL, (hx, hy), 14)
        pygame.draw.circle(screen, (255, 255, 255), (hx, hy), 14, 2)
        lbl = font_sm.render("HOME", True, (255, 255, 255))
        screen.blit(lbl, (hx - 18, hy - 28))

    # ── Current position marker ───────────────────────────────
    if relocated:
        cx, cy = world_to_screen(current_pos[0], current_pos[2])
        if MAP_X < cx < MAP_X + MAP_W and MAP_Y < cy < MAP_Y + MAP_H:
            pygame.draw.circle(screen, CURRENT_COL, (cx, cy), 11)
            pygame.draw.circle(screen, (255, 255, 255), (cx, cy), 11, 2)
            lbl = font_sm.render("YOU", True, (255, 255, 255))
            screen.blit(lbl, (cx - 10, cy - 26))
    else:
        # Show searching message with spatial memory status
        line1 = font_md.render(
            f"Searching...  confidence: {confidence}%",
            True, WARN_COL)
        line2 = font_sm.render(
            f"Spatial memory: {spatial_status}",
            True, TEXT_COL)
        line3 = font_sm.render(
            "Slowly pan camera around the room to relocalize",
            True, DIM_COL)
        screen.blit(line1, (MAP_X + 20, MAP_CY - 40))
        screen.blit(line2, (MAP_X + 20, MAP_CY))
        screen.blit(line3, (MAP_X + 20, MAP_CY + 30))

    # ── Left panel HUD ────────────────────────────────────────
    dist = float(np.linalg.norm(current_pos - T0))

    conf_color = (HOME_COL if confidence >= 70
                  else (WARN_COL if confidence >= 40
                  else (180, 60, 60)))
    conf_label = ("LOCALIZED ✓" if confidence >= 70
                  else ("SEARCHING..." if confidence < 40
                  else "PARTIAL"))

    hud = [
        ("ZED RETURN SIM",       font_lg, TEXT_COL,   20),
        ("",                     font_sm, DIM_COL,    52),
        ("CONFIDENCE",           font_sm, DIM_COL,    70),
        (f"{confidence}%  {conf_label}",
                                 font_md, conf_color,  88),
        ("",                     font_sm, DIM_COL,   110),
        ("POSITION (m)",         font_sm, DIM_COL,   120),
        (f"X: {current_pos[0]:+.3f}", font_sm, TEXT_COL, 138),
        (f"Y: {current_pos[1]:+.3f}", font_sm, TEXT_COL, 154),
        (f"Z: {current_pos[2]:+.3f}", font_sm, TEXT_COL, 170),
        ("",                     font_sm, DIM_COL,   190),
        ("HOME (m)",             font_sm, DIM_COL,   200),
        (f"X: {T0[0]:+.3f}",    font_sm, HOME_COL,  218),
        (f"Y: {T0[1]:+.3f}",    font_sm, HOME_COL,  234),
        (f"Z: {T0[2]:+.3f}",    font_sm, HOME_COL,  250),
        ("",                     font_sm, DIM_COL,   270),
        ("DIST TO HOME",         font_sm, DIM_COL,   280),
        (f"{dist:.2f} m",        font_lg, TEXT_COL,  298),
        ("",                     font_sm, DIM_COL,   325),
        ("SPATIAL MEMORY",       font_sm, DIM_COL,   340),
        (spatial_status.split(".")[-1],
                                 font_sm,
                                 HOME_COL if "LOOP" in spatial_status
                                 else WARN_COL,
                                 358),
        ("",                     font_sm, DIM_COL,   380),
        ("RELOCATED",            font_sm, DIM_COL,   395),
        ("YES ✓" if relocated else "NO - pan camera",
                                 font_md,
                                 HOME_COL if relocated else WARN_COL,
                                 413),
        ("",                     font_sm, DIM_COL,   438),
        ("CONTROLS",             font_sm, DIM_COL,   455),
        ("[R] Simulate return",  font_sm, TEXT_COL,  473),
        ("[C] Clear trail",      font_sm, TEXT_COL,  491),
        ("[Q] Quit",             font_sm, TEXT_COL,  509),
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
        screen.blit(msg, (MAP_X + MAP_W // 2 - 100, MAP_Y + MAP_H - 40))

    # Warning at bottom of map while not relocated
    if not relocated:
        warn = font_sm.render(
            "Waiting for LOOP_CLOSED before showing position",
            True, WARN_COL)
        screen.blit(warn, (MAP_X + 10, MAP_Y + MAP_H - 30))

    pygame.display.flip()
    clock.tick(30)

zed.close()
pygame.quit()
sys.exit()
