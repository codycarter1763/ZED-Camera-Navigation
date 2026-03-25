# save_origin.py
# Does two things:
#   1. Records the center of the room as home (origin_T0.json)
#   2. Walks the room and saves area memory (rover_home.area)
#
# HOW TO USE:
#   Step 1 - Stand at center of room, press Enter to save home position
#   Step 2 - Walk slowly around the entire room covering all walls
#   Step 3 - Press Enter when done walking to save the area file

import pyzed.sl as sl
import json
import os
import sys
import msvcrt

# ── File paths ────────────────────────────────────────────────
SCRIPT_DIR  = os.path.dirname(os.path.abspath(__file__))
AREA_FILE   = os.path.join(SCRIPT_DIR, "rover_home.area")
ORIGIN_FILE = os.path.join(SCRIPT_DIR, "origin_T0.json")

# ── Header ────────────────────────────────────────────────────
print("")
print("=================================")
print("  ZED Room Mapper + Origin Saver")
print("=================================")
print("")

# ── Open camera ───────────────────────────────────────────────
zed = sl.Camera()
init = sl.InitParameters()
init.coordinate_units    = sl.UNIT.METER
init.coordinate_system   = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP
init.depth_mode          = sl.DEPTH_MODE.NEURAL

print("Opening ZED camera...")
status = zed.open(init)

if status != sl.ERROR_CODE.SUCCESS:
    print("")
    print(f"  ERROR: Could not open camera")
    print(f"  Code: {status}")
    print("")
    print("  Check:")
    print("  - ZED2i is plugged into a BLUE USB 3.0 port")
    print("  - ZEDfu is closed")
    print("  - Try unplugging and replugging the cable")
    print("")
    input("Press Enter to close...")
    sys.exit(1)

print("  Camera opened OK!")
print("")

# ── Enable positional tracking with area memory ───────────────
print("Enabling positional tracking...")
tracking_params = sl.PositionalTrackingParameters()
tracking_params.enable_area_memory = True

track_status = zed.enable_positional_tracking(tracking_params)

if track_status != sl.ERROR_CODE.SUCCESS:
    print(f"  ERROR: Could not enable tracking: {track_status}")
    input("Press Enter to close...")
    sys.exit(1)

print("  Tracking enabled OK!")
print("")

# ── Let tracking stabilize ────────────────────────────────────
print("  Stabilizing tracking...")
print("  Point camera at the room and hold steady...")
print("")

pose = sl.Pose()

for i in range(60):
    zed.grab()
    if i % 15 == 0:
        print(f"  {i}/60 frames stabilized...")

print("  Stabilization done!")
print("")

# ── Step 1: Save origin at center of room ─────────────────────
print("=================================")
print("  STEP 1 OF 2 — SAVE HOME")
print("=================================")
print("")
print("  Go to the CENTER of the room")
print("  This is where the drone will return to")
print("  Hold the camera steady")
print("")
print("  Press Enter when you are ready...")
print("")
input()

# Grab several frames and pick the most confident one
best_confidence = -1
best_translation = None
best_rotation = None

for attempt in range(30):
    if zed.grab() == sl.ERROR_CODE.SUCCESS:
        zed.get_position(pose)
        conf = pose.pose_confidence
        if conf > best_confidence:
            best_confidence = conf
            t = pose.get_translation().get()
            r = pose.get_rotation_vector()
            best_translation = t
            best_rotation = r

if best_translation is None:
    print("  ERROR: Could not get position from camera.")
    print("  Make sure the camera can see the room clearly.")
    input("Press Enter to close...")
    sys.exit(1)

origin = {
    "x":  float(best_translation[0]),
    "y":  float(best_translation[1]),
    "z":  float(best_translation[2]),
    "rx": float(best_rotation[0]),
    "ry": float(best_rotation[1]),
    "rz": float(best_rotation[2])
}

with open(ORIGIN_FILE, "w") as f:
    json.dump(origin, f, indent=2)

print("")
print("  Home position saved!")
print(f"  Confidence : {best_confidence}%")
print(f"  X          : {origin['x']:.3f} m")
print(f"  Y          : {origin['y']:.3f} m")
print(f"  Z          : {origin['z']:.3f} m")
print("")

if best_confidence < 30:
    print("  WARNING: Confidence is low.")
    print("  Point the camera at furniture or textured surfaces.")
    print("  Avoid pointing at blank walls or the floor.")
    print("")

# ── Step 2: Walk the room to build area memory ────────────────
print("=================================")
print("  STEP 2 OF 2 — MAP THE ROOM")
print("=================================")
print("")
print("  NOW WALK SLOWLY AROUND THE ROOM")
print("  - Cover all walls and corners")
print("  - Move slowly and smoothly")
print("  - Keep camera pointed forward")
print("  - Go back to center at the end")
print("")
print("  Press Enter when you are DONE walking")
print("")

frame_count  = 0
origin_saved = False

while True:
    if zed.grab() == sl.ERROR_CODE.SUCCESS:
        zed.get_position(pose)
        confidence = pose.pose_confidence
        t = pose.get_translation().get()
        frame_count += 1

        # Print status every 60 frames (about every 2 seconds)
        if frame_count % 60 == 0:
            print(f"  Mapping... "
                  f"frames: {frame_count:4d} | "
                  f"confidence: {confidence:3d}% | "
                  f"pos: X={t[0]:.2f} Y={t[1]:.2f} Z={t[2]:.2f}")

    # Check if Enter was pressed
    if msvcrt.kbhit():
        key = msvcrt.getch()
        if key == b'\r':
            break

print("")
print(f"  Mapping complete — {frame_count} frames captured")
print("")

# ── Save area file ────────────────────────────────────────────
print("  Saving rover_home.area ...")
print("  Please wait, do not close this window...")
print("")

save_status = zed.save_area_map(AREA_FILE)

if save_status == sl.ERROR_CODE.SUCCESS:
    print("  rover_home.area saved successfully!")
else:
    print(f"  Save returned: {save_status}")
    print("  The file may still have saved — check your folder.")

print("")
print("=================================")
print("  ALL DONE!")
print("=================================")
print("")
print(f"  Origin file : {ORIGIN_FILE}")
print(f"  Area file   : {AREA_FILE}")
print("")
print("  You can now run desktop_return_sim.py")
print("")

zed.close()
input("Press Enter to close...")