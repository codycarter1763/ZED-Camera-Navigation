# save_origin.py
# Does two things:
#   1. Records the center of the room as home (origin_T0.json)
#   2. Walks the room and saves area memory (rover_home.area)
#
# HOW TO USE:
#   Step 1 - Stand at center of room, press Enter to save home position
#   Step 2 - Walk slowly around the entire room covering all walls
#   Step 3 - Press Enter in terminal when done walking to save the area file
#
# A live camera + depth view window will open while mapping

import pyzed.sl as sl
import json
import os
import sys
import msvcrt
import cv2
import numpy as np

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
    print(f"  ERROR: Could not open camera: {status}")
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

# ── Enable positional tracking FIRST ─────────────────────────
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

# ── Enable spatial mapping SECOND ────────────────────────────
print("Enabling spatial mapping...")
mapping_params = sl.SpatialMappingParameters()
mapping_params.resolution_meter = 0.05
mapping_params.range_meter      = 5.0
mapping_params.save_texture     = False
mapping_params.map_type         = sl.SPATIAL_MAP_TYPE.FUSED_POINT_CLOUD

map_status = zed.enable_spatial_mapping(mapping_params)
if map_status != sl.ERROR_CODE.SUCCESS:
    print(f"  WARNING: Spatial mapping failed: {map_status}")
    print("  Continuing without mapping...")
else:
    print("  Spatial mapping enabled OK!")
print("")

# ── Image containers ──────────────────────────────────────────
left_image  = sl.Mat()
depth_image = sl.Mat()
pose        = sl.Pose()

# ── Stabilize ─────────────────────────────────────────────────
print("  Stabilizing tracking...")
print("  Point camera at the room and hold steady...")
print("  Live camera window opening...")
print("")

for i in range(60):
    if zed.grab() == sl.ERROR_CODE.SUCCESS:
        zed.retrieve_image(left_image, sl.VIEW.LEFT)
        frame = left_image.get_data()

        display = cv2.resize(frame, (960, 540))
        if display.shape[2] == 4:
            display = cv2.cvtColor(display, cv2.COLOR_BGRA2BGR)

        cv2.putText(display,
                    f"Stabilizing... {i}/60",
                    (20, 40), cv2.FONT_HERSHEY_SIMPLEX,
                    1.0, (0, 255, 0), 2)
        cv2.putText(display,
                    "Point camera at the room",
                    (20, 80), cv2.FONT_HERSHEY_SIMPLEX,
                    0.7, (255, 255, 255), 2)
        cv2.imshow("ZED Room Mapper", display)
        cv2.waitKey(1)

    if i % 15 == 0:
        print(f"  {i}/60 frames stabilized...")

print("  Stabilization done!")
print("")

# ── Step 1: Save origin ───────────────────────────────────────
print("=================================")
print("  STEP 1 OF 2 — SAVE HOME")
print("=================================")
print("")
print("  Go to the CENTER of the room")
print("  This is where the drone will return to")
print("  Hold the camera steady")
print("")
print("  Press Enter in the TERMINAL when ready...")
print("")
input()

best_confidence  = -1
best_translation = None
best_rotation    = None

for attempt in range(30):
    if zed.grab() == sl.ERROR_CODE.SUCCESS:
        zed.get_position(pose)
        conf = pose.pose_confidence
        if conf > best_confidence:
            best_confidence  = conf
            t                = pose.get_translation().get()
            r                = pose.get_rotation_vector()
            best_translation = t
            best_rotation    = r

if best_translation is None:
    print("  ERROR: Could not get position.")
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
    print("  Point camera at furniture or textured surfaces.")
    print("  Avoid pointing at blank walls or the floor.")
    print("")

# ── Step 2: Walk the room ─────────────────────────────────────
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
print("  LEFT  = live camera view")
print("  RIGHT = depth map (brighter = closer)")
print("")
print("  Press Enter in the TERMINAL when DONE walking")
print("")

frame_count = 0

while True:
    if zed.grab() == sl.ERROR_CODE.SUCCESS:
        zed.get_position(pose)
        zed.retrieve_image(left_image,  sl.VIEW.LEFT)
        zed.retrieve_image(depth_image, sl.VIEW.DEPTH)

        confidence = pose.pose_confidence
        t          = pose.get_translation().get()
        frame_count += 1

        # ── Build display ─────────────────────────────────────
        cam_frame   = left_image.get_data()
        depth_frame = depth_image.get_data()

        cam_small   = cv2.resize(cam_frame,   (640, 360))
        depth_small = cv2.resize(depth_frame, (640, 360))

        if cam_small.shape[2] == 4:
            cam_small = cv2.cvtColor(cam_small, cv2.COLOR_BGRA2BGR)

        if len(depth_small.shape) == 2:
            depth_small = cv2.cvtColor(depth_small, cv2.COLOR_GRAY2BGR)
        elif depth_small.shape[2] == 4:
            depth_small = cv2.cvtColor(depth_small, cv2.COLOR_BGRA2BGR)

        combined = np.hstack([cam_small, depth_small])

        # Confidence color and label
        if confidence >= 70:
            conf_color = (0, 220, 100)
            conf_text  = "TRACKING OK"
        elif confidence >= 30:
            conf_color = (0, 180, 255)
            conf_text  = "TRACKING..."
        else:
            conf_color = (0, 80, 220)
            conf_text  = "LOW CONFIDENCE"

        # Labels
        cv2.putText(combined, "CAMERA VIEW",
                    (20, 30), cv2.FONT_HERSHEY_SIMPLEX,
                    0.7, (255, 255, 255), 2)
        cv2.putText(combined, "DEPTH VIEW",
                    (660, 30), cv2.FONT_HERSHEY_SIMPLEX,
                    0.7, (255, 255, 255), 2)

        # Confidence status
        cv2.putText(combined,
                    f"Confidence: {confidence}%  {conf_text}",
                    (20, 340), cv2.FONT_HERSHEY_SIMPLEX,
                    0.65, conf_color, 2)

        # Position
        cv2.putText(combined,
                    f"X:{t[0]:.2f}  Y:{t[1]:.2f}  Z:{t[2]:.2f}  frames:{frame_count}",
                    (20, 695), cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, (200, 200, 200), 1)

        # Reminder
        cv2.putText(combined,
                    "Press Enter in TERMINAL when done",
                    (640, 340), cv2.FONT_HERSHEY_SIMPLEX,
                    0.55, (255, 255, 255), 1)

        cv2.imshow("ZED Room Mapper", combined)
        cv2.waitKey(1)

        # Print to terminal every 2 seconds
        if frame_count % 60 == 0:
            print(f"  Mapping... "
                  f"frames: {frame_count:4d} | "
                  f"confidence: {confidence:3d}% | "
                  f"X={t[0]:.2f} Y={t[1]:.2f} Z={t[2]:.2f}")

    # Check for Enter key in terminal
    if msvcrt.kbhit():
        key = msvcrt.getch()
        if key == b'\r':
            break

print("")
print(f"  Mapping complete — {frame_count} frames captured")
print("")

# ── Save files ────────────────────────────────────────────────
print("  Saving rover_home.area ...")
print("  Please wait, do not close this window...")
print("")

cv2.destroyAllWindows()

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

zed.disable_spatial_mapping()
zed.disable_positional_tracking()
zed.close()
input("Press Enter to close...")