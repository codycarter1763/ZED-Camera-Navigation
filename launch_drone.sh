#!/bin/bash
# launch_drone.sh  --  Jetson tmux session launcher
# Workspace: /home/clarq/ZED Navigation/ZED-Camera-Navigation
#
# Usage:
#   chmod +x launch_drone.sh
#   ./launch_drone.sh           # real hardware
#   ./launch_drone.sh --sim     # SITL simulation
#
# tmux keys:
#   Ctrl+B, 0  -- ZED camera
#   Ctrl+B, 1  -- AprilTag detector
#   Ctrl+B, 2  -- set_origin.py
#   Ctrl+B, 3  -- save_position.py
#   Ctrl+B, 4  -- return_land.py
#   Ctrl+B, 5  -- monitor terminal
#   Ctrl+B, 6  -- MAVROS (Pixhawk USB bridge)
#   Ctrl+B, 7  -- mavlink_trigger_listener.py
#   Ctrl+B, d  -- detach (programs keep running)
#   Ctrl+B, [  -- scroll mode (q to exit)
#   tmux attach -t drone  -- reattach

SESSION="drone"
WS="/home/clarq/ZED Navigation/ZED-Camera-Navigation"
ROS_SETUP="/opt/ros/humble/setup.bash"
ZED_SETUP="/home/clarq/ros2_ws/install/local_setup.bash"
TAGS_CFG="/home/clarq/ZED Navigation/ZED-Camera-Navigation/tags_config.yaml"

SIM=false
[[ "$1" == "--sim" ]] && SIM=true

# Kill any existing session
tmux kill-session -t $SESSION 2>/dev/null
sleep 0.5

# Build source string
if [ -f "$ZED_SETUP" ]; then
    SRC="source $ROS_SETUP && source $ZED_SETUP"
else
    SRC="source $ROS_SETUP"
    echo "WARNING: ZED workspace not found at $ZED_SETUP"
fi

# ── Window 0: ZED camera ---------------------------------------
tmux new-session -d -s $SESSION -x 230 -y 55
tmux rename-window -t $SESSION:0 "zed"

if [ "$SIM" = true ]; then
    tmux send-keys -t $SESSION:0 \
        "$SRC && cd \"$WS\" && python3 sitl_to_zed_pose.py" Enter
else
    tmux send-keys -t $SESSION:0 \
        "$SRC && ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i" Enter
fi

# ── Window 1: AprilTag detector --------------------------------
# 10 second delay to let ZED fully start before apriltag connects
tmux new-window -t $SESSION:1 -n "apriltag"

if [ "$SIM" = true ]; then
    tmux send-keys -t $SESSION:1 \
        "sleep 3 && $SRC && cd \"$WS\" && python3 fake_apriltag.py" Enter
else
    tmux send-keys -t $SESSION:1 \
        "sleep 10 && $SRC && ros2 run apriltag_ros apriltag_node --ros-args \
-r image_rect:=/zed/zed_node/rgb/color/rect/image \
-r camera_info:=/zed/zed_node/rgb/color/rect/camera_info \
-r detections:=/detections \
--params-file '$TAGS_CFG'" Enter
fi

# ── Window 2: Program 1 -- set_origin -------------------------
# 12 second delay so ZED and apriltag are ready first
tmux new-window -t $SESSION:2 -n "set_origin"
tmux send-keys -t $SESSION:2 \
    "sleep 12 && $SRC && cd \"$WS\" && python3 set_origin.py" Enter

# ── Window 3: Program 2 -- save_position ----------------------
tmux new-window -t $SESSION:3 -n "save_pos"
tmux send-keys -t $SESSION:3 \
    "cd \"$WS\" && echo '' && \
     echo '=============================' && \
     echo ' STEP 3 — AFTER SCAN LOITERS' && \
     echo '=============================' && \
     echo 'Run:  python3 save_position.py' && \
     echo '=============================' && \
     echo ''" Enter

# ── Window 4: Program 3 -- return_land ------------------------
tmux new-window -t $SESSION:4 -n "return_land"
tmux send-keys -t $SESSION:4 \
    "cd \"$WS\" && echo '' && \
     echo '==============================' && \
     echo ' STEP 5 — WHEN RETURN STARTS ' && \
     echo '==============================' && \
     echo 'Run:  python3 return_land.py  ' && \
     echo '==============================' && \
     echo ''" Enter

# ── Window 5: Topic monitor ------------------------------------
tmux new-window -t $SESSION:5 -n "monitor"
tmux send-keys -t $SESSION:5 \
    "$SRC && echo '' && \
     echo 'Topic monitor ready.' && \
     echo 'Useful commands:' && \
     echo '  ros2 topic hz /zed/zed_node/pose' && \
     echo '  ros2 topic hz /zed/zed_node/depth/depth_registered' && \
     echo '  ros2 topic hz /detections' && \
     echo '  ros2 topic list | grep zed' && \
     echo '  ros2 topic echo /detections' && \
     echo ''" Enter

# ── Window 6: MAVROS — bridges Pixhawk USB to ROS2 ------------
# Required so return_land.py can receive GUI commands via
# /mavros/statustext/recv and send velocity commands to Pixhawk
tmux new-window -t $SESSION:6 -n "mavros"
tmux send-keys -t $SESSION:6 \
    "sleep 5 && $SRC && \
     ros2 launch mavros apm.launch \
     fcu_url:=/dev/ttyACM0:115200" Enter

# ── Window 7: MAVLink trigger listener -------------------------
# Listens for CLARQ_* commands from Windows GUI via MAVROS
# When CLARQ_LAND received:
#   1. Starts apriltag_ros node
#   2. Starts return_land.py
# Must start after MAVROS (window 6) is ready
tmux new-window -t $SESSION:7 -n "trigger"
tmux send-keys -t $SESSION:7 \
    "sleep 8 && $SRC && cd \"$WS\" && python3 mavlink_trigger_listener.py" Enter

tmux select-window -t $SESSION:0

echo ""
if [ "$SIM" = true ]; then
    echo "=== SIMULATION MODE ==="
else
    echo "=== REAL HARDWARE MODE ==="
fi
echo ""
echo "Workspace: $WS"
echo ""
echo "  Ctrl+B, 0  -- ZED camera (starting now)"
echo "  Ctrl+B, 1  -- AprilTag  (starts in 10 sec)"
echo "  Ctrl+B, 2  -- set_origin.py (starts in 12 sec) -- PRESS SPACE to save"
echo "  Ctrl+B, 3  -- save_position.py -- run after scan loiters"
echo "  Ctrl+B, 4  -- return_land.py  -- run when return mission starts"
echo "  Ctrl+B, 5  -- monitor terminal"
echo "  Ctrl+B, 6  -- MAVROS (Pixhawk USB — starts in 5 sec)"
echo "  Ctrl+B, 7  -- MAVLink trigger listener (starts in 8 sec)"
echo ""
echo "  GUI command flow:"
echo "    START PREC LAND → CLARQ_LAND → Pixhawk → MAVROS"
echo "    → trigger listener → starts apriltag + return_land"
echo ""
echo "  Ctrl+B, d  -- detach session"
echo "  tmux attach -t drone  -- reattach"
echo ""

tmux attach -t $SESSION