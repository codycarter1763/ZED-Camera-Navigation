#!/bin/bash
# launch_drone.sh  --  Jetson tmux session launcher
# All files live in ~/ZED Navigation/
#
# Usage:
#   cd ~/ZED\ Navigation
#   chmod +x launch_drone.sh
#   ./launch_drone.sh           # real hardware
#   ./launch_drone.sh --sim     # SITL simulation
#
# tmux keys:
#   Ctrl+B, 0-5  -- switch window
#   Ctrl+B, d    -- detach (programs keep running)
#   Ctrl+B, [    -- scroll mode (q to exit)
#   tmux attach -t drone  -- reattach after detach

SESSION="drone"
WS="$HOME/ZED Navigation"
ROS_SETUP="/opt/ros/humble/setup.bash"
ZED_SETUP="$HOME/ros2_ws/install/local_setup.bash"

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
    echo "         Run: cd ~/ros2_ws && colcon build"
fi

# ── Window 0: ZED camera (real) or SITL bridge (sim) ----------
# ros2 launch does not support file paths with spaces so ZED
# and AprilTag are started as separate commands in windows 0 + 1
tmux new-session -d -s $SESSION -x 230 -y 55
tmux rename-window -t $SESSION:0 "zed"

if [ "$SIM" = true ]; then
    tmux send-keys -t $SESSION:0 \
        "$SRC && cd \"$WS\" && python3 sitl_to_zed_pose.py" Enter
else
    tmux send-keys -t $SESSION:0 \
        "$SRC && ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i" Enter
fi

# ── Window 1: AprilTag (real) or fake tag (sim) ---------------
tmux new-window -t $SESSION:1 -n "apriltag"

if [ "$SIM" = true ]; then
    tmux send-keys -t $SESSION:1 \
        "sleep 2 && $SRC && cd \"$WS\" && python3 fake_apriltag.py" Enter
else
    # 5 second delay gives ZED time to start publishing before apriltag connects
    tmux send-keys -t $SESSION:1 \
        "sleep 5 && $SRC && ros2 run apriltag_ros apriltag_node --ros-args \
-r image_rect:=/zed/zed_node/rgb/color/rect/image \
-r camera_info:=/zed/zed_node/rgb/color/rect/camera_info \
-r detections:=/detections \
--params-file $HOME/tags_config.yaml" Enter
fi

# ── Window 2: Program 1 -- set_origin -------------------------
tmux new-window -t $SESSION:2 -n "set_origin"
tmux send-keys -t $SESSION:2 \
    "sleep 6 && $SRC && cd \"$WS/ZED-Camera-Navigation\" && python3 set_origin.py" Enter

# ── Window 3: Program 2 -- save_position ----------------------
tmux new-window -t $SESSION:3 -n "save_pos"
tmux send-keys -t $SESSION:3 \
    "echo 'After scan mission loiters, run:' && echo '  cd ~/ZED Navigation/ZED-Camera-Navigation && python3 save_position.py'" Enter

# ── Window 4: Program 3 -- return_land ------------------------
tmux new-window -t $SESSION:4 -n "return_land"
tmux send-keys -t $SESSION:4 \
    "echo 'When return mission starts in MP, run:' && echo '  cd ~/ZED Navigation/ZED-Camera-Navigation && python3 return_land.py'" Enter

# ── Window 5: Topic monitor -----------------------------------
tmux new-window -t $SESSION:5 -n "monitor"
tmux send-keys -t $SESSION:5 "$SRC" Enter

tmux select-window -t $SESSION:0

echo ""
if [ "$SIM" = true ]; then
    echo "=== SIMULATION MODE ==="
else
    echo "=== REAL HARDWARE MODE ==="
fi
echo ""
echo "Workspace: ~/ZED Navigation/"
echo ""
echo "  Ctrl+B, 0  -- ZED camera"
echo "  Ctrl+B, 1  -- AprilTag detector"
echo "  Ctrl+B, 2  -- set_origin.py     (SPACE to save origin)"
echo "  Ctrl+B, 3  -- save_position.py  (run after scan loiters)"
echo "  Ctrl+B, 4  -- return_land.py    (run when return mission starts)"
echo "  Ctrl+B, 5  -- monitor terminal"
echo ""
echo "Topic checks (window 5):"
echo "  ros2 topic hz /zed/zed_node/pose"
echo "  ros2 topic hz /zed/zed_node/depth/depth_registered"
echo "  ros2 topic hz /detections"
echo ""

# Note: tmux attach removed — script runs non-interactively under systemd
# To attach manually after boot: tmux attach -t drone
