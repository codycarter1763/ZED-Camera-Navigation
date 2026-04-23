#!/bin/bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
python3 "$(dirname "$0")/zed_photo_capture.py" "$@"
