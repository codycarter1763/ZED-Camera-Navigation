#!/usr/bin/env python3
# ─────────────────────────────────────────────────────────────
# zed_apriltag.launch.py
# Launches both the ZED ROS2 wrapper and the AprilTag detector
# in one command so you don't need two terminals.
#
# Usage:
#   ros2 launch ~/ZED\ Navigation/zed_apriltag.launch.py
#   ros2 launch ~/ZED\ Navigation/zed_apriltag.launch.py camera_model:=zed2i
#
# Correct topic names published after launch:
#   /zed/zed_node/pose                          ← position tracking
#   /zed/zed_node/rgb/color/rect/image          ← rectified colour image
#   /zed/zed_node/left/color/rect/camera_info   ← camera intrinsics
#   /zed/zed_node/depth/depth_registered        ← depth map (metres, float32)
#   /detections                                 ← AprilTag detections
# ─────────────────────────────────────────────────────────────

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    camera_model_arg = DeclareLaunchArgument(
        'camera_model',
        default_value='zed2',
        description='ZED camera model: zed, zedm, zed2, zed2i, zedx, zedxm'
    )

    tags_config = os.path.expanduser('~/ZED Navigation/tags_config.yaml')

    # ── ZED wrapper ───────────────────────────────────────────
    zed_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('zed_wrapper'), 'launch', 'zed_camera.launch.py'
            ])
        ]),
        launch_arguments={
            'camera_model':        LaunchConfiguration('camera_model'),
            'camera_name':         'zed',
            'node_name':           'zed_node',
            'publish_tf':          'true',
            'publish_map_tf':      'true',
            # Enable depth — required for depth-based distance measurement
            'depth.depth_mode':    '1',           # 1=PERFORMANCE, 3=QUALITY
            # HD720 gives good balance of FOV and frame rate on Jetson
            'general.grab_resolution': 'HD720',
            'general.grab_frame_rate': '30',
        }.items()
    )

    # ── AprilTag detector ─────────────────────────────────────
    # Remaps ZED image topics into apriltag_ros expected topic names
    apriltag_node = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag',
        remappings=[
            # ZED rectified colour image → apriltag input
            ('image_rect',   '/zed/zed_node/rgb/color/rect/image'),
            # ZED camera info → apriltag calibration
            ('camera_info',  '/zed/zed_node/left/color/rect/camera_info'),
            # Output detections on /detections (what our scripts subscribe to)
            ('detections',   '/detections'),
        ],
        parameters=[tags_config],
        output='screen'
    )

    return LaunchDescription([
        camera_model_arg,
        zed_launch,
        apriltag_node,
    ])
