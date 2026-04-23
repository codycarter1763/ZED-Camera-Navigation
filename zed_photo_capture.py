#!/usr/bin/env python3
"""
zed_photo_capture.py - ZED Photo Capture via ROS2 Topics

Captures still images from the running ZED ROS2 wrapper with configurable quality.
Subscribes to /zed/zed_node/rgb/image_rect_color topic.

Usage:
    python3 zed_photo_capture.py [LOW|MEDIUM|HIGH|ULTRA]
    
Output:
    - ~/ZED Navigation/ZED-Camera-Navigation/captures/photos/YYYYMMDD_HHMMSS_####.jpg
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time
import os
import sys
import threading
import signal

# Quality settings (JPG quality only - resolution comes from ZED node)
QUALITY_PRESETS = {
    "LOW": {
        "jpg_quality": 75,
        "interval": 2.0  # seconds between captures
    },
    "MEDIUM": {
        "jpg_quality": 85,
        "interval": 1.5
    },
    "HIGH": {
        "jpg_quality": 90,
        "interval": 1.0
    },
    "ULTRA": {
        "jpg_quality": 95,
        "interval": 0.5
    }
}

# Output directory
BASE_DIR = os.path.expanduser('~/ZED Navigation/ZED-Camera-Navigation')
PHOTOS_DIR = os.path.join(BASE_DIR, 'captures', 'photos')

# Shared state
class State:
    def __init__(self):
        self.lock = threading.Lock()
        self.last_image = None
        self.image_count = 0
        self.running = True

S = State()


class PhotoCaptureNode(Node):
    def __init__(self):
        super().__init__('photo_capture')
        self.bridge = CvBridge()
        
        # Subscribe to ZED rectified color image
        self.subscription = self.create_subscription(
            Image,
            '/zed/zed_node/rgb/color/rect/image',
            self.image_callback,
            10)
        
        self.get_logger().info('Subscribed to /zed/zed_node/rgb/color/rect/image')
    
    def image_callback(self, msg):
        try:
            # Convert ROS Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            with S.lock:
                S.last_image = cv_image
                S.image_count += 1
                
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')


def main():
    # Parse quality setting
    quality = sys.argv[1] if len(sys.argv) > 1 else "HIGH"
    quality = quality.upper()
    
    if quality not in QUALITY_PRESETS:
        print(f"Invalid quality: {quality}")
        print(f"Options: {', '.join(QUALITY_PRESETS.keys())}")
        sys.exit(1)
    
    preset = QUALITY_PRESETS[quality]
    
    # Signal handler for graceful shutdown
    def signal_handler(sig, frame):
        S.running = False
    
    signal.signal(signal.SIGTERM, signal_handler)
    signal.signal(signal.SIGINT, signal_handler)
    
    print("=" * 50)
    print("  ZED PHOTO CAPTURE (ROS2)")
    print("=" * 50)
    print(f"Quality: {quality}")
    print(f"JPG Quality: {preset['jpg_quality']}%")
    print(f"Interval: {preset['interval']}s")
    print(f"Topic: /zed/zed_node/rgb/color/rect/image")
    print("=" * 50)
    
    # Create output directory
    os.makedirs(PHOTOS_DIR, exist_ok=True)
    
    # Initialize ROS2
    rclpy.init()
    node = PhotoCaptureNode()
    
    # Spin ROS2 in background thread
    def ros_spin():
        try:
            rclpy.spin(node)
        except ExternalShutdownException:
            pass  # Normal shutdown, ignore
    
    ros_thread = threading.Thread(target=ros_spin, daemon=True)
    ros_thread.start()
    
    # Wait for first image
    print("\nWaiting for images from ZED...")
    time.sleep(2)
    
    with S.lock:
        if S.last_image is None:
            print("ERROR: No images received from ZED!")
            print("Make sure ZED ROS2 wrapper is running.")
            rclpy.shutdown()
            sys.exit(1)
    
    print("✓ Receiving images from ZED")
    
    # Session info
    session_id = time.strftime("%Y%m%d_%H%M%S")
    photo_count = 0
    last_capture = 0
    
    print(f"\nCapturing photos to: {PHOTOS_DIR}/")
    print("Press Ctrl+C to stop\n")
    
    try:
        while S.running:
            current_time = time.time()
            
            # Check if enough time has passed
            if current_time - last_capture >= preset['interval']:
                with S.lock:
                    if S.last_image is not None:
                        img = S.last_image.copy()
                    else:
                        time.sleep(0.1)
                        continue
                
                # Save image
                filename = f"{session_id}_{photo_count:04d}.jpg"
                filepath = os.path.join(PHOTOS_DIR, filename)
                
                cv2.imwrite(
                    filepath,
                    img,
                    [cv2.IMWRITE_JPEG_QUALITY, preset['jpg_quality']]
                )
                
                photo_count += 1
                last_capture = current_time
                
                print(f"  [{photo_count:4d}] {filename}")
            
            time.sleep(0.05)
            
    except KeyboardInterrupt:
        print(f"\n\nStopping...")
        S.running = False
    
    # Summary
    print("\n" + "=" * 50)
    print("CAPTURE COMPLETE")
    print("=" * 50)
    print(f"Photos captured: {photo_count}")
    print(f"Location: {PHOTOS_DIR}/")
    print(f"Session ID: {session_id}")
    
    # List file sizes
    total_size = 0
    for f in os.listdir(PHOTOS_DIR):
        if f.startswith(session_id):
            size = os.path.getsize(os.path.join(PHOTOS_DIR, f))
            total_size += size
    
    print(f"Total size: {total_size / (1024*1024):.1f} MB")
    print("=" * 50)
    
    # Cleanup
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()