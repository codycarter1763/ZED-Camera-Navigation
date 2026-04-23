#!/usr/bin/env python3
"""
zed_video_capture.py - ZED Video Recording via ROS2 Topics

Records video from the running ZED ROS2 wrapper with configurable quality.
Subscribes to /zed/zed_node/rgb/image_rect_color topic.

Usage:
    python3 zed_video_capture.py [LOW|MEDIUM|HIGH|ULTRA]
    
Output:
    - ~/ZED Navigation/ZED-Camera-Navigation/captures/videos/YYYYMMDD_HHMMSS.avi
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

# Quality settings
QUALITY_PRESETS = {
    "LOW": {
        "fps": 15,
        "bitrate": 2000000,  # 2 Mbps
        "codec": "XVID"  # Software codec
    },
    "MEDIUM": {
        "fps": 30,
        "bitrate": 4000000,  # 4 Mbps
        "codec": "XVID"  # Software codec
    },
    "HIGH": {
        "fps": 30,
        "bitrate": 8000000,  # 8 Mbps
        "codec": "XVID"  # Software codec
    },
    "ULTRA": {
        "fps": 30,
        "bitrate": 12000000,  # 12 Mbps
        "codec": "XVID"  # Software codec
    }
}

# Output directory
BASE_DIR = os.path.expanduser('~/ZED Navigation/ZED-Camera-Navigation')
VIDEOS_DIR = os.path.join(BASE_DIR, 'captures', 'videos')

# Shared state
class State:
    def __init__(self):
        self.lock = threading.Lock()
        self.last_image = None
        self.image_count = 0
        self.running = True
        self.writer = None

S = State()


class VideoCaptureNode(Node):
    def __init__(self):
        super().__init__('video_capture')
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
                
                # Write frame to video if writer is active
                if S.writer is not None:
                    S.writer.write(cv_image)
                    
        except Exception as e:
            self.get_logger().error(f'Failed to process image: {e}')


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
    print("  ZED VIDEO RECORDING (ROS2)")
    print("=" * 50)
    print(f"Quality: {quality}")
    print(f"FPS: {preset['fps']}")
    print(f"Bitrate: {preset['bitrate']/1000000:.1f} Mbps")
    print(f"Codec: {preset['codec']}")
    print(f"Topic: /zed/zed_node/rgb/color/rect/image")
    print("=" * 50)
    
    # Create output directory
    os.makedirs(VIDEOS_DIR, exist_ok=True)
    
    # Initialize ROS2
    rclpy.init()
    node = VideoCaptureNode()
    
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
        
        # Get image dimensions
        height, width = S.last_image.shape[:2]
    
    print(f"✓ Receiving images from ZED ({width}x{height})")
    
    # Setup video writer
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    video_path = os.path.join(VIDEOS_DIR, f"{timestamp}.avi")  # .avi for XVID
    
    # Define codec
    fourcc = cv2.VideoWriter_fourcc(*preset['codec'])
    
    with S.lock:
        S.writer = cv2.VideoWriter(
            video_path,
            fourcc,
            preset['fps'],
            (width, height))
    
    if not S.writer.isOpened():
        print("ERROR: Failed to create video writer!")
        rclpy.shutdown()
        sys.exit(1)
    
    print(f"✓ Recording to: {video_path}")
    print("\nRECORDING...")
    print("Press Ctrl+C to stop\n")
    
    frame_count = 0
    start_time = time.time()
    
    try:
        while S.running:
            with S.lock:
                current_count = S.image_count
            
            # Print status every 30 frames
            if current_count > frame_count and current_count % 30 == 0:
                elapsed = time.time() - start_time
                fps = current_count / elapsed if elapsed > 0 else 0
                duration_min = elapsed / 60
                
                print(f"  Frames: {current_count:5d} | "
                      f"FPS: {fps:5.1f} | "
                      f"Duration: {duration_min:.1f}m")
                
                frame_count = current_count
            
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        print(f"\n\nStopping...")
        S.running = False
    
    # Close video writer
    with S.lock:
        if S.writer is not None:
            S.writer.release()
            S.writer = None
    
    # Summary
    elapsed = time.time() - start_time
    duration_min = elapsed / 60
    
    print("\n" + "=" * 50)
    print("RECORDING COMPLETE")
    print("=" * 50)
    print(f"Duration: {duration_min:.2f} minutes")
    
    with S.lock:
        print(f"Frames: {S.image_count}")
        if elapsed > 0:
            print(f"Average FPS: {S.image_count/elapsed:.1f}")
    
    print(f"File: {video_path}")
    
    # Get file size
    try:
        if os.path.exists(video_path):
            file_size = os.path.getsize(video_path) / (1024*1024)
            print(f"Size: {file_size:.1f} MB")
    except Exception as e:
        print(f"Size: Unable to determine ({e})")
    
    print("=" * 50)
    
    # Cleanup
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()