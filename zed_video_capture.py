#!/usr/bin/env python3
"""
zed_video_capture.py - ZED Video Recording with Quality Settings

Records video from ZED camera during flight with configurable quality.

Usage:
    python3 zed_video_capture.py [LOW|MEDIUM|HIGH|ULTRA]
    
Output:
    - ~/ZED Navigation/ZED-Camera-Navigation/captures/videos/YYYYMMDD_HHMMSS.svo
"""

import pyzed.sl as sl
import time
import os
import sys

# Quality settings
QUALITY_PRESETS = {
    "LOW": {
        "resolution": sl.RESOLUTION.VGA,           # 672x376
        "fps": 15,
        "compression": sl.SVO_COMPRESSION_MODE.H265,
        "bitrate": 2000  # kbps
    },
    "MEDIUM": {
        "resolution": sl.RESOLUTION.HD720,         # 1280x720
        "fps": 30,
        "compression": sl.SVO_COMPRESSION_MODE.H265,
        "bitrate": 4000
    },
    "HIGH": {
        "resolution": sl.RESOLUTION.HD1080,        # 1920x1080
        "fps": 30,
        "compression": sl.SVO_COMPRESSION_MODE.H264,
        "bitrate": 8000
    },
    "ULTRA": {
        "resolution": sl.RESOLUTION.HD2K,          # 2208x1242
        "fps": 30,
        "compression": sl.SVO_COMPRESSION_MODE.H264,
        "bitrate": 12000
    }
}

# Output directory
BASE_DIR = os.path.expanduser('~/ZED Navigation/ZED-Camera-Navigation')
VIDEOS_DIR = os.path.join(BASE_DIR, 'captures', 'videos')


def main():
    # Parse quality setting
    quality = sys.argv[1] if len(sys.argv) > 1 else "HIGH"
    quality = quality.upper()
    
    if quality not in QUALITY_PRESETS:
        print(f"Invalid quality: {quality}")
        print(f"Options: {', '.join(QUALITY_PRESETS.keys())}")
        sys.exit(1)
    
    preset = QUALITY_PRESETS[quality]
    
    print("=" * 50)
    print("  ZED VIDEO RECORDING")
    print("=" * 50)
    print(f"Quality: {quality}")
    print(f"Resolution: {preset['resolution']}")
    print(f"FPS: {preset['fps']}")
    print(f"Compression: {preset['compression']}")
    print(f"Bitrate: {preset['bitrate']} kbps")
    print("=" * 50)
    
    # Create output directory
    os.makedirs(VIDEOS_DIR, exist_ok=True)
    
    # Initialize ZED
    print("\nInitializing ZED camera...")
    zed = sl.Camera()
    
    init_params = sl.InitParameters()
    init_params.camera_resolution = preset['resolution']
    init_params.camera_fps = preset['fps']
    init_params.depth_mode = sl.DEPTH_MODE.NONE  # No depth for video
    
    err = zed.open(init_params)
    if err != sl.ERROR_CODE.SUCCESS:
        print(f"ERROR: Failed to open ZED: {err}")
        sys.exit(1)
    
    print("✓ Camera opened")
    
    # Setup recording
    print("\nStarting recording...")
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    video_path = os.path.join(VIDEOS_DIR, f"{timestamp}.svo")
    
    recording_params = sl.RecordingParameters()
    recording_params.compression_mode = preset['compression']
    recording_params.video_filename = video_path
    recording_params.bitrate = preset['bitrate']
    
    err = zed.enable_recording(recording_params)
    if err != sl.ERROR_CODE.SUCCESS:
        print(f"ERROR: Failed to start recording: {err}")
        zed.close()
        sys.exit(1)
    
    print(f"✓ Recording to: {video_path}")
    
    # Runtime parameters
    runtime_params = sl.RuntimeParameters()
    
    print("\nRECORDING...")
    print("Press Ctrl+C to stop\n")
    
    frame_count = 0
    start_time = time.time()
    
    try:
        while True:
            err = zed.grab(runtime_params)
            
            if err == sl.ERROR_CODE.SUCCESS:
                frame_count += 1
                
                # Print status every 30 frames
                if frame_count % 30 == 0:
                    elapsed = time.time() - start_time
                    fps = frame_count / elapsed if elapsed > 0 else 0
                    duration_min = elapsed / 60
                    
                    print(f"  Frames: {frame_count:5d} | "
                          f"FPS: {fps:5.1f} | "
                          f"Duration: {duration_min:.1f}m")
            
            time.sleep(0.001)
            
    except KeyboardInterrupt:
        print(f"\n\nStopping...")
    
    # Summary
    elapsed = time.time() - start_time
    duration_min = elapsed / 60
    
    print("\n" + "=" * 50)
    print("RECORDING COMPLETE")
    print("=" * 50)
    print(f"Duration: {duration_min:.2f} minutes")
    print(f"Frames: {frame_count}")
    print(f"Average FPS: {frame_count/elapsed:.1f}")
    print(f"File: {video_path}")
    
    # Get file size if it exists
    try:
        if os.path.exists(video_path):
            file_size = os.path.getsize(video_path) / (1024*1024)
            print(f"Size: {file_size:.1f} MB")
        else:
            print("Size: File still being written...")
    except Exception as e:
        print(f"Size: Unable to determine ({e})")
    
    print("=" * 50)
    
    zed.disable_recording()
    zed.close()

if __name__ == "__main__":
    main()