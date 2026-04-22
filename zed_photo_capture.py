# #!/usr/bin/env python3
# """
# zed_photo_capture.py - ZED Photo Capture with Quality Settings

# Captures still images from ZED camera during flight with configurable quality.
# Images are saved with GPS coordinates (if available) and timestamps.

# Usage:
#     python3 zed_photo_capture.py [LOW|MEDIUM|HIGH|ULTRA]
    
# Output:
#     - ~/ZED Navigation/ZED-Camera-Navigation/captures/photos/YYYYMMDD_HHMMSS_####.jpg
# """

# import pyzed.sl as sl
# import cv2
# import time
# import os
# import sys

# # Quality settings
# QUALITY_PRESETS = {
#     "LOW": {
#         "resolution": sl.RESOLUTION.VGA,      # 672x376
#         "jpg_quality": 75,
#         "interval": 2.0  # seconds between captures
#     },
#     "MEDIUM": {
#         "resolution": sl.RESOLUTION.HD720,    # 1280x720
#         "jpg_quality": 85,
#         "interval": 1.5
#     },
#     "HIGH": {
#         "resolution": sl.RESOLUTION.HD1080,   # 1920x1080
#         "jpg_quality": 90,
#         "interval": 1.0
#     },
#     "ULTRA": {
#         "resolution": sl.RESOLUTION.HD2K,     # 2208x1242
#         "jpg_quality": 95,
#         "interval": 0.5
#     }
# }

# # Output directory
# BASE_DIR = os.path.expanduser('~/ZED Navigation/ZED-Camera-Navigation')
# PHOTOS_DIR = os.path.join(BASE_DIR, 'captures', 'photos')


# def main():
#     # Parse quality setting
#     quality = sys.argv[1] if len(sys.argv) > 1 else "HIGH"
#     quality = quality.upper()
    
#     if quality not in QUALITY_PRESETS:
#         print(f"Invalid quality: {quality}")
#         print(f"Options: {', '.join(QUALITY_PRESETS.keys())}")
#         sys.exit(1)
    
#     preset = QUALITY_PRESETS[quality]
    
#     print("=" * 50)
#     print("  ZED PHOTO CAPTURE")
#     print("=" * 50)
#     print(f"Quality: {quality}")
#     print(f"Resolution: {preset['resolution']}")
#     print(f"JPG Quality: {preset['jpg_quality']}%")
#     print(f"Interval: {preset['interval']}s")
#     print("=" * 50)
    
#     # Create output directory
#     os.makedirs(PHOTOS_DIR, exist_ok=True)
    
#     # Initialize ZED
#     print("\nInitializing ZED camera...")
#     zed = sl.Camera()
    
#     init_params = sl.InitParameters()
#     init_params.camera_resolution = preset['resolution']
#     init_params.camera_fps = 30
#     init_params.depth_mode = sl.DEPTH_MODE.NONE  # No depth for photos
    
#     err = zed.open(init_params)
#     if err != sl.ERROR_CODE.SUCCESS:
#         print(f"ERROR: Failed to open ZED: {err}")
#         sys.exit(1)
    
#     print("✓ Camera opened")
    
#     # Prepare image container
#     image = sl.Mat()
#     runtime_params = sl.RuntimeParameters()
    
#     # Session info
#     session_id = time.strftime("%Y%m%d_%H%M%S")
#     photo_count = 0
#     last_capture = 0
    
#     print(f"\nCapturing photos to: {PHOTOS_DIR}/")
#     print("Press Ctrl+C to stop\n")
    
#     try:
#         while True:
#             if zed.grab(runtime_params) == sl.ERROR_CODE.SUCCESS:
#                 current_time = time.time()
                
#                 # Check if enough time has passed
#                 if current_time - last_capture >= preset['interval']:
#                     # Retrieve left RGB image
#                     zed.retrieve_image(image, sl.VIEW.LEFT)
                    
#                     # Convert to numpy/opencv format
#                     img_np = image.get_data()
                    
#                     # Convert RGBA to RGB
#                     img_rgb = cv2.cvtColor(img_np, cv2.COLOR_RGBA2RGB)
                    
#                     # Save image
#                     filename = f"{session_id}_{photo_count:04d}.jpg"
#                     filepath = os.path.join(PHOTOS_DIR, filename)
                    
#                     cv2.imwrite(
#                         filepath,
#                         img_rgb,
#                         [cv2.IMWRITE_JPEG_QUALITY, preset['jpg_quality']]
#                     )
                    
#                     photo_count += 1
#                     last_capture = current_time
                    
#                     print(f"  [{photo_count:4d}] {filename}")
            
#             time.sleep(0.01)
            
#     except KeyboardInterrupt:
#         print(f"\n\nStopping...")
    
#     # Summary
#     print("\n" + "=" * 50)
#     print("CAPTURE COMPLETE")
#     print("=" * 50)
#     print(f"Photos captured: {photo_count}")
#     print(f"Location: {PHOTOS_DIR}/")
#     print(f"Session ID: {session_id}")
    
#     # List file sizes
#     total_size = 0
#     for f in os.listdir(PHOTOS_DIR):
#         if f.startswith(session_id):
#             size = os.path.getsize(os.path.join(PHOTOS_DIR, f))
#             total_size += size
    
#     print(f"Total size: {total_size / (1024*1024):.1f} MB")
#     print("=" * 50)
    
#     zed.close()

# if __name__ == "__main__":
#     main()








#REMOTE
#!/usr/bin/env python3
"""
zed_photo_capture.py - ZED Photo Capture with Quality Settings

Captures still images from ZED camera during flight with configurable quality.
Images are saved with GPS coordinates (if available) and timestamps.

Usage:
    python3 zed_photo_capture.py [LOW|MEDIUM|HIGH|ULTRA]
    
Output:
    - ~/ZED Navigation/ZED-Camera-Navigation/captures/photos/YYYYMMDD_HHMMSS_####.jpg
"""

import pyzed.sl as sl
import cv2
import time
import os
import sys

# Quality settings
QUALITY_PRESETS = {
    "LOW": {
        "resolution": sl.RESOLUTION.VGA,      # 672x376
        "jpg_quality": 75,
        "interval": 2.0  # seconds between captures
    },
    "MEDIUM": {
        "resolution": sl.RESOLUTION.HD720,    # 1280x720
        "jpg_quality": 85,
        "interval": 1.5
    },
    "HIGH": {
        "resolution": sl.RESOLUTION.HD1080,   # 1920x1080
        "jpg_quality": 90,
        "interval": 1.0
    },
    "ULTRA": {
        "resolution": sl.RESOLUTION.HD2K,     # 2208x1242
        "jpg_quality": 95,
        "interval": 0.5
    }
}

# Output directory
BASE_DIR = os.path.expanduser('~/ZED Navigation/ZED-Camera-Navigation')
PHOTOS_DIR = os.path.join(BASE_DIR, 'captures', 'photos')


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
    print("  ZED PHOTO CAPTURE")
    print("=" * 50)
    print(f"Quality: {quality}")
    print(f"Resolution: {preset['resolution']}")
    print(f"JPG Quality: {preset['jpg_quality']}%")
    print(f"Interval: {preset['interval']}s")
    print("=" * 50)
    
    # Create output directory
    os.makedirs(PHOTOS_DIR, exist_ok=True)
    
    # Initialize ZED
    print("\nInitializing ZED camera...")
    zed = sl.Camera()
    
    init_params = sl.InitParameters()
    init_params.camera_resolution = preset['resolution']
    init_params.camera_fps = 30
    init_params.depth_mode = sl.DEPTH_MODE.NONE  # No depth for photos
    
    err = zed.open(init_params)
    if err != sl.ERROR_CODE.SUCCESS:
        print(f"ERROR: Failed to open ZED: {err}")
        sys.exit(1)
    
    print("✓ Camera opened")
    
    # Prepare image container
    image = sl.Mat()
    runtime_params = sl.RuntimeParameters()
    
    # Session info
    session_id = time.strftime("%Y%m%d_%H%M%S")
    photo_count = 0
    last_capture = 0
    
    print(f"\nCapturing photos to: {PHOTOS_DIR}/")
    print("Press Ctrl+C to stop\n")
    
    try:
        while True:
            if zed.grab(runtime_params) == sl.ERROR_CODE.SUCCESS:
                current_time = time.time()
                
                # Check if enough time has passed
                if current_time - last_capture >= preset['interval']:
                    # Retrieve left RGB image
                    zed.retrieve_image(image, sl.VIEW.LEFT)
                    
                    # Convert to numpy/opencv format
                    img_np = image.get_data()
                    
                    # Convert RGBA to RGB
                    img_rgb = cv2.cvtColor(img_np, cv2.COLOR_RGBA2RGB)
                    
                    # Save image
                    filename = f"{session_id}_{photo_count:04d}.jpg"
                    filepath = os.path.join(PHOTOS_DIR, filename)
                    
                    cv2.imwrite(
                        filepath,
                        img_rgb,
                        [cv2.IMWRITE_JPEG_QUALITY, preset['jpg_quality']]
                    )
                    
                    photo_count += 1
                    last_capture = current_time
                    
                    print(f"  [{photo_count:4d}] {filename}")
            
            time.sleep(0.01)
            
    except KeyboardInterrupt:
        print(f"\n\nStopping...")
    
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
    
    zed.close()

if __name__ == "__main__":
    main()
    