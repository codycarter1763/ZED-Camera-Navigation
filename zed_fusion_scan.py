# #!/usr/bin/env python3
# """
# zed_fusion_scan.py - ZED Fusion 3D Reconstruction + Recording

# Records ZED camera feed to .SVO file and performs real-time 3D reconstruction
# using spatial mapping. Saves both the video file and extracted 3D mesh.

# Usage:
#     python3 zed_fusion_scan.py
    
# Output:
#     - ~/ZED Navigation/ZED-Camera-Navigation/scans/scan_YYYYMMDD_HHMMSS.svo
#     - ~/ZED Navigation/ZED-Camera-Navigation/scans/mesh_YYYYMMDD_HHMMSS.obj
# """

# import pyzed.sl as sl
# import time
# import os
# import sys

# # Output directory - must match clarq_rf_listener.py
# BASE_DIR = os.path.expanduser('~/ZED Navigation/ZED-Camera-Navigation')
# SCANS_DIR = os.path.join(BASE_DIR, 'scans')

# def main():
#     print("=" * 50)
#     print("  ZED FUSION 3D SCAN")
#     print("=" * 50)
    
#     # Create output directory
#     os.makedirs(SCANS_DIR, exist_ok=True)
    
#     # Initialize ZED camera
#     print("\n[1/5] Initializing ZED camera...")
#     zed = sl.Camera()
    
#     # Camera configuration
#     init_params = sl.InitParameters()
#     init_params.camera_resolution = sl.RESOLUTION.HD720  # 720p for balance
#     init_params.depth_mode = sl.DEPTH_MODE.NEURAL        # AI-powered depth (replaces ULTRA)
#     init_params.coordinate_units = sl.UNIT.METER         # Meters
#     init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP
    
#     # Open camera
#     err = zed.open(init_params)
#     if err != sl.ERROR_CODE.SUCCESS:
#         print(f"ERROR: Failed to open ZED camera: {err}")
#         sys.exit(1)
    
#     print("✓ ZED camera opened")
    
#     # Enable positional tracking (required for spatial mapping)
#     print("\n[2/5] Enabling positional tracking...")
#     tracking_params = sl.PositionalTrackingParameters()
#     tracking_params.enable_imu_fusion = True  # Use IMU for better tracking
    
#     err = zed.enable_positional_tracking(tracking_params)
#     if err != sl.ERROR_CODE.SUCCESS:
#         print(f"ERROR: Failed to enable tracking: {err}")
#         zed.close()
#         sys.exit(1)
    
#     print("✓ Positional tracking enabled")
    
#     # Enable spatial mapping (3D reconstruction)
#     print("\n[3/5] Enabling spatial mapping...")
#     mapping_params = sl.SpatialMappingParameters()
#     mapping_params.resolution_meter = 0.05        # 5cm voxel resolution
#     mapping_params.range_meter = 10.0             # Map up to 10m range
#     mapping_params.save_texture = True            # Save RGB texture
#     mapping_params.use_chunk_only = False         # Use full map
#     mapping_params.map_type = sl.SPATIAL_MAP_TYPE.MESH  # Output as mesh
    
#     err = zed.enable_spatial_mapping(mapping_params)
#     if err != sl.ERROR_CODE.SUCCESS:
#         print(f"ERROR: Failed to enable mapping: {err}")
#         zed.close()
#         sys.exit(1)
    
#     print("✓ Spatial mapping enabled")
#     print(f"  Resolution: {mapping_params.resolution_meter}m")
#     print(f"  Range: {mapping_params.range_meter}m")
    
#     # Setup recording
#     print("\n[4/5] Starting recording...")
#     timestamp = time.strftime("%Y%m%d_%H%M%S")
#     svo_path = os.path.join(SCANS_DIR, f"scan_{timestamp}.svo")
#     mesh_path = os.path.join(SCANS_DIR, f"mesh_{timestamp}.obj")
    
#     recording_params = sl.RecordingParameters()
#     recording_params.compression_mode = sl.SVO_COMPRESSION_MODE.H264
#     recording_params.video_filename = svo_path
    
#     err = zed.enable_recording(recording_params)
#     if err != sl.ERROR_CODE.SUCCESS:
#         print(f"ERROR: Failed to start recording: {err}")
#         zed.close()
#         sys.exit(1)
    
#     print(f"✓ Recording to: {svo_path}")
    
#     # Runtime parameters
#     runtime_params = sl.RuntimeParameters()
#     runtime_params.confidence_threshold = 50  # Filter noisy depth
    
#     print("\n[5/5] 3D FUSION ACTIVE")
#     print("=" * 50)
#     print("Recording 3D scan...")
#     print("Press Ctrl+C to stop and save")
#     print("=" * 50)
    
#     frame_count = 0
#     start_time = time.time()
    
#     try:
#         while True:
#             # Grab frame
#             err = zed.grab(runtime_params)
            
#             if err == sl.ERROR_CODE.SUCCESS:
#                 frame_count += 1
                
#                 # Update spatial map
#                 mapping_state = zed.get_spatial_mapping_state()
                
#                 # Print status every 30 frames (~1 second at 30fps)
#                 if frame_count % 30 == 0:
#                     elapsed = time.time() - start_time
#                     fps = frame_count / elapsed if elapsed > 0 else 0
#                     print(f"  Frames: {frame_count:5d} | "
#                           f"FPS: {fps:5.1f} | "
#                           f"Mapping: {mapping_state}")
                
#             elif err == sl.ERROR_CODE.END_OF_SVOFILE_REACHED:
#                 print("\nEnd of SVO file reached")
#                 break
#             else:
#                 print(f"\nGrab error: {err}")
#                 break
            
#             time.sleep(0.001)  # Small delay
            
#     except KeyboardInterrupt:
#         print("\n\nStopping...")
    
#     # Save the 3D mesh
#     print("\n" + "=" * 50)
#     print("SAVING 3D RECONSTRUCTION")
#     print("=" * 50)
    
#     print("\nExtracting spatial map...")
#     print("This may take a minute...")
    
#     mesh = sl.Mesh()
#     err = zed.extract_whole_spatial_map(mesh)
    
#     if err == sl.ERROR_CODE.SUCCESS:
#         print(f"✓ Map extracted: {len(mesh.vertices)} vertices")
        
#         # Save mesh directly (skip filtering to avoid SDK version issues)
#         print(f"\nSaving mesh to: {mesh_path}")
#         err = mesh.save(mesh_path)
        
#         if err == sl.ERROR_CODE.SUCCESS:
#             print(f"✓ Mesh saved successfully")
            
#             # Print file sizes
#             svo_size = os.path.getsize(svo_path) / (1024*1024)
#             mesh_size = os.path.getsize(mesh_path) / (1024*1024)
            
#             print("\n" + "=" * 50)
#             print("SCAN COMPLETE")
#             print("=" * 50)
#             print(f"Duration: {time.time() - start_time:.1f}s")
#             print(f"Frames: {frame_count}")
#             print(f"\nFiles:")
#             print(f"  SVO:  {svo_path}")
#             print(f"        {svo_size:.1f} MB")
#             print(f"  Mesh: {mesh_path}")
#             print(f"        {mesh_size:.1f} MB")
#             print("=" * 50)
#         else:
#             print(f"ERROR: Failed to save mesh: {err}")
#     else:
#         print(f"ERROR: Failed to extract map: {err}")
    
#     # Cleanup
#     print("\nCleaning up...")
#     zed.disable_recording()
#     zed.disable_spatial_mapping()
#     zed.disable_positional_tracking()
#     zed.close()
    
#     print("✓ Done")

# if __name__ == "__main__":
#     main()





#REMOTE
#!/usr/bin/env python3
"""
zed_fusion_scan.py - ZED Fusion 3D Reconstruction + Recording

Records ZED camera feed to .SVO file and performs real-time 3D reconstruction
using spatial mapping. Saves both the video file and extracted 3D mesh.

Usage:
    python3 zed_fusion_scan.py
    
Output:
    - ~/ZED Navigation/ZED-Camera-Navigation/scans/scan_YYYYMMDD_HHMMSS.svo
    - ~/ZED Navigation/ZED-Camera-Navigation/scans/mesh_YYYYMMDD_HHMMSS.obj
"""

import pyzed.sl as sl
import time
import os
import sys

# Output directory - must match clarq_rf_listener.py
BASE_DIR = os.path.expanduser('~/ZED Navigation/ZED-Camera-Navigation')
SCANS_DIR = os.path.join(BASE_DIR, 'scans')

def main():
    print("=" * 50)
    print("  ZED FUSION 3D SCAN")
    print("=" * 50)
    
    # Create output directory
    os.makedirs(SCANS_DIR, exist_ok=True)
    
    # Initialize ZED camera
    print("\n[1/5] Initializing ZED camera...")
    zed = sl.Camera()
    
    # Camera configuration
    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD720  # 720p for balance
    init_params.depth_mode = sl.DEPTH_MODE.NEURAL        # AI-powered depth (replaces ULTRA)
    init_params.coordinate_units = sl.UNIT.METER         # Meters
    init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP
    
    # Open camera
    err = zed.open(init_params)
    if err != sl.ERROR_CODE.SUCCESS:
        print(f"ERROR: Failed to open ZED camera: {err}")
        sys.exit(1)
    
    print("✓ ZED camera opened")
    
    # Enable positional tracking (required for spatial mapping)
    print("\n[2/5] Enabling positional tracking...")
    tracking_params = sl.PositionalTrackingParameters()
    tracking_params.enable_imu_fusion = True  # Use IMU for better tracking
    
    err = zed.enable_positional_tracking(tracking_params)
    if err != sl.ERROR_CODE.SUCCESS:
        print(f"ERROR: Failed to enable tracking: {err}")
        zed.close()
        sys.exit(1)
    
    print("✓ Positional tracking enabled")
    
    # Enable spatial mapping (3D reconstruction)
    print("\n[3/5] Enabling spatial mapping...")
    mapping_params = sl.SpatialMappingParameters()
    mapping_params.resolution_meter = 0.05        # 5cm voxel resolution
    mapping_params.range_meter = 10.0             # Map up to 10m range
    mapping_params.save_texture = True            # Save RGB texture
    mapping_params.use_chunk_only = False         # Use full map
    mapping_params.map_type = sl.SPATIAL_MAP_TYPE.MESH  # Output as mesh
    
    err = zed.enable_spatial_mapping(mapping_params)
    if err != sl.ERROR_CODE.SUCCESS:
        print(f"ERROR: Failed to enable mapping: {err}")
        zed.close()
        sys.exit(1)
    
    print("✓ Spatial mapping enabled")
    print(f"  Resolution: {mapping_params.resolution_meter}m")
    print(f"  Range: {mapping_params.range_meter}m")
    
    # Setup recording
    print("\n[4/5] Starting recording...")
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    svo_path = os.path.join(SCANS_DIR, f"scan_{timestamp}.svo")
    mesh_path = os.path.join(SCANS_DIR, f"mesh_{timestamp}.obj")
    
    recording_params = sl.RecordingParameters()
    recording_params.compression_mode = sl.SVO_COMPRESSION_MODE.H264
    recording_params.video_filename = svo_path
    
    err = zed.enable_recording(recording_params)
    if err != sl.ERROR_CODE.SUCCESS:
        print(f"ERROR: Failed to start recording: {err}")
        zed.close()
        sys.exit(1)
    
    print(f"✓ Recording to: {svo_path}")
    
    # Runtime parameters
    runtime_params = sl.RuntimeParameters()
    runtime_params.confidence_threshold = 50  # Filter noisy depth
    
    print("\n[5/5] 3D FUSION ACTIVE")
    print("=" * 50)
    print("Recording 3D scan...")
    print("Press Ctrl+C to stop and save")
    print("=" * 50)
    
    frame_count = 0
    start_time = time.time()
    
    try:
        while True:
            # Grab frame
            err = zed.grab(runtime_params)
            
            if err == sl.ERROR_CODE.SUCCESS:
                frame_count += 1
                
                # Update spatial map
                mapping_state = zed.get_spatial_mapping_state()
                
                # Print status every 30 frames (~1 second at 30fps)
                if frame_count % 30 == 0:
                    elapsed = time.time() - start_time
                    fps = frame_count / elapsed if elapsed > 0 else 0
                    print(f"  Frames: {frame_count:5d} | "
                          f"FPS: {fps:5.1f} | "
                          f"Mapping: {mapping_state}")
                
            elif err == sl.ERROR_CODE.END_OF_SVOFILE_REACHED:
                print("\nEnd of SVO file reached")
                break
            else:
                print(f"\nGrab error: {err}")
                break
            
            time.sleep(0.001)  # Small delay
            
    except KeyboardInterrupt:
        print("\n\nStopping...")
    
    # Save the 3D mesh
    print("\n" + "=" * 50)
    print("SAVING 3D RECONSTRUCTION")
    print("=" * 50)
    
    print("\nExtracting spatial map...")
    print("This may take a minute...")
    
    mesh = sl.Mesh()
    err = zed.extract_whole_spatial_map(mesh)
    
    if err == sl.ERROR_CODE.SUCCESS:
        print(f"✓ Map extracted: {len(mesh.vertices)} vertices")
        
        # Save mesh directly (skip filtering to avoid SDK version issues)
        print(f"\nSaving mesh to: {mesh_path}")
        err = mesh.save(mesh_path)
        
        if err == sl.ERROR_CODE.SUCCESS:
            print(f"✓ Mesh saved successfully")
            
            # Print file sizes
            svo_size = os.path.getsize(svo_path) / (1024*1024)
            mesh_size = os.path.getsize(mesh_path) / (1024*1024)
            
            print("\n" + "=" * 50)
            print("SCAN COMPLETE")
            print("=" * 50)
            print(f"Duration: {time.time() - start_time:.1f}s")
            print(f"Frames: {frame_count}")
            print(f"\nFiles:")
            print(f"  SVO:  {svo_path}")
            print(f"        {svo_size:.1f} MB")
            print(f"  Mesh: {mesh_path}")
            print(f"        {mesh_size:.1f} MB")
            print("=" * 50)
        else:
            print(f"ERROR: Failed to save mesh: {err}")
    else:
        print(f"ERROR: Failed to extract map: {err}")
    
    # Cleanup
    print("\nCleaning up...")
    zed.disable_recording()
    zed.disable_spatial_mapping()
    zed.disable_positional_tracking()
    zed.close()
    
    print("✓ Done")

if __name__ == "__main__":
    main()