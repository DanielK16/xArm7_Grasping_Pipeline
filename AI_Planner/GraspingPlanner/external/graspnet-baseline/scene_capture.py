import pyrealsense2 as rs
import numpy as np
import cv2
import os
import time

# Create a pipeline
pipeline = rs.pipeline()

# Configure the pipeline to stream color and depth images
config = rs.config()
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)

# Start streaming
profile = pipeline.start(config)

# Get depth and color intrinsics [if aligned to color frame, use color intrinsics]
depth_sensor = profile.get_stream(rs.stream.depth)  # Depth stream
color_sensor = profile.get_stream(rs.stream.color)  # Color stream

depth_intrinsics = depth_sensor.as_video_stream_profile().get_intrinsics()
color_intrinsics = color_sensor.as_video_stream_profile().get_intrinsics()

#print("Depth Camera Intrinsics:")
#print(f"  Width: {depth_intrinsics.width}, Height: {depth_intrinsics.height}")
#print(f"  fx: {depth_intrinsics.fx}, fy: {depth_intrinsics.fy}")
#print(f"  cx: {depth_intrinsics.ppx}, cy: {depth_intrinsics.ppy}")
#print(f"  Distortion Model: {depth_intrinsics.model}")

#print("\nColor Camera Intrinsics:")
#print(f"  Width: {color_intrinsics.width}, Height: {color_intrinsics.height}")
#print(f"  fx: {color_intrinsics.fx}, fy: {color_intrinsics.fy}")
#print(f"  cx: {color_intrinsics.ppx}, cy: {color_intrinsics.ppy}")
#print(f"  Distortion Model: {color_intrinsics.model}")

# Wait 2 seconds for the camera to stabilize
#print("\nWaiting for 2 seconds before capturing frames...")
time.sleep(2.25)

rgb_dir = "./scene_images"
depth_dir = "./scene_images"
os.makedirs(rgb_dir, exist_ok=True)
os.makedirs(depth_dir, exist_ok=True)

# Create an alignment object to align depth to color
align = rs.align(rs.stream.color)

#Filters:
# spatial=rs.spatial_filter()
# spatial.set_option(rs.option.filter_magnitude, 2)
# spatial.set_option(rs.option.filter_smooth_alpha, 1)
# spatial.set_option(rs.option.filter_smooth_delta, 20)
#hole_filling=rs.hole_filling_filter()


try:

    frames = pipeline.wait_for_frames()
    aligned_frames = align.process(frames)  # Align depth to color

    color_frame = aligned_frames.get_color_frame()
    depth_frame = aligned_frames.get_depth_frame()

    #depth_frame=spatial.process(depth_frame)    
    #depth_frame=hole_filling.process(depth_frame)

    if not color_frame or not depth_frame:
        print(f"Frame skipped due to capture failure.")

        # Convert images to numpy arrays
    color_image = np.asanyarray(color_frame.get_data())
    depth_image = np.asanyarray(depth_frame.get_data())

        # Save color image
    color_filename = os.path.join(rgb_dir, f"color.png")
    cv2.imwrite(color_filename, color_image)

        # Save depth image
    depth_filename = os.path.join(depth_dir, f"depth.png")
    cv2.imwrite(depth_filename, depth_image)

    print(f"Saved: {color_filename} and {depth_filename}")

finally:
    pipeline.stop()
    print("Frame Captured")

