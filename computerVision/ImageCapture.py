import pyrealsense2 as rs
import numpy as np
import cv2
import os
import time

# Parameters
save_folder = "captured_images"
os.makedirs(save_folder, exist_ok=True)
interval_sec = 1  # Time between captures (seconds)

# Setup pipeline
def print_depth_coverage(depth):
    """
    Prints a text-based visualization of depth coverage within 1 meter
    for a 640x480 RealSense depth frame.

    Args:
        depth: pyrealsense2.depth_frame object
    """
    coverage = [0]*64
    for y in range(480):
        for x in range(640):
            dist = depth.get_distance(x, y)
            if 0 < dist < 1:
                coverage[x//10] += 1

        if y % 20 == 19:
            line = ""
            for c in coverage:
                line += " .:nhBXWW"[c//25]
            coverage = [0]*64
            print(line)

try:
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)

    pipeline.start(config)
    # Create a colorizer for depth frames with histogram equalization to match RealSense viewer
    colorizer = rs.colorizer()
    colorizer.set_option(rs.option.histogram_equalization_enabled, True)

    img_index = 0
    while True:
        # Wait for a coherent frame
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        depth = frames.get_depth_frame()

        if not depth:
            print("print no depth data")
            continue

      # Convert color and depth frames to numpy arrays
        color_image = np.asanyarray(color_frame.get_data())
        # Use RealSense colorizer to generate a color-mapped depth image with histogram equalization
        depth_color_frame = colorizer.colorize(depth)
        depth_colormap = np.asanyarray(depth_color_frame.get_data())

        # Save images
        filename = os.path.join(save_folder, f"image_{img_index:05d}.jpg")
        depthFilename = os.path.join(save_folder, f"image_depth{img_index:05d}.jpg")
        cv2.imwrite(filename, color_image)
        cv2.imwrite(depthFilename, depth_colormap)
        print(f"Saved {filename}")
        print(f"Saved {depthFilename}")

        img_index += 1
        time.sleep(interval_sec)
    exit(0)

except KeyboardInterrupt:
    print("Interrupted by user, stopping capture.")