import pyrealsense2 as rs
import numpy as np
import cv2

# Configure streams
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
pipeline.start(config)
colorizer = rs.colorizer()

try:
    while True:
        # Wait for a coherent pair of frames
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue

        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # Apply colormap to depth image for visualization
        depth_color_frame = colorizer.colorize(depth_frame)
        depth_colormap = np.asanyarray(depth_color_frame.get_data())

        # Stack both images horizontally
        images = np.hstack((color_image, depth_colormap))

        # Display images
        cv2.imshow('RealSense (Color | Depth)', images)
        key = cv2.waitKey(1)
        if key == 27:  # ESC to exit
            break
finally:
    pipeline.stop()
    cv2.destroyAllWindows()
