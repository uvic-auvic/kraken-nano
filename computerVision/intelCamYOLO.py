from ultralytics import YOLO
import pyrealsense2 as rs
import numpy as np
import cv2

model = YOLO('yolov8n.pt')
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipeline.start(config)

try:
    while True:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue

        img = np.asanyarray(color_frame.get_data())
        results = model(img)
        
        depth_frame = frames.get_depth_frame()
        if not depth_frame:
            continue

        for box in results[0].boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            cx, cy = (x1 + x2) //2, (y1 + y2) //2
            depth = depth_frame.get_distance(cx, cy)
            print(f"object at({cx}, {cy}) with {depth} meters")

        annotated_img = results[0].plot()
        cv2.imshow('YOLOv8 Detection', annotated_img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
finally:
    pipeline.stop()
    cv2.destroyAllWindows()
