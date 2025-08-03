import pyrealsense2 as rs
import numpy as np
import cv2
import onnxruntime as ort

# --- Class Names from your data.yaml ---
CLASS_NAMES = ['bottom', 'center', 'gate', 'left', 'red_flare', 'right', 'top', 'tubs']

# --- ONNX Model Setup ---
ONNX_MODEL_PATH = "best.onnx"
session = ort.InferenceSession(ONNX_MODEL_PATH, providers=['CUDAExecutionProvider'])
input_name = session.get_inputs()[0].name

# --- RealSense Camera Setup ---
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

        frame = np.asanyarray(color_frame.get_data())
        original_height, original_width = frame.shape[:2]

        # --- Preprocessing ---
        img = cv2.resize(frame, (640, 640))
        img = img.astype(np.float32) / 255.0
        img = np.transpose(img, (2, 0, 1))  # HWC -> CHW
        img = np.expand_dims(img, axis=0)   # Add batch dimension

        # --- Run ONNX Inference ---
        outputs = session.run(None, {input_name: img})
        
        # --- YOLO Postprocessing ---
        if len(outputs[0].shape) == 3:  # YOLOv8 format [1, 84, 8400]
            detections = outputs[0][0].T  # [8400, 84]
            
            # Extract boxes, scores, and class predictions
            boxes = detections[:, :4]  # x_center, y_center, width, height
            scores = detections[:, 4:].max(axis=1)
            class_ids = detections[:, 4:].argmax(axis=1)
            
            # Filter by confidence threshold
            conf_threshold = 0.5
            valid_detections = scores > conf_threshold
            
            if valid_detections.any():
                valid_boxes = boxes[valid_detections]
                valid_scores = scores[valid_detections]
                valid_class_ids = class_ids[valid_detections]
                
                for i, box in enumerate(valid_boxes):
                    # Convert from YOLO format (center, width, height) to corner coordinates
                    x_center, y_center, width, height = box
                    
                    # Convert to pixel coordinates (accounting for resize)
                    x_center = int(x_center * original_width / 640)
                    y_center = int(y_center * original_height / 640)
                    width = int(width * original_width / 640)
                    height = int(height * original_height / 640)
                    
                    # Calculate corner coordinates
                    x1 = int(x_center - width / 2)
                    y1 = int(y_center - height / 2)
                    x2 = int(x_center + width / 2)
                    y2 = int(y_center + height / 2)
                    
                    # Get class name from class ID
                    class_id = valid_class_ids[i]
                    class_name = CLASS_NAMES[class_id] if class_id < len(CLASS_NAMES) else f"Class_{class_id}"
                    confidence = valid_scores[i]
                    
                    # Draw bounding box
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    
                    # Draw label with class name and confidence
                    label = f'{class_name}: {confidence:.2f}'
                    cv2.putText(frame, label, (x1, y1-10), 
                              cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                    
                    # Print detection info to console
                    print(f"Detected: {class_name} with confidence {confidence:.2f} at ({x1},{y1}) to ({x2},{y2})")
                
                print(f"Total objects detected: {len(valid_boxes)}")
            else:
                print("No objects detected")
        else:
            print("ONNX output shape:", outputs[0].shape)

        # Show live feed with bounding boxes
        cv2.imshow('RealSense ONNX Detection', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
finally:
    pipeline.stop()
    cv2.destroyAllWindows( )
