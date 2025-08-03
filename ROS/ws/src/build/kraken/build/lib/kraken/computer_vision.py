import rclpy
import pyrealsense2 as rs
import numpy as np
import onnxruntime as ort
import cv2
from rclpy.node import Node

import sys
from std_msgs.msg import Int32MultiArray 

class ComputerVision(Node):

    def __init__(self):
        super().__init__('computer_vision')
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.publisher_ = self.create_publisher(Int32MultiArray, 'objects', 10)

        # --- Class Names from your data.yaml ---
        self.CLASS_NAMES = ['bottom', 'center', 'gate', 'left', 'red_flare', 'right', 'top', 'tubs']

        # --- ONNX Model Setup ---
        ONNX_MODEL_PATH = "/home/kraken/kraken-nano/computerVision/best.onnx"
        self.session = ort.InferenceSession(ONNX_MODEL_PATH, providers=['CUDAExecutionProvider'])
        self.input_name = self.session.get_inputs()[0].name

        # --- RealSense Camera Setup ---
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.pipeline.start(config)


    def timer_callback(self):
        self.get_logger().info('Computer Vision Running')
        self.boolList = self.objectDetector()
        self.get_logger().info(str(self.boolList))
        self.msg = Int32MultiArray()
        self.msg.data = self.boolList
        self.publisher_.publish(self.msg)

    def objectDetector(self):
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
           return 

        frame = np.asanyarray(color_frame.get_data())
        original_height, original_width = frame.shape[:2]

        # --- Preprocessing ---
        img = cv2.resize(frame, (640, 640))
        img = img.astype(np.float32) / 255.0
        img = np.transpose(img, (2, 0, 1))  # HWC -> CHW
        img = np.expand_dims(img, axis=0)   # Add batch dimension

        # --- Run ONNX Inference ---
        outputs = self.session.run(None, {self.input_name: img})
        
        # Initialize default return value
        validObjectBools = [0] * 8
        
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
                    class_name = self.CLASS_NAMES[class_id] if class_id < len(self.CLASS_NAMES) else f"Class_{class_id}"
                    confidence = valid_scores[i]
                    
                    # Draw bounding box
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    
                    # Draw label with class name and confidence
                    label = f'{class_name}: {confidence:.2f}'
                    cv2.putText(frame, label, (x1, y1-10), 
                              cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                    
                    # Print detection info to console
                    print(f"Detected: {class_name} with confidence {confidence:.2f} at ({x1},{y1}) to ({x2},{y2})")
                    validObjectBools[class_id] = 1 
                
                print(f"Total objects detected: {len(valid_boxes)}")
            else:
                print("No objects detected")
        else:
            print("ONNX output shape:", outputs[0].shape)

        # Show live feed with bounding boxes - MOVED OUTSIDE CONDITIONAL BLOCKS
        cv2.imshow('RealSense ONNX Detection', frame)
        cv2.waitKey(1)  # Add this to properly handle OpenCV window events
        
        return validObjectBools


def main(args=None):
    rclpy.init(args=args)

    computer_vision = ComputerVision()

    rclpy.spin(computer_vision)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    computer_vision.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
