import rclpy
import pyrealsense2 as rs
import numpy as np
import onnxruntime as ort
import cv2
from rclpy.node import Node
import os
import json
from datetime import datetime

import sys
from std_msgs.msg import Int32MultiArray, String 

class ComputerVision(Node):

    def __init__(self):
        super().__init__('computer_vision')
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.publisher_ = self.create_publisher(Int32MultiArray, 'objects', 10)
        
        # Add publisher for object positioning data
        self.position_publisher_ = self.create_publisher(String, 'object_positions', 10)
        
        # Add publishers for gyro and accel data
        self.gyro_publisher_ = self.create_publisher(String, 'gyro_data', 10)
        self.accel_publisher_ = self.create_publisher(String, 'accel_data', 10)

        # --- Class Names from your data.yaml ---
        self.CLASS_NAMES = ['Sawfish Gate Banner', 'Shark Gate Banner', 'Full Gate', 'Red Slalom', 'White Slalom', 'Full Torpedo Banner', 'Sawfish Torpedo Hole', 'Shark Torpedo Hole']

        # --- ONNX Model Setup ---
        ONNX_MODEL_PATH = "/home/kraken/kraken-nano/computerVision/new_arvp_front.onnx"
        self.session = ort.InferenceSession(ONNX_MODEL_PATH, providers=['CUDAExecutionProvider'])
        self.input_name = self.session.get_inputs()[0].name

        # --- RealSense Camera Setup with Depth ---
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        
        # this is the shit for real sense that gets the acceleration data and the gyro, it will publish so our state estimator can get the info and do some calc with it. calc is short for calculus and calculator btw
        config.enable_stream(rs.stream.accel)
        config.enable_stream(rs.stream.gyro)

        # Start pipeline and get profile for intrinsics
        profile = self.pipeline.start(config)
        
        # Get depth scale for converting depth values to meters
        depth_sensor = profile.get_device().first_depth_sensor()
        self.depth_scale = depth_sensor.get_depth_scale()
        
        # Create alignment object to align depth frame to color frame
        align_to = rs.stream.color
        self.align = rs.align(align_to)
        
        # Create error reporting directory and image folder
        self.error_report_dir = "/home/kraken/Desktop/error_reporting"
        self.image_folder = os.path.join(self.error_report_dir, "imageFolder")
        os.makedirs(self.error_report_dir, exist_ok=True)
        os.makedirs(self.image_folder, exist_ok=True)
        self.report_file = os.path.join(self.error_report_dir, "detection_distances.txt")

        # Create depth colorizer for better depth visualization
        self.colorizer = rs.colorizer()

    def get_object_quadrant_and_position(self, x_center, y_center, frame_width, frame_height):
        """
        Determine object quadrant and relative position in frame
        Returns quadrant (1-4) and relative position (-1 to 1 for both x,y)
        """
        # Calculate frame center
        center_x = frame_width // 2
        center_y = frame_height // 2
        
        # Calculate relative position (-1 to 1)
        relative_x = (x_center - center_x) / (frame_width / 2)
        relative_y = (y_center - center_y) / (frame_height / 2)
        
        # Determine quadrant (1=top-right, 2=top-left, 3=bottom-left, 4=bottom-right)
        if x_center >= center_x and y_center <= center_y:
            quadrant = 1  # Top-right
        elif x_center < center_x and y_center <= center_y:
            quadrant = 2  # Top-left
        elif x_center < center_x and y_center > center_y:
            quadrant = 3  # Bottom-left
        else:
            quadrant = 4  # Bottom-right
            
        return quadrant, relative_x, relative_y

    def timer_callback(self):
        self.get_logger().info('Computer Vision Running')
        self.boolList = self.objectDetector()
        self.get_logger().info(str(self.boolList))
        self.msg = Int32MultiArray()
        self.msg.data = self.boolList
        self.publisher_.publish(self.msg)

    def save_images(self, color_frame_with_boxes, depth_frame, timestamp):
        """Save color image with bounding boxes and depth images to imageFolder"""
        timestamp_str = timestamp.replace(" ", "_").replace(":", "-").replace(".", "-")
        
        # Save color image with bounding boxes already drawn
        color_filename = f"color_{timestamp_str}.jpg"
        color_filepath = os.path.join(self.image_folder, color_filename)
        cv2.imwrite(color_filepath, color_frame_with_boxes)
        
        # Save depth image (raw depth data)
        depth_image = np.asanyarray(depth_frame.get_data())
        depth_raw_filename = f"depth_raw_{timestamp_str}.png"
        depth_raw_filepath = os.path.join(self.image_folder, depth_raw_filename)
        cv2.imwrite(depth_raw_filepath, depth_image)
        
        # Save colorized depth image for better visualization
        depth_colorized_frame = self.colorizer.colorize(depth_frame)
        depth_colorized_image = np.asanyarray(depth_colorized_frame.get_data())
        depth_colorized_filename = f"depth_colorized_{timestamp_str}.jpg"
        depth_colorized_filepath = os.path.join(self.image_folder, depth_colorized_filename)
        cv2.imwrite(depth_colorized_filepath, depth_colorized_image)
        
        return {
            'color_image': color_filename,
            'depth_raw_image': depth_raw_filename,
            'depth_colorized_image': depth_colorized_filename
        }

    def write_positioning_report(self, positioning_data, image_filenames, timestamp):
        """Write positioning data being sent to planner to report file"""
        
        with open(self.report_file, 'a') as f:
            f.write(f"\n--- PLANNER POSITIONING DATA - {timestamp} ---\n")
            f.write(f"Color Image: imageFolder/{image_filenames['color_image']}\n")
            f.write(f"Depth Raw Image: imageFolder/{image_filenames['depth_raw_image']}\n")
            f.write(f"Depth Colorized Image: imageFolder/{image_filenames['depth_colorized_image']}\n")
            f.write("---\n")
            
            for class_name, data in positioning_data.items():
                f.write(f"Object: {class_name}\n")
                f.write(f"Class ID: {data['class_id']}\n")
                f.write(f"Quadrant: {data['quadrant']}\n")
                f.write(f"Relative Position: X={data['relative_x']:.3f}, Y={data['relative_y']:.3f}\n")
                f.write(f"Pixel Center: ({data['center_x']}, {data['center_y']})\n")
                f.write(f"Distance: {data['distance']:.3f} meters\n")
                f.write(f"Confidence: {data['confidence']:.3f}\n")
                f.write(f"Size: {data['width']}x{data['height']} pixels\n")
                f.write("---\n")
            
            f.write(f"Total objects sent to planner: {len(positioning_data)}\n")
            f.write(f"JSON Data: {json.dumps(positioning_data, indent=2)}\n")
            f.write("\n")


    def write_gyro_accel_report(self, gyro, accel, timestamp):
        """Write gyro and accel data being sent to state estimator to report file"""

        with open(self.report_file, 'a') as f:
            f.write(f"\n--- GYRO ACCEL DATA - {timestamp} ---\n")
            f.write(f"GYRO INFO: X={gyro.x:.6f}, Y={gyro.y:.6f}, Z={gyro.z:.6f} rad/s\n")
            f.write(f"ACCEL INFO: X={accel.x:.6f}, Y={accel.y:.6f}, Z={accel.z:.6f} m/s²\n")
            f.write("---\n")

    def objectDetector(self):
        frames = self.pipeline.wait_for_frames()
        
        # Get gyro and accel data from RealSense
        gyro_frame = None
        accel_frame = None
        
        for f in frames:
            if f.profile.stream_type() == rs.stream.gyro:
                gyro_frame = f.as_motion_frame()
            elif f.profile.stream_type() == rs.stream.accel:
                accel_frame = f.as_motion_frame()
        
        # Generate timestamp for IMU data
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        
        # Process and publish gyro/accel data if available
        if gyro_frame and accel_frame:
            gyro_data = gyro_frame.get_motion_data()
            accel_data = accel_frame.get_motion_data()
            
            # Create data structures for publishing
            gyro_json = {
                'timestamp': timestamp,
                'x': float(gyro_data.x),
                'y': float(gyro_data.y),
                'z': float(gyro_data.z),
                'units': 'rad/s'
            }
            
            accel_json = {
                'timestamp': timestamp,
                'x': float(accel_data.x),
                'y': float(accel_data.y),
                'z': float(accel_data.z),
                'units': 'm/s²'
            }
            
            # Publish gyro and accel data
            gyro_msg = String()
            gyro_msg.data = json.dumps(gyro_json)
            self.gyro_publisher_.publish(gyro_msg)
            
            accel_msg = String()
            accel_msg.data = json.dumps(accel_json)
            self.accel_publisher_.publish(accel_msg)
            
            # Write gyro/accel report continuously
            self.write_gyro_accel_report(gyro_data, accel_data, timestamp)
            
            print(f"IMU DATA - Gyro: X={gyro_data.x:.3f}, Y={gyro_data.y:.3f}, Z={gyro_data.z:.3f} rad/s | Accel: X={accel_data.x:.3f}, Y={accel_data.y:.3f}, Z={accel_data.z:.3f} m/s²")
        
        # Align depth frame to color frame
        aligned_frames = self.align.process(frames)
        
        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()
        
        if not color_frame or not depth_frame:
           return [0] * 8

        frame = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())
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
        positioning_data = {}  # Store positioning data for planner use
        
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

                # Generate timestamp only when detections are found
                timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]

                for i, box in enumerate(valid_boxes):
                    # Convert from YOLO format (center, width, height) to corner coordinates
                    x_center, y_center, width, height = box
                    
                    # Convert to pixel coordinates (accounting for resize)
                    x_center_pixel = int(x_center * original_width / 640)
                    y_center_pixel = int(y_center * original_height / 640)
                    width_pixel = int(width * original_width / 640)
                    height_pixel = int(height * original_height / 640)
                    
                    # Calculate corner coordinates
                    x1 = int(x_center_pixel - width_pixel / 2)
                    y1 = int(y_center_pixel - height_pixel / 2)
                    x2 = int(x_center_pixel + width_pixel / 2)
                    y2 = int(y_center_pixel + height_pixel / 2)
                    
                    # Ensure coordinates are within frame bounds
                    x_center_pixel = max(0, min(x_center_pixel, original_width - 1))
                    y_center_pixel = max(0, min(y_center_pixel, original_height - 1))
                    
                    # Get depth at center of detection
                    depth_value = depth_frame.get_distance(x_center_pixel, y_center_pixel)
                    
                    # Get quadrant and relative position
                    quadrant, relative_x, relative_y = self.get_object_quadrant_and_position(
                        x_center_pixel, y_center_pixel, original_width, original_height)
                    
                    # Get class name from class ID
                    class_id = valid_class_ids[i]
                    class_name = self.CLASS_NAMES[class_id] if class_id < len(self.CLASS_NAMES) else f"Class_{class_id}"
                    confidence = valid_scores[i]
                    

                    
                    # Store positioning data by class name for planner use
                    positioning_data[class_name] = {
                        'class_id': int(class_id),
                        'center_x': int(x_center_pixel),
                        'center_y': int(y_center_pixel),
                        'relative_x': float(relative_x),
                        'relative_y': float(relative_y),
                        'quadrant': int(quadrant),
                        'distance': float(depth_value),
                        'confidence': float(confidence),
                        'width': int(width_pixel),
                        'height': int(height_pixel)
                    }
                    
                    # Draw bounding box on the frame
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    
                    # Draw center point
                    cv2.circle(frame, (x_center_pixel, y_center_pixel), 5, (255, 0, 0), -1)
                    
                    # Draw label with class name, confidence, distance, and quadrant
                    label = f'{class_name}: {confidence:.2f} - {depth_value:.2f}m - Q{quadrant}'
                    cv2.putText(frame, label, (x1, y1-10), 
                              cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                    
                    # Draw relative position info
                    pos_label = f'Rel: ({relative_x:.2f}, {relative_y:.2f})'
                    cv2.putText(frame, pos_label, (x1, y1-25), 
                              cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)
                    
                    # Print positioning data being sent to planner
                    print(f"PLANNER DATA - {class_name}: Q{quadrant}, Rel_X:{relative_x:.3f}, Rel_Y:{relative_y:.3f}, Dist:{depth_value:.3f}m, Conf:{confidence:.3f}")
                    validObjectBools[class_id] = 1 
                
                print(f"Total objects detected: {len(valid_boxes)}")
                print(f"POSITIONING DATA TO PLANNER: {json.dumps(positioning_data, indent=2)}")
                
                # Publish positioning data for planner use
                if positioning_data:
                    position_msg = String()
                    position_msg.data = json.dumps(positioning_data)
                    self.position_publisher_.publish(position_msg)
                
                # Save images and write positioning data to report file
                image_filenames = self.save_images(frame, depth_frame, timestamp)
                self.write_positioning_report(positioning_data, image_filenames, timestamp)
                
            # No else clause for "Nil" - we just don't report anything when no detections
        else:
            print("ONNX output shape:", outputs[0].shape)

        # Show live feed with bounding boxes and depth info
        
        # Draw frame reference lines (quadrant dividers)
        center_x = original_width // 2
        center_y = original_height // 2
        
        # Draw center crosshair
        cv2.line(frame, (center_x, 0), (center_x, original_height), (255, 255, 255), 1)  # Vertical line
        cv2.line(frame, (0, center_y), (original_width, center_y), (255, 255, 255), 1)   # Horizontal line
        
        # Label quadrants
        cv2.putText(frame, 'Q1', (center_x + 10, center_y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(frame, 'Q2', (10, center_y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(frame, 'Q3', (10, center_y + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(frame, 'Q4', (center_x + 10, center_y + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        cv2.imshow('RealSense ONNX Detection', frame)
        cv2.waitKey(1)  # Add this to properly handle OpenCV window events
        
        return validObjectBools


def main(args=None):
    rclpy.init(args=args)

    computer_vision = ComputerVision()

    try:
        rclpy.spin(computer_vision)
    finally:
        # Clean up
        computer_vision.pipeline.stop()
        cv2.destroyAllWindows()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    computer_vision.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
