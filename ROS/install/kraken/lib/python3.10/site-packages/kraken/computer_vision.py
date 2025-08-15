import rclpy
import pyrealsense2 as rs
import numpy as np
import onnxruntime as ort
import cv2
from rclpy.node import Node
import os
from datetime import datetime
import json

import sys
from std_msgs.msg import Int32MultiArray, String

class ComputerVision(Node):

    def __init__(self):
        super().__init__('computer_vision')
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # Shutdown flag
        self.shutdown_requested = False
        
        # Publishers
        self.publisher_ = self.create_publisher(Int32MultiArray, 'objects', 10)
        self.detection_info_publisher = self.create_publisher(String, 'detection_info', 10)
        self.gyro_publisher = self.create_publisher(String, 'gyro_data', 10)
        self.accel_publisher = self.create_publisher(String, 'accel_data', 10)

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
        config.enable_stream(rs.stream.gyro)
        config.enable_stream(rs.stream.accel)
        
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
        self.gyro_report_file = os.path.join(self.error_report_dir, "gyro_accel_report.txt")

        # Create depth colorizer for better depth visualization
        self.colorizer = rs.colorizer()

    def shutdown_gracefully(self):
        """Gracefully shutdown the computer vision node"""
        self.shutdown_requested = True
        try:
            if hasattr(self, 'pipeline'):
                self.pipeline.stop()
        except Exception as e:
            self.get_logger().error(f"Error stopping pipeline during shutdown: {str(e)}")
        cv2.destroyAllWindows()

    def timer_callback(self):
        if self.shutdown_requested:
            return
            
        self.get_logger().info('Computer Vision Running')
        self.boolList = self.objectDetector()
        self.get_logger().info(str(self.boolList))
        
        # Publish the boolean array (existing functionality)
        self.msg = Int32MultiArray()
        self.msg.data = self.boolList
        self.publisher_.publish(self.msg)

    def publish_detection_info(self, detections_info):
        """Publish detailed detection information for FSM"""
        try:
            # Create detection summary for FSM
            detection_summary = {
                'timestamp': str(self.get_clock().now().nanoseconds),
                'total_detections': len(detections_info),
                'detections': []
            }
            
            for detection in detections_info:
                detection_data = {
                    'class_name': detection['class_name'],
                    'class_id': self.CLASS_NAMES.index(detection['class_name']) if detection['class_name'] in self.CLASS_NAMES else -1,
                    'confidence': float(detection['confidence']),
                    'center_x': int(detection['center_x']),
                    'center_y': int(detection['center_y']),
                    'distance': float(detection['distance']),
                    'bbox': {
                        'x1': int(detection['x1']),
                        'y1': int(detection['y1']),
                        'x2': int(detection['x2']),
                        'y2': int(detection['y2'])
                    }
                }
                detection_summary['detections'].append(detection_data)
            
            # Publish detection info as JSON string
            detection_msg = String()
            detection_msg.data = json.dumps(detection_summary)
            self.detection_info_publisher.publish(detection_msg)
            
            self.get_logger().info(f'Published detection info: {len(detections_info)} objects')
            
        except Exception as e:
            self.get_logger().error(f'Error publishing detection info: {str(e)}')

    def publish_gyro_accel_data(self, gyro_data, accel_data):
        """Publish gyroscope and accelerometer data"""
        try:
            gyro_msg = String()
            gyro_msg.data = json.dumps({
                'timestamp': str(self.get_clock().now().nanoseconds),
                'gyro': {'x': gyro_data[0], 'y': gyro_data[1], 'z': gyro_data[2]}
            })
            #publish without timestamp
            gyro_msg.data = json.dumps({
                'gyro': {'x': gyro_data[0], 'y': gyro_data[1], 'z': gyro_data[2]}
            })
            self.gyro_publisher.publish(gyro_msg)

            accel_msg = String()
            accel_msg.data = json.dumps({
                'timestamp': str(self.get_clock().now().nanoseconds),
                'accel': {'x': accel_data[0], 'y': accel_data[1], 'z': accel_data[2]}
            })
            self.accel_publisher.publish(accel_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error publishing IMU data: {str(e)}')

    # ... keep all your existing functions (save_images, write_detection_report, etc.) ...
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

    def write_detection_report(self, detections_info, image_filenames, timestamp):
        """Write detection information to report file"""
        with open(self.report_file, 'a') as f:
            f.write(f"\n--- Detection Report - {timestamp} ---\n")
            f.write(f"Color Image: imageFolder/{image_filenames['color_image']}\n")
            f.write(f"Depth Raw Image: imageFolder/{image_filenames['depth_raw_image']}\n")
            f.write(f"Depth Colorized Image: imageFolder/{image_filenames['depth_colorized_image']}\n")
            f.write("---\n")
            
            for info in detections_info:
                f.write(f"Object: {info['class_name']}\n")
                f.write(f"Confidence: {info['confidence']:.3f}\n")
                f.write(f"Center Position: ({info['center_x']}, {info['center_y']})\n")
                f.write(f"Distance: {info['distance']:.3f} meters\n")
                f.write(f"Bounding Box: ({info['x1']}, {info['y1']}) to ({info['x2']}, {info['y2']})\n")
                f.write("---\n")
            
            f.write(f"Total objects detected: {len(detections_info)}\n")
            f.write("\n")

    def write_gyro_accel_report(self, gyro_data, accel_data):
        """Write gyroscope and accelerometer data to report file"""
        try:
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
            with open(self.gyro_report_file, 'a') as f:
                f.write(f"{timestamp} - Gyro: x={gyro_data[0]:.3f}, y={gyro_data[1]:.3f}, z={gyro_data[2]:.3f} | ")
                f.write(f"Accel: x={accel_data[0]:.3f}, y={accel_data[1]:.3f}, z={accel_data[2]:.3f}\n")
        except Exception as e:
            self.get_logger().error(f'Error writing gyro/accel report: {str(e)}')

    def objectDetector(self):
        try:
            frames = self.pipeline.wait_for_frames(timeout_ms=1000)  # Add 1 second timeout
        except RuntimeError as e:
            self.get_logger().warn(f"Failed to get frames: {str(e)}")
            return [0] * 8
        
        # GET IMU DATA FIRST (ADD THIS BACK)
        gyro_data = [0, 0, 0]  # Default values
        accel_data = [0, 0, 0]  # Default values
        
        # Try to get gyroscope data
        try:
            gyro_frame = frames.first_or_default(rs.stream.gyro)
            if gyro_frame:
                gyro_data_raw = gyro_frame.as_motion_frame().get_motion_data()
                gyro_data = [gyro_data_raw.x, gyro_data_raw.y, gyro_data_raw.z]
        except Exception as e:
            self.get_logger().debug(f'No gyro data available: {str(e)}')

        # Try to get accelerometer data
        try:
            accel_frame = frames.first_or_default(rs.stream.accel)
            if accel_frame:
                accel_data_raw = accel_frame.as_motion_frame().get_motion_data()
                accel_data = [accel_data_raw.x, accel_data_raw.y, accel_data_raw.z]
        except Exception as e:
            self.get_logger().debug(f'No accel data available: {str(e)}')

        # Publish IMU data
        self.publish_gyro_accel_data(gyro_data, accel_data)
        
        # Write IMU data to report
        self.write_gyro_accel_report(gyro_data, accel_data)
        
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
        detections_info = []
        
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
                    
                    # Get class name from class ID
                    class_id = valid_class_ids[i]
                    class_name = self.CLASS_NAMES[class_id] if class_id < len(self.CLASS_NAMES) else f"Class_{class_id}"
                    confidence = valid_scores[i]
                    
                    # Store detection info for reporting
                    detection_info = {
                        'class_name': class_name,
                        'confidence': confidence,
                        'center_x': x_center_pixel,
                        'center_y': y_center_pixel,
                        'distance': depth_value,
                        'x1': x1,
                        'y1': y1,
                        'x2': x2,
                        'y2': y2
                    }
                    detections_info.append(detection_info)
                    
                    # Draw bounding box on the frame
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    
                    # Draw center point
                    cv2.circle(frame, (x_center_pixel, y_center_pixel), 5, (255, 0, 0), -1)
                    
                    # Draw label with class name, confidence, and distance
                    label = f'{class_name}: {confidence:.2f} - {depth_value:.2f}m'
                    cv2.putText(frame, label, (x1, y1-10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                    
                    validObjectBools[class_id] = 1 
                
                # Save images with bounding boxes already drawn
                image_filenames = self.save_images(frame, depth_frame, timestamp)
                
                # Write detection report to file with image references (only when objects detected)
                self.write_detection_report(detections_info, image_filenames, timestamp)
                
                # PUBLISH DETECTION INFO FOR FSM
                self.publish_detection_info(detections_info)

        # Show live feed with bounding boxes and depth info
        # cv2.imshow('RealSense ONNX Detection', frame)
        # cv2.waitKey(1)  # Add this to properly handle OpenCV window events
        
        return validObjectBools


def main(args=None):
    rclpy.init(args=args)

    computer_vision = ComputerVision()

    try:
        rclpy.spin(computer_vision)
    except KeyboardInterrupt:
        computer_vision.get_logger().info("Computer Vision shutting down...")
    finally:
        # Clean up gracefully
        computer_vision.shutdown_gracefully()
        
        # Destroy the node explicitly
        computer_vision.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
