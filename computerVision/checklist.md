# Kraken Nano Computer Vision TODO Checklist

## 🔄 Model Conversion
- [ ] Install required dependencies (`pip install ultralytics torch torchvision onnx`)
- [ ] Run ONNX conversion script on `new_arvp_front.pt`
- [ ] Verify ONNX model file is created successfully
- [ ] Test ONNX model loads correctly with onnxruntime
- [ ] Update computer_vision.py to use new ONNX model path
- [ ] Compare detection accuracy between PyTorch and ONNX versions

## 📹 Bottom Camera Setup
- [ ] Configure RealSense pipeline for bottom camera stream
- [ ] Test bottom camera video feed
- [ ] Verify camera calibration/orientation
- [ ] Update camera stream parameters in computer_vision.py
- [ ] Test object detection on bottom camera feed
- [ ] Implement camera switching logic (if needed)

## 📐 Object Positioning/Distance Reporting
- [ ] Implement depth frame integration from RealSense
- [ ] Add distance calculation for detected objects
- [ ] Implement object centering detection logic
- [ ] Add X,Y position relative to frame center
- [ ] Create distance/position data structure for ROS messages
- [ ] Test positioning accuracy with known objects
- [ ] Add position data to published ROS messages

## 🤖 Planner Node Migration
- [ ] Create new ROS2 planner node structure
- [ ] Convert Python planner logic to ROS2 format
- [ ] Update message types and topic names for ROS2
- [ ] Implement ROS2 publishers and subscribers
- [ ] Add proper ROS2 lifecycle management
- [ ] Test planner node communication with computer_vision
- [ ] Verify mission planning logic works in ROS2
- [ ] Update launch files for new ROS2 planner

## 🧪 Integration Testing
- [ ] Test complete pipeline: camera → detection → positioning → planning
- [ ] Verify ROS2 message flow between nodes
- [ ] Test system performance and timing
- [ ] Validate detection accuracy in real-world scenarios
- [ ] Performance optimization if needed

## 📝 Documentation
- [ ] Update README with new setup instructions
- [ ] Document ONNX conversion process
- [ ] Document camera setup procedures
- [ ] Update ROS2
