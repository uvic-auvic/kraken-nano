# Underwater Camera Calibration Checklist (Intel RealSense D455 + MATLAB)

## Preparation
- [X] Install MATLAB (with Computer Vision Toolbox) on your MacBook Air
- [X] Connect Intel RealSense D455 camera via USB
- [ ] Ensure camera drivers/software (e.g., `librealsense` for Mac) are installed
- [ ] Print a checkerboard calibration target (e.g., 9x6 squares, 25mm x 25mm each)

## Image Capture
- [ ] Set up the checkerboard in your underwater environment/tank
- [ ] Ensure good, even lighting for clear visibility of the checkerboard
- [ ] Capture 15–20 images of the checkerboard underwater at various positions, angles, and distances
- [ ] Save all calibration images in a dedicated folder on your computer

## MATLAB Camera Calibrator Workflow
- [ ] Open MATLAB
- [ ] Launch Camera Calibrator app (`cameraCalibrator` command or via Apps tab)
- [ ] Add calibration images to the app
- [ ] Specify checkerboard square size and number of squares
- [ ] Confirm the checkerboard is detected in all images

## Calibration & Validation
- [ ] Click **Calibrate** to compute camera parameters
- [ ] Check reprojection errors (they should be low)
- [ ] Export/save calibration results
- [ ] Use the **Undistort Image** feature to validate distortion correction
- [ ] Optionally, test undistortion on new underwater images

## Usage
- [ ] Apply exported camera parameters in your MATLAB code for underwater vision tasks
- [ ] (Optional) Explore advanced calibration models/settings for underwater refraction effects
