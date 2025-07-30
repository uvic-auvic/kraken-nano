from ultralytics import YOLO
import cv2

model = YOLO('yolov8n.pt')
img = cv2.imread('your_tes_image.jpg')
results = model(img)
results[0].show()

