from ultralytics import YOLO
import cv2

cap = cv2.VideoCapture(0)
model = YOLO("yolov8m.pt")

if not cap.isOpened():
    print("Error: Could not open video source.")
else:
    cap.release()
    results = model.train(data="YOLODataset/dataset.yaml", epochs=2, imgsz=640)
    results = model.predict(source=0, show=True)

