from ultralytics import YOLO
import cv2

cap = cv2.VideoCapture(0)
model = YOLO("/home/lidia/2024-1B-T08-EC06-G02/runs/detect/train6/weights/best.pt")

if not cap.isOpened():
    print("Error: Could not open video source.")
else:
    cap.release()
# results = model.train(data="/home/grupo-02-t08/2024-1B-T08-EC06-G02/src/yolo_v8/YOLODataset/dataset.yaml", epochs=100, imgsz=640)
# val = model.val(results)
# print(val)
results = model.predict(source=0, show=True)

