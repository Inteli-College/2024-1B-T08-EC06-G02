from ultralytics import YOLO
import cv2

cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Error: Could not open video source.")
else:
    model = YOLO("/home/lidia/2024-1B-T08-EC06-G02/runs/detect/train6/weights/best.pt")
    cap.release()

    train_results = model.train(data="/home/grupo-02-t08/2024-1B-T08-EC06-G02/src/yolo_v8/YOLODataset/dataset.yaml", epochs=100, imgsz=640)
    
    val_results = model.val()

    model.tune(data="/home/lidia/2024-1B-T08-EC06-G02/src/yolo_v8/YOLODataset/dataset.yaml", epochs=30, iterations=300, optimizer="AdamW", plots=False, save=False, val=False)
    
    results = model.predict(source=0, show=True)


