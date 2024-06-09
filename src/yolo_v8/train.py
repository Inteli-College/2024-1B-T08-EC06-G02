from ultralytics import YOLO
import cv2

cap = cv2.VideoCapture(2)

#carregamos o modelo já pré- treinado
model = YOLO("yolov8m.pt")

if not cap.isOpened():
    print("Error: Could not open video source.")
else:
    cap.release()
    # treinamento do modelo
    results = model.train(data="YOLODataset/dataset.yaml", epochs=100, imgsz=640)
    val = model.val(results)