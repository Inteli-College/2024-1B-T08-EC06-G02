from ultralytics import YOLO
import cv2


cap = cv2.VideoCapture(2)

#carregamos o modelo já pré- treinado
model = YOLO("runs/detect/train6/weights/best.pt")

if not cap.isOpened():
    print("Error: Could not open video source.")
else:
    cap.release()
    results = model.predict(source=2, show=True)

# # Carregamos o modelo já pré-treinado
# model = YOLO("runs/detect/train6/weights/best.pt")

# image_path = ""

# image = cv2.imread(image_path)

# if image is None:
#     print("Error: Could not load image.")
# else:
#     results = model.predict(source=image, show=True)

