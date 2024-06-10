from ultralytics import YOLO

#carregamos o modelo já pré-treinado
model = YOLO("yolov8m.pt")

# treinamento do modelo
results = model.train(data="YOLODataset/dataset.yaml", epochs=100, imgsz=640)
val = model.val(results)