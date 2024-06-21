from fastapi import APIRouter, HTTPException
from fastapi.responses import JSONResponse
from database.connector import DatabaseConnection
from schemas.schemas import Predict
import base64
from PIL import Image
import io
import torch
from ultralytics import YOLO
import datetime
import os

router = APIRouter(tags=["predict"])
connector = DatabaseConnection()

@router.post("/predict")
async def predict(predict: Predict):
    print("Current working directory:", os.getcwd())
    try:
        # Check if the base64 string includes the prefix and remove it
        if predict.image.startswith('data:image/jpeg;base64,'):
            base64_string = predict.image.split('data:image/jpeg;base64,')[1]
        else:
            base64_string = predict.image

        # Decode the base64 string to an image
        decoded_image = base64.b64decode(base64_string)
        image = Image.open(io.BytesIO(decoded_image))

        # Predict using the YOLO model
        result = detect_objects(image)

        # Log the prediction result using the value of 'result' directly
        log_prediction(str(result))

        return JSONResponse(content={"result": result})
    except Exception as e:
        raise HTTPException(status_code=400, detail=str(e))
    

def detect_objects(image):
    model = YOLO('./utils/best.pt')
    results = model.predict(source=image, show=False)

    print("Results structure:", results)

    # Itera sobre os resultados e verifica se há detecções 
    for result in results:
        print("Type of result element:", type(result))
        print("Attributes of result element:", dir(result))

        # Checa se o resultado possui a chave 'boxes' e se há detecções
        if hasattr(result, 'boxes'):
            boxes = result.boxes
            num_detections = len(boxes) if boxes else 0
            print(f"Number of detections: {num_detections}")
            
            # Printa os detalhes de cada detecção - comentado para evitar poluição no console
            # for box in boxes:
            #     print("Detection details:", box)
            
            return num_detections > 0
    
    # If no valid detections found
    return False

def log_prediction(status):
    date = str(datetime.datetime.now())  # Capture current date and time
    query = "INSERT INTO PredictLogs (Date, Status, reboiler_id) VALUES (?, ?, ?)"
    params = (date, status, 1)  # Always using 1 for reboiler_id
    # Execute the query to log the prediction
    connector.query_database("INSERT", "Predict_Logs", data={"Date": date, "Status": status, "reboiler_id": 1})
