from fastapi import APIRouter, HTTPException
from fastapi.responses import JSONResponse
from database.connector import DatabaseConnection
import datetime

router = APIRouter(tags=["predict_logs"])

connector = DatabaseConnection()

@router.post("/insertTestLog")
async def insert_test_log():
    date = str(datetime.datetime.now())
    status = "Limpo"
    reboiler_id = 1
    result = connector.query_database("INSERT", "Predict_Logs", data={"Date": date, "Status": status, "reboiler_id": reboiler_id})
    if result == "Dados inserido com sucesso":
        return JSONResponse(content={"message": "Log inserted successfully"}, status_code=201)
    else:
        return JSONResponse(content={"message": "Failed to insert log"}, status_code=500)

@router.get("/getLogs")
async def get_all_logs():
    logs = connector.query_database("SELECT", "Predict_Logs")
    if logs:
        return JSONResponse(content={"logs": logs}, status_code=200)
    else:
        return JSONResponse(content={"message": "No logs found"}, status_code=404)

@router.get("/getLog/{reboiler_id}")
async def get_log(reboiler_id: int):
    log = connector.query_database("SELECT", "Predict_Logs", where={"reboiler_id": reboiler_id})
    if log:
        return JSONResponse(content={"log": log}, status_code=200)
    else:
        return HTTPException(status_code=404, detail="Log not found")
