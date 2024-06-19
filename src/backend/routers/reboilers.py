from fastapi import APIRouter, HTTPException
from fastapi.responses import JSONResponse
from database.connector import DatabaseConnection
from schemas.schemas import Reboiler  

router = APIRouter(tags=["reboilers"])

connector = DatabaseConnection()

@router.post("/createReboiler")
async def create_reboiler(reboiler: Reboiler):
    operation_result = connector.query_database(operation="INSERT", table_name="Reboilers", data=reboiler)
    if operation_result != "Dados inserido com sucesso":
        raise HTTPException(status_code=404, detail="Reboiler not created")
    return JSONResponse(content={"message": "Reboiler created"}, status_code=201)

@router.get("/getReboiler/{reboiler_id}")
async def get_reboiler(reboiler_id: int):
    reboiler = connector.query_database(operation="SELECT", table_name="Reboilers", where={"reboiler_id": reboiler_id})
    if not reboiler:
        raise HTTPException(status_code=404, detail="Reboiler not found")
    return JSONResponse(content={"reboiler": reboiler}, status_code=200)

@router.put("/updateReboiler/{reboiler_id}")
async def update_reboiler(reboiler_id: int, reboiler: Reboiler):
    update_result = connector.query_database(operation="UPDATE", table_name="Reboilers", data=reboiler, where={"reboiler_id": reboiler_id})
    if update_result != "Dados atualizado com sucesso":
        raise HTTPException(status_code=404, detail="Reboiler not updated")
    return JSONResponse(content={"message": "Reboiler updated"}, status_code=200)

@router.delete("/deleteReboiler/{reboiler_id}")
async def delete_reboiler(reboiler_id: int):
    delete_result = connector.query_database(operation="DELETE", table_name="Reboilers", where={"reboiler_id": reboiler_id})
    if delete_result != "Dados deletado com sucesso":
        raise HTTPException(status_code=404, detail="Reboiler not deleted")
    return JSONResponse(content={"message": "Reboiler deleted"}, status_code=200)

@router.get("/getReboilers")
async def get_reboilers():
    reboilers = connector.query_database(operation="SELECT", table_name="Reboilers")
    if not reboilers:
        raise HTTPException(status_code=404, detail="No Reboilers found")
    return JSONResponse(content={"reboilers": reboilers}, status_code=200)
