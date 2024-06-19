from fastapi import APIRouter, HTTPException, Body
from fastapi.responses import JSONResponse
from database.database import DatabaseConnector
from schemas.schemas import Reboiler


router = APIRouter(tags=["reboilers"])

connector = DatabaseConnector('repipe.db')

@router.post("/createReboiler")
async def create_reboiler(reboiler: Reboiler):
    reboiler_id = connector.insert_reboiler(reboiler)
    if reboiler_id is None:
        raise HTTPException(status_code=404, detail="Reboiler not created")
    return JSONResponse(content={"message": "Reboiler created", "reboiler_id": reboiler_id}, status_code=201)

@router.get("/getReboiler/{reboiler_id}")
async def get_reboiler(reboiler_id: int):
    reboiler = connector.get_reboiler(reboiler_id)
    if reboiler is None:
        raise HTTPException(status_code=404, detail="Reboiler not found")
    return JSONResponse(content={"reboiler": reboiler}, status_code=200)

@router.put("/updateReboiler/{reboiler_id}")
async def update_reboiler(reboiler_id: int, reboiler: Reboiler):
    update_count = connector.update_reboiler(reboiler_id, reboiler)
    if update_count == 0:
        raise HTTPException(status_code=404, detail="Reboiler not updated")
    return JSONResponse(content={"message": "Reboiler updated"}, status_code=200)

@router.delete("/deleteReboiler/{reboiler_id}")
async def delete_reboiler(reboiler_id: int):
    delete_count = connector.delete_reboiler(reboiler_id)
    if delete_count == 0:
        raise HTTPException(status_code=404, detail="Reboiler not deleted")
    return JSONResponse(content={"message": "Reboiler deleted"}, status_code=200)

@router.get("/getReboilers")
async def get_reboilers():
    reboilers = connector.get_all_reboilers()
    if not reboilers:
        raise HTTPException(status_code=404, detail="No Reboilers found")
    return JSONResponse(content={"reboilers": reboilers}, status_code=200)

