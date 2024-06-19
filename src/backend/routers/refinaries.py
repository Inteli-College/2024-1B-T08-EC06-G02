from fastapi import APIRouter, HTTPException, Body
from fastapi.responses import JSONResponse
from database.database import DatabaseConnector
from schemas.schemas import Refinary


router = APIRouter(tags=["refinaries"])

connector = DatabaseConnector('repipe.db')

@router.post("/createRefinary")
async def create_refinary(refinary: Refinary):
    refinary_id = connector.insert_refinary(refinary)
    if refinary_id is None:
        raise HTTPException(status_code=404, detail="Refinary not created")
    return JSONResponse(content={"message": "Refinary created", "refinary_id": refinary_id}, status_code=201)

@router.get("/getRefinary/{refinary_id}")
async def get_refinary(refinary_id: int):
    refinary = connector.get_refinary(refinary_id)
    if refinary is None:
        raise HTTPException(status_code=404, detail="Refinary not found")
    return JSONResponse(content={"refinary": refinary}, status_code=200)

@router.put("/updateRefinary/{refinary_id}")
async def update_refinary(refinary_id: int, refinary: Refinary):
    update_count = connector.update_refinary(refinary_id, refinary)
    if update_count == 0:
        raise HTTPException(status_code=404, detail="Refinary not updated")
    return JSONResponse(content={"message": "Refinary updated"}, status_code=200)

@router.delete("/deleteRefinary/{refinary_id}")
async def delete_refinary(refinary_id: int):
    delete_count = connector.delete_refinary(refinary_id)
    if delete_count == 0:
        raise HTTPException(status_code=404, detail="Refinary not deleted")
    return JSONResponse(content={"message": "Refinary deleted"}, status_code=200)

@router.get("/getRefinaries")
async def get_refinaries():
    refinaries = connector.get_all_refinaries()
    if not refinaries:
        raise HTTPException(status_code=404, detail="No Refinaries found")
    return JSONResponse(content={"refinaries": refinaries}, status_code=200)



