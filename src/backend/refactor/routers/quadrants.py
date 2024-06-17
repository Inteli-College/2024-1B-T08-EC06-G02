from fastapi import APIRouter, HTTPException, Body
from fastapi.responses import JSONResponse
from database.database import DatabaseConnector
from schemas.schemas import Quadrant


router = APIRouter(tags=["quadrants"])

connector = DatabaseConnector('repipe.db')



@router.post("/createQuadrant")
async def create_quadrant(quadrant: Quadrant):
    quadrant_id = connector.insert_quadrant(quadrant)
    if quadrant_id is None:
        raise HTTPException(status_code=404, detail="Quadrant not created")
    return JSONResponse(content={"message": "Quadrant created", "quadrant_id": quadrant_id}, status_code=201)

@router.get("/getQuadrant/{quadrant_id}")
async def get_quadrant(quadrant_id: int):
    quadrant = connector.get_quadrant(quadrant_id)
    if quadrant is None:
        raise HTTPException(status_code=404, detail="Quadrant not found")
    return JSONResponse(content={"quadrant": quadrant}, status_code=200)

@router.put("/updateQuadrant/{quadrant_id}")
async def update_quadrant(quadrant_id: int, quadrant: Quadrant):
    update_count = connector.update_quadrant(quadrant_id, quadrant)
    if update_count == 0:
        raise HTTPException(status_code=404, detail="Quadrant not updated")
    return JSONResponse(content={"message": "Quadrant updated"}, status_code=200)

@router.delete("/deleteQuadrant/{quadrant_id}")
async def delete_quadrant(quadrant_id: int):
    delete_count = connector.delete_quadrant(quadrant_id)
    if delete_count == 0:
        raise HTTPException(status_code=404, detail="Quadrant not deleted")
    return JSONResponse(content={"message": "Quadrant deleted"}, status_code=200)

@router.get("/getQuadrants")
async def get_quadrants():
    quadrants = connector.get_all_quadrants()
    if not quadrants:
        raise HTTPException(status_code=404, detail="No quadrants found")
    return JSONResponse(content={"quadrants": quadrants}, status_code=200)