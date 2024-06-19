from fastapi import APIRouter, HTTPException, Body
from fastapi.responses import JSONResponse
from database.database import DatabaseConnector
from schemas.schemas import QuadrantZone


router = APIRouter(tags=["zones"])

connector = DatabaseConnector('repipe.db')



@router.post("/createQuadrantZone")
async def create_quadrant_zone(quadrant_zone: QuadrantZone):
    zone_id = connector.insert_quadrant_zone(quadrant_zone)
    if zone_id is None:
        raise HTTPException(status_code=404, detail="Quadrant Zone not created")
    return JSONResponse(content={"message": "Quadrant Zone created", "zone_id": zone_id}, status_code=201)

@router.get("/getQuadrantZone/{zone_id}")
async def get_quadrant_zone(zone_id: int):
    quadrant_zone = connector.get_quadrant_zone(zone_id)
    if quadrant_zone is None:
        raise HTTPException(status_code=404, detail="Quadrant Zone not found")
    return JSONResponse(content={"quadrant_zone": quadrant_zone}, status_code=200)

@router.put("/updateQuadrantZone/{zone_id}")
async def update_quadrant_zone(zone_id: int, quadrant_zone: QuadrantZone):
    update_count = connector.update_quadrant_zone(zone_id, quadrant_zone)
    if update_count == 0:
        raise HTTPException(status_code=404, detail="Quadrant Zone not updated")
    return JSONResponse(content={"message": "Quadrant Zone updated"}, status_code=200)

@router.delete("/deleteQuadrantZone/{zone_id}")
async def delete_quadrant_zone(zone_id: int):
    delete_count = connector.delete_quadrant_zone(zone_id)
    if delete_count == 0:
        raise HTTPException(status_code=404, detail="Quadrant Zone not deleted")
    return JSONResponse(content={"message": "Quadrant Zone deleted"}, status_code=200)

@router.get("/getQuadrantZones")
async def get_quadrant_zones():
    zones = connector.get_all_quadrant_zones()
    if not zones:
        raise HTTPException(status_code=404, detail="No Quadrant Zones found")
    return JSONResponse(content={"quadrant_zones": zones}, status_code=200)