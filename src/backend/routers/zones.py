from fastapi import APIRouter, HTTPException
from fastapi.responses import JSONResponse
from database.connector import DatabaseConnection
from schemas.schemas import QuadrantZone

router = APIRouter(tags=["zones"])
connector = DatabaseConnection()

@router.post("/createQuadrantZone")
async def create_quadrant_zone(quadrant_zone: QuadrantZone):
    operation_result = connector.query_database(operation="INSERT", table_name="QuadrantZones", data=quadrant_zone)
    if operation_result != "Dados inserido com sucesso":
        raise HTTPException(status_code=404, detail="Quadrant Zone not created")
    return JSONResponse(content={"message": "Quadrant Zone created", "zone_id": quadrant_zone.zone_id}, status_code=201)

@router.get("/getQuadrantZone/{zone_id}")
async def get_quadrant_zone(zone_id: int):
    quadrant_zone = connector.query_database(operation="SELECT", table_name="QuadrantZones", where={"zone_id": zone_id})
    if not quadrant_zone:
        raise HTTPException(status_code=404, detail="Quadrant Zone not found")
    return JSONResponse(content={"quadrant_zone": quadrant_zone}, status_code=200)

@router.put("/updateQuadrantZone/{zone_id}")
async def update_quadrant_zone(zone_id: int, quadrant_zone: QuadrantZone):
    update_result = connector.query_database(operation="UPDATE", table_name="QuadrantZones", data=quadrant_zone, where={"zone_id": zone_id})
    if update_result != "Dados atualizado com sucesso":
        raise HTTPException(status_code=404, detail="Quadrant Zone not updated")
    return JSONResponse(content={"message": "Quadrant Zone updated"}, status_code=200)

@router.delete("/deleteQuadrantZone/{zone_id}")
async def delete_quadrant_zone(zone_id: int):
    delete_result = connector.query_database(operation="DELETE", table_name="QuadrantZones", where={"zone_id": zone_id})
    if delete_result != "Dados deletado com sucesso":
        raise HTTPException(status_code=404, detail="Quadrant Zone not deleted")
    return JSONResponse(content={"message": "Quadrant Zone deleted"}, status_code=200)

@router.get("/getQuadrantZones")
async def get_quadrant_zones():
    zones = connector.query_database(operation="SELECT", table_name="QuadrantZones")
    if not zones:
        raise HTTPException(status_code=404, detail="No Quadrant Zones found")
    return JSONResponse(content={"quadrant_zones": zones}, status_code=200)
