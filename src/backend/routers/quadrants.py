from fastapi import APIRouter, HTTPException, Body
from fastapi.responses import JSONResponse
from database.connector import DatabaseConnection  # Certifique-se de que o caminho do import está correto
from schemas.schemas import Quadrant

router = APIRouter(tags=["quadrants"])
db_path = 'repipe.db'  # Certifique-se de que o caminho do banco de dados está correto
connector = DatabaseConnection(db_path)

@router.post("/createQuadrant")
async def create_quadrant(quadrant: Quadrant):
    operation_result = connector.query_database(operation="INSERT", table_name="Quadrants", data=quadrant)
    if operation_result != "Dados inserido com sucesso":
        raise HTTPException(status_code=404, detail="Quadrant not created")
    return JSONResponse(content={"message": "Quadrant created"}, status_code=201)

@router.get("/getQuadrant/{quadrant_id}")
async def get_quadrant(quadrant_id: int):
    quadrant = connector.query_database(operation="SELECT", table_name="Quadrants", where={"quadrant_id": quadrant_id})
    if not quadrant:
        raise HTTPException(status_code=404, detail="Quadrant not found")
    return JSONResponse(content={"quadrant": quadrant}, status_code=200)

@router.put("/updateQuadrant/{quadrant_id}")
async def update_quadrant(quadrant_id: int, quadrant: Quadrant):
    update_result = connector.query_database(operation="UPDATE", table_name="Quadrants", data=quadrant, where={"quadrant_id": quadrant_id})
    if update_result != "Dados atualizado com sucesso":
        raise HTTPException(status_code=404, detail="Quadrant not updated")
    return JSONResponse(content={"message": "Quadrant updated"}, status_code=200)

@router.delete("/deleteQuadrant/{quadrant_id}")
async def delete_quadrant(quadrant_id: int):
    delete_result = connector.query_database(operation="DELETE", table_name="Quadrants", where={"quadrant_id": quadrant_id})
    if delete_result != "Dados deletado com sucesso":
        raise HTTPException(status_code=404, detail="Quadrant not deleted")
    return JSONResponse(content={"message": "Quadrant deleted"}, status_code=200)

@router.get("/getQuadrants")
async def get_quadrants():
    quadrants = connector.query_database(operation="SELECT", table_name="Quadrants")
    if not quadrants:
        raise HTTPException(status_code=404, detail="No quadrants found")
    return JSONResponse(content={"quadrants": quadrants}, status_code=200)
