from fastapi import FastAPI, HTTPException, Body
from fastapi.responses import JSONResponse
from pydantic import BaseModel
from database import DatabaseConnector

app = FastAPI(swagger_ui_parameters={"syntaxHighlight.theme": "obsidian"})

connector = DatabaseConnector('repipe.db')

#Definições dos BaseModels de todas as tabelas (SERÁ REFATORADO !!!!!!!!!!!!!!!! NÃO TEREMOS UM SERVER.py com 1238912983128973897321 LINHAS) Mano EU odeio esse Codigo
class User(BaseModel):
    user_email: str
    user_password: str
    user_role: int
    user_name: str

class Role(BaseModel):
    role_id: int
    role_name: str
    role_permission_level: int

class Quadrant(BaseModel):
    quadrant_position: int
    quadrant_status: str
    reboiler_id: int

class QuadrantZone(BaseModel):
    zone_area: int
    zone_status: int
    quadrant_id: int

class Reboiler(BaseModel):
    num_pipes: int
    status: str
    refinery_id: int

class Refinary(BaseModel):
    location: str
    num_reboilers: int
    name: str




#OPERAÇÕES DE CRUD PRA USUÁRIOS

@app.post("/createUser", status_code=201)
async def create_user(user: User):
    new_user_id = connector.create_user(user)
    if new_user_id is None:
        raise HTTPException(status_code=404, detail="Usuário não foi criado")
    return JSONResponse(content={"message": "Usuário Criado com Sucesso", "user_id": new_user_id}, status_code=201)

@app.get("/getUser/{user_id}")
async def get_user(user_id: int):
    user = connector.get_user(user_id)
    if user is None:
        raise HTTPException(status_code=404, detail="Usuário não encontrado")
    return JSONResponse(content={"user": user}, status_code=200)

@app.put("/updateUser/{user_id}")
async def update_user(user_id: int, user: User):
    update_count = connector.update_user(user_id, user)
    if update_count == 0:
        raise HTTPException(status_code=404, detail="Usuário não foi atualizado")
    return JSONResponse(content={"message": "Usuário atualizado com sucesso"}, status_code=200)

@app.delete("/deleteUser/{user_id}")
async def delete_user(user_id: int):
    delete_count = connector.delete_user(user_id)
    if delete_count == 0:
        raise HTTPException(status_code=404, detail="Usuário não foi deletado")
    return JSONResponse(content={"message": "Usuário deletado com sucesso"}, status_code=200)

@app.get("/getUsers")
async def get_users(): 
    users = connector.get_all_users()
    if not users:
        raise HTTPException(status_code=404, detail="Nenhum usuário encontrado")
    return JSONResponse(content={"users": users}, status_code=200)



#OPERAÇÕES DE CRUD PARA QUADRANTES 

@app.post("/createQuadrant")
async def create_quadrant(quadrant: Quadrant):
    quadrant_id = connector.insert_quadrant(quadrant)
    if quadrant_id is None:
        raise HTTPException(status_code=404, detail="Quadrant not created")
    return JSONResponse(content={"message": "Quadrant created", "quadrant_id": quadrant_id}, status_code=201)

@app.get("/getQuadrant/{quadrant_id}")
async def get_quadrant(quadrant_id: int):
    quadrant = connector.get_quadrant(quadrant_id)
    if quadrant is None:
        raise HTTPException(status_code=404, detail="Quadrant not found")
    return JSONResponse(content={"quadrant": quadrant}, status_code=200)

@app.put("/updateQuadrant/{quadrant_id}")
async def update_quadrant(quadrant_id: int, quadrant: Quadrant):
    update_count = connector.update_quadrant(quadrant_id, quadrant)
    if update_count == 0:
        raise HTTPException(status_code=404, detail="Quadrant not updated")
    return JSONResponse(content={"message": "Quadrant updated"}, status_code=200)

@app.delete("/deleteQuadrant/{quadrant_id}")
async def delete_quadrant(quadrant_id: int):
    delete_count = connector.delete_quadrant(quadrant_id)
    if delete_count == 0:
        raise HTTPException(status_code=404, detail="Quadrant not deleted")
    return JSONResponse(content={"message": "Quadrant deleted"}, status_code=200)

@app.get("/getQuadrants")
async def get_quadrants():
    quadrants = connector.get_all_quadrants()
    if not quadrants:
        raise HTTPException(status_code=404, detail="No quadrants found")
    return JSONResponse(content={"quadrants": quadrants}, status_code=200)



#OPERAÇÕES DE CRUD PARA ZONAS DE QUADRANTES

@app.post("/createQuadrantZone")
async def create_quadrant_zone(quadrant_zone: QuadrantZone):
    zone_id = connector.insert_quadrant_zone(quadrant_zone)
    if zone_id is None:
        raise HTTPException(status_code=404, detail="Quadrant Zone not created")
    return JSONResponse(content={"message": "Quadrant Zone created", "zone_id": zone_id}, status_code=201)

@app.get("/getQuadrantZone/{zone_id}")
async def get_quadrant_zone(zone_id: int):
    quadrant_zone = connector.get_quadrant_zone(zone_id)
    if quadrant_zone is None:
        raise HTTPException(status_code=404, detail="Quadrant Zone not found")
    return JSONResponse(content={"quadrant_zone": quadrant_zone}, status_code=200)

@app.put("/updateQuadrantZone/{zone_id}")
async def update_quadrant_zone(zone_id: int, quadrant_zone: QuadrantZone):
    update_count = connector.update_quadrant_zone(zone_id, quadrant_zone)
    if update_count == 0:
        raise HTTPException(status_code=404, detail="Quadrant Zone not updated")
    return JSONResponse(content={"message": "Quadrant Zone updated"}, status_code=200)

@app.delete("/deleteQuadrantZone/{zone_id}")
async def delete_quadrant_zone(zone_id: int):
    delete_count = connector.delete_quadrant_zone(zone_id)
    if delete_count == 0:
        raise HTTPException(status_code=404, detail="Quadrant Zone not deleted")
    return JSONResponse(content={"message": "Quadrant Zone deleted"}, status_code=200)

@app.get("/getQuadrantZones")
async def get_quadrant_zones():
    zones = connector.get_all_quadrant_zones()
    if not zones:
        raise HTTPException(status_code=404, detail="No Quadrant Zones found")
    return JSONResponse(content={"quadrant_zones": zones}, status_code=200)



#OPERAÇÕES DE CRUD PARA REBOILERS

@app.post("/createReboiler")
async def create_reboiler(reboiler: Reboiler):
    reboiler_id = connector.insert_reboiler(reboiler)
    if reboiler_id is None:
        raise HTTPException(status_code=404, detail="Reboiler not created")
    return JSONResponse(content={"message": "Reboiler created", "reboiler_id": reboiler_id}, status_code=201)

@app.get("/getReboiler/{reboiler_id}")
async def get_reboiler(reboiler_id: int):
    reboiler = connector.get_reboiler(reboiler_id)
    if reboiler is None:
        raise HTTPException(status_code=404, detail="Reboiler not found")
    return JSONResponse(content={"reboiler": reboiler}, status_code=200)

@app.put("/updateReboiler/{reboiler_id}")
async def update_reboiler(reboiler_id: int, reboiler: Reboiler):
    update_count = connector.update_reboiler(reboiler_id, reboiler)
    if update_count == 0:
        raise HTTPException(status_code=404, detail="Reboiler not updated")
    return JSONResponse(content={"message": "Reboiler updated"}, status_code=200)

@app.delete("/deleteReboiler/{reboiler_id}")
async def delete_reboiler(reboiler_id: int):
    delete_count = connector.delete_reboiler(reboiler_id)
    if delete_count == 0:
        raise HTTPException(status_code=404, detail="Reboiler not deleted")
    return JSONResponse(content={"message": "Reboiler deleted"}, status_code=200)

@app.get("/getReboilers")
async def get_reboilers():
    reboilers = connector.get_all_reboilers()
    if not reboilers:
        raise HTTPException(status_code=404, detail="No Reboilers found")
    return JSONResponse(content={"reboilers": reboilers}, status_code=200)



#OPERAÇÕES DE CRUD PARA REFINARIAS

@app.post("/createRefinary")
async def create_refinary(refinary: Refinary):
    refinary_id = connector.insert_refinary(refinary)
    if refinary_id is None:
        raise HTTPException(status_code=404, detail="Refinary not created")
    return JSONResponse(content={"message": "Refinary created", "refinary_id": refinary_id}, status_code=201)

@app.get("/getRefinary/{refinary_id}")
async def get_refinary(refinary_id: int):
    refinary = connector.get_refinary(refinary_id)
    if refinary is None:
        raise HTTPException(status_code=404, detail="Refinary not found")
    return JSONResponse(content={"refinary": refinary}, status_code=200)

@app.put("/updateRefinary/{refinary_id}")
async def update_refinary(refinary_id: int, refinary: Refinary):
    update_count = connector.update_refinary(refinary_id, refinary)
    if update_count == 0:
        raise HTTPException(status_code=404, detail="Refinary not updated")
    return JSONResponse(content={"message": "Refinary updated"}, status_code=200)

@app.delete("/deleteRefinary/{refinary_id}")
async def delete_refinary(refinary_id: int):
    delete_count = connector.delete_refinary(refinary_id)
    if delete_count == 0:
        raise HTTPException(status_code=404, detail="Refinary not deleted")
    return JSONResponse(content={"message": "Refinary deleted"}, status_code=200)

@app.get("/getRefinaries")
async def get_refinaries():
    refinaries = connector.get_all_refinaries()
    if not refinaries:
        raise HTTPException(status_code=404, detail="No Refinaries found")
    return JSONResponse(content={"refinaries": refinaries}, status_code=200)


