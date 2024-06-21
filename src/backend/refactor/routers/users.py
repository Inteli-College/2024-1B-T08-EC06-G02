from fastapi import APIRouter, HTTPException, Body
from fastapi.responses import JSONResponse
from databases.users import DatabaseConnector
from schemas.users import User, Role


router = APIRouter(tags=["users"])

connector = DatabaseConnector('repipe.db')

@router.post("/createUser", status_code=201)
async def create_user(user: User):
    new_user_id = connector.insert_in_table(user)
    if new_user_id is None:
        raise HTTPException(status_code=404, detail="Usuário não foi criado")
    return JSONResponse(content={"message": "Usuário Criado com Sucesso", "user_id": new_user_id}, status_code=201)

@router.get("/getUser/{user_id}")
async def get_user(user_id: int):
    user = connector.get_user(user_id)
    if user is None:
        raise HTTPException(status_code=404, detail="Usuário não encontrado")
    return JSONResponse(content={"user": user}, status_code=200)

@router.put("/updateUser/{user_id}")
async def update_user(user_id: int, user: User):
    update_count = connector.update_user(user_id, user)
    if update_count == 0:
        raise HTTPException(status_code=404, detail="Usuário não foi atualizado")
    return JSONResponse(content={"message": "Usuário atualizado com sucesso"}, status_code=200)

@router.delete("/deleteUser/{user_id}")
async def delete_user(user_id: int):
    delete_count = connector.delete_user(user_id)
    if delete_count == 0:
        raise HTTPException(status_code=404, detail="Usuário não foi deletado")
    return JSONResponse(content={"message": "Usuário deletado com sucesso"}, status_code=200)

@router.get("/getUsers")
async def get_users(): 
    users = connector.get_all_users()
    if not users:
        raise HTTPException(status_code=404, detail="Nenhum usuário encontrado")
    return JSONResponse(content={"users": users}, status_code=200)


