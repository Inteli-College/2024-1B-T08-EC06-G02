from fastapi import APIRouter, HTTPException
from fastapi.responses import JSONResponse
from database.connector import DatabaseConnection
from schemas.schemas import User, Role

router = APIRouter(tags=["users"])
connector = DatabaseConnection()

@router.post("/createUser", status_code=201)
async def create_user(user: User):
    operation_result = connector.query_database(operation="INSERT", table_name="Users", data=user)
    if operation_result != "Dados inserido com sucesso":
        raise HTTPException(status_code=404, detail="User not created")
    return JSONResponse(content={"message": "User created successfully", "user_name": user.user_name}, status_code=201)

@router.get("/getUser/{user_id}")
async def get_user(user_id: int):
    user = connector.query_database(operation="SELECT", table_name="Users", where={"user_id": user_id})
    if not user:
        raise HTTPException(status_code=404, detail="User not found")
    return JSONResponse(content={"user": user}, status_code=200)

@router.put("/updateUser/{user_id}")
async def update_user(user_id: int, user: User):
    update_result = connector.query_database(operation="UPDATE", table_name="Users", data=user, where={"user_id": user_id})
    if update_result != "Dados atualizado com sucesso":
        raise HTTPException(status_code=404, detail="User not updated")
    return JSONResponse(content={"message": "User updated successfully"}, status_code=200)

@router.delete("/deleteUser/{user_id}")
async def delete_user(user_id: int):
    delete_result = connector.query_database(operation="DELETE", table_name="Users", where={"user_id": user_id})
    if delete_result != "Dados deletado com sucesso":
        raise HTTPException(status_code=404, detail="User not deleted")
    return JSONResponse(content={"message": "User deleted successfully"}, status_code=200)

@router.get("/getUsers")
async def get_users(): 
    users = connector.query_database(operation="SELECT", table_name="Users")
    if not users:
        raise HTTPException(status_code=404, detail="No users found")
    return JSONResponse(content={"users": users}, status_code=200)
