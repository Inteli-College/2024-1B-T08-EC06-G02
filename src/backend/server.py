from fastapi import FastAPI, HTTPException
from fastapi.responses import JSONResponse
from database import DatabaseConnector

app = FastAPI(swagger_ui_parameters={"syntaxHighlight.theme": "obsidian"})

connector = DatabaseConnector('repipe.db')

@app.post("/users/{username}", status_code=201)
async def create_user(username: str):
    new_user = connector.insert_in_table(username)
    if new_user is None:
        raise HTTPException(status_code=404, detail="User not created")
    return JSONResponse(content={"message": "User created"}, status_code=201)

@app.get("/users/{username}")
async def read_user(username: str):
    user = connector.get_user(username)
    if user is None:
        raise HTTPException(status_code=404, detail="User not found")
    return JSONResponse(content={"message": "User found"}, status_code=200)

@app.put("/users/{username}")
async def update_user(username: str):
    update_user = connector.update_user(username)
    if update_user is None:
        raise HTTPException(status_code=404, detail="User not found")
    return JSONResponse(content={"message": "User updated"}, status_code=200)

@app.delete("/users/{username}")
async def delete_user(username: str):
    delete_user = connector.delete_user(username)
    if not delete_user:
        raise HTTPException(status_code=404, detail="User not deleted")
    return JSONResponse(content={"message": "User deleted"}, status_code=200)   
