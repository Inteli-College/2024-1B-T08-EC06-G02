from fastapi import FastAPI
from routers import users

app = FastAPI()

# Include the users router
app.include_router(users.router)

@app.get("/")
def read_root():
    return {"message": "Welcome to the FastAPI application!"}
