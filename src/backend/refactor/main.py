from fastapi import FastAPI
from routers import users, zones, quadrants, reboilers, refinaries

app = FastAPI()

# Include the users router
app.include_router(users.router)
app.include_router(zones.router)
app.include_router(quadrants.router)
app.include_router(reboilers.router)
app.include_router(refinaries.router)


@app.get("/")
def read_root():
    return {"message": "Welcome to the FastAPI application!"}
