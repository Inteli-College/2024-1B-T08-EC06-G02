from fastapi import FastAPI
from routers import users, zones, quadrants, reboilers, refinaries, predict, logs
import uvicorn
from fastapi.middleware.cors import CORSMiddleware

app = FastAPI()


app.include_router(users.router)
app.include_router(zones.router)
app.include_router(quadrants.router)
app.include_router(reboilers.router)
app.include_router(refinaries.router)
app.include_router(predict.router)
app.include_router(logs.router)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:3000"],  # Update with your frontend URL
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

@app.get("/")
def read_root():
    return {"message": "AHahahahahahhahahahahahahahahahahahahahahahahahhhahahahahahahahahahahahahahahahahahahahahahaha!"}
    