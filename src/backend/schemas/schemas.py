from pydantic import BaseModel
from fastapi import File, UploadFile

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

class Predict(BaseModel):
    image: str