from pydantic import BaseModel

class User(BaseModel):
    user_email: str
    user_password: str
    user_role: int
    user_name: str

class Role(BaseModel):
    role_id: int
    role_name: str
    role_permission_level: int