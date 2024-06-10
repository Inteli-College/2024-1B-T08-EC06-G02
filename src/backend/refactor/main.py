from fastapi import FastAPI

from routers import users

app = FastAPI(swagger_ui_parameters={"syntaxHighlight.theme": "obsidian"}) 

app.include_router(users.router)

