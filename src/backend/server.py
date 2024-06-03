from fastapi import FastAPI, HTTPException
from fastapi.responses import JSONResponses

app = FastAPI(swagger_ui_parameters={"syntaxHighlight.theme": "obsidian"})


@app.get("/users/{username}")
async def read_user(username: str):
    return {"message": f"Hello {username}"}


