from fastapi import APIRouter, HTTPException
from fastapi.responses import JSONResponse

router = APIRouter(prefix="/pipe", tags=["pipe"])

@router.post("/createUser", status_code=201)
async def create_pipe(pipe: Pipe):
    img_pipe = create_pipe(pipe)
    if img_pipe is None:
        raise HTTPException(status_code=404, detail="Usuário não foi criado")
    return JSONResponse(content={"message": "Imagem não recebida"}, status_code=201)
