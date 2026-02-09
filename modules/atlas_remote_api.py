import os
from dotenv import load_dotenv
from fastapi import FastAPI, HTTPException
from pydantic import BaseModel

from modules.atlas_llm import atlas
from modules.command_router import route_command

load_dotenv(r"C:\ATLAS\config\.env")

API_TOKEN = os.getenv("ATLAS_TOKEN")
if not API_TOKEN:
    raise RuntimeError("Falta ATLAS_TOKEN en C:\\ATLAS\\config\\.env")

app = FastAPI(title="ATLAS Remote API", version="1.0")

class Req(BaseModel):
    token: str
    text: str

@app.get("/health")
def health():
    return {"ok": True}

@app.post("/chat")
def chat(req: Req):
    if req.token != API_TOKEN:
        raise HTTPException(status_code=401, detail="token inválido")

    system = (
        "Eres ATLAS, asistente de Raúl. Responde en español y con acciones ejecutables si aplica."
    )
    reply = atlas.think(req.text, system=system)
    return {"reply": reply}

@app.post("/cmd")
def cmd(req: Req):
    if req.token != API_TOKEN:
        raise HTTPException(status_code=401, detail="token inválido")
    out = route_command(req.text)
    return {"result": out}
