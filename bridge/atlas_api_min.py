import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
import uvicorn

from atlas_runtime import handle, status, doctor, modules_report

app = FastAPI()

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)

@app.get("/status")
def _status():
    return {"ok": True, "atlas": status()}

@app.get("/tools")
def _tools():
    return {
        "ok": True,
        "tools": [
            {"name": "atlas.status", "path": "/status", "method": "GET"},
            {"name": "atlas.doctor", "path": "/doctor", "method": "GET"},
            {"name": "atlas.modules", "path": "/modules", "method": "GET"},
            {"name": "atlas.run", "path": "/run", "method": "POST"},
        ],
    }

@app.get("/doctor")
def _doctor():
    return {"ok": True, "result": doctor()}

@app.get("/modules")
def _modules():
    return {"ok": True, "result": modules_report()}

@app.post("/run")
def _run(payload: dict):
    text = (payload or {}).get("text", "")
    return {"ok": True, "result": handle(text)}

if __name__ == "__main__":
    uvicorn.run(app, host="127.0.0.1", port=8791)

