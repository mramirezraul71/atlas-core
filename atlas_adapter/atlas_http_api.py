"""ATLAS HTTP API adapter
Expone /status /tools /execute usando el command_router.handle de C:\ATLAS\modules\command_router.py
"""
from fastapi import FastAPI
from pydantic import BaseModel
import importlib.util
from pathlib import Path

ATLAS_ROOT = Path(r"C:\ATLAS")
ROUTER_PATH = ATLAS_ROOT / "modules" / "command_router.py"

def load_handle():
    spec = importlib.util.spec_from_file_location("command_router", str(ROUTER_PATH))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)  # type: ignore
    return mod.handle  # type: ignore

handle = load_handle()
app = FastAPI(title="ATLAS Adapter", version="1.0.0")

class Step(BaseModel):
    tool: str
    args: dict = {}

@app.get("/status")
def status():
    return {"ok": True, "atlas": handle("/status")}

@app.get("/tools")
def tools():
    return {"ok": True, "tools": [
        "atlas.status","atlas.doctor","atlas.modules","atlas.snapshot",
        "atlas.note.create","atlas.note.append","atlas.note.view","atlas.inbox"
    ]}

@app.post("/execute")
def execute(step: Step):
    t = step.tool
    a = step.args or {}
    if t == "atlas.status": cmd = "/status"
    elif t == "atlas.doctor": cmd = "/doctor"
    elif t == "atlas.modules": cmd = "/modules"
    elif t == "atlas.snapshot": cmd = f"/snapshot {a.get('label','snapshot')}"
    elif t == "atlas.note.create": cmd = f"/note create {a.get('title','Nota')}"
    elif t == "atlas.note.append": cmd = f"/note append {a.get('title','Inbox')} | {a.get('text','')}"
    elif t == "atlas.note.view": cmd = f"/note view {a.get('title','Inbox')}"
    elif t == "atlas.inbox": cmd = a.get("text","")
    else:
        return {"ok": False, "error": f"Tool no soportada aún: {t}"}
    out = handle(cmd)
    return {"ok": True, "tool": t, "output": out}

@app.get("/modules")
def modules():
    return {
        "ok": True,
        "modules": [
            {"name": "vision", "enabled": False},
            {"name": "voice", "enabled": False},
            {"name": "agent_router", "enabled": False},
            {"name": "telegram", "enabled": True},
        ]
    }

@app.get("/modules")
def modules():
    return {
        "ok": True,
        "modules": [
            {"name": "vision", "enabled": False},
            {"name": "voice", "enabled": False},
            {"name": "agent_router", "enabled": False},
            {"name": "telegram", "enabled": True},
        ]
    }
