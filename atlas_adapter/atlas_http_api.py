"""ATLAS HTTP API adapter.

Expone /status, /tools, /modules, /execute, /intent usando
``modules.command_router.handle`` directamente, y envolviendo /intent
con ``atlas_push.intents.IntentRouter`` (ver
``docs/atlas_push/PLAN_STEP_B.md`` y ``docs/atlas_push/PLAN_STEP_C.md``).
"""
from typing import Any, Optional
import time

from fastapi import FastAPI
from pydantic import BaseModel

from modules.command_router import handle
from atlas_push.intents import IntentRouter

app = FastAPI(title="ATLAS Adapter", version="1.0.0")

# Instancia única a nivel de módulo: IntentRouter es stateless, se
# puede reutilizar entre requests. Ver docs/atlas_push/PLAN_STEP_B.md.
_intent_router = IntentRouter()


class Step(BaseModel):
    tool: str
    args: dict = {}


class IntentIn(BaseModel):
    user: str = "raul"
    text: str
    meta: Optional[dict[str, Any]] = None


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


@app.post("/intent")
def intent(payload: IntentIn):
    """
    Endpoint canónico: recibe intención (texto) y la resuelve a través
    del IntentRouter (que delega en command_router.handle por dentro).
    Ejemplos: "/status", "/modules", "hola", etc.
    """
    t0 = time.time()
    try:
        result = _intent_router.handle(payload.text)
        return {
            "ok": True,
            "input": payload.model_dump(),
            "output": result.output,
            "ms": int((time.time() - t0) * 1000),
        }
    except Exception as e:
        return {
            "ok": False,
            "input": payload.model_dump(),
            "error": str(e),
            "ms": int((time.time() - t0) * 1000),
        }
