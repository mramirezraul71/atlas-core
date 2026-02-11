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

# Metrics middleware: request count + latency per path
from modules.humanoid.metrics import MetricsMiddleware
app.add_middleware(MetricsMiddleware)

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
from pydantic import BaseModel
from typing import Any, Optional
import time

class IntentIn(BaseModel):
    user: str = "raul"
    text: str
    meta: Optional[dict[str, Any]] = None

@app.post("/intent")
def intent(payload: IntentIn):
    # Respuesta mínima (v1): eco + timestamp.
    # Luego conectamos command_router/agent_router.
    return {
        "ok": True,
        "received": payload.model_dump(),
        "result": {
            "message": "Intent recibido",
            "ts": time.time(),
        }
    }
# --- Canonical Intent API (v1) ---
from pydantic import BaseModel
from typing import Any, Optional
import time

from modules.command_router import handle as route_command

class IntentIn(BaseModel):
    user: str = "raul"
    text: str = ""
    meta: Optional[dict[str, Any]] = None

@app.post("/intent")
def intent(payload: IntentIn):
    t0 = time.time()
    if not (payload.text or "").strip():
        return {
            "ok": False,
            "received": payload.model_dump(),
            "error": "Campo 'text' es requerido (ej: {\"text\": \"/status\"})",
            "ms": int((time.time() - t0) * 1000),
        }
    try:
        out = route_command((payload.text or "").strip())
        return {
            "ok": True,
            "received": payload.model_dump(),
            "output": out,
            "ms": int((time.time() - t0) * 1000),
        }
    except Exception as e:
        return {
            "ok": False,
            "received": payload.model_dump(),
            "error": str(e),
            "ms": int((time.time() - t0) * 1000),
        }


# --- LLM híbrido (router + Ollama) ---
from modules.llm.schemas import LLMRequest, LLMResponse
from modules.llm.service import LLMService

llm_service = LLMService()


@app.post(
    "/llm",
    response_model=LLMResponse,
    summary="Hybrid LLM (Ollama)",
    description="Route prompt to FAST/CHAT/CODE/REASON/TOOLS via HybridRouter and call Ollama. Returns ok, output, route, model_used, ms, tokens_est, error, meta.",
)
def llm_endpoint(req: LLMRequest) -> LLMResponse:
    """Run LLM request through router + Ollama and return structured response."""
    return llm_service.run(req)


# --- Humanoid (kernel + modules) ---
from modules.humanoid.api import router as humanoid_router

app.include_router(humanoid_router)


# --- Metrics / Policy / Audit endpoints ---
from modules.humanoid.metrics import get_metrics_store
from modules.humanoid.policy import ActorContext, get_policy_engine
from modules.humanoid.audit import get_audit_logger


@app.get("/metrics")
def metrics():
    """JSON metrics: counters and latencies per endpoint."""
    return get_metrics_store().snapshot()


class PolicyTestBody(BaseModel):
    actor: str = "api"
    role: str = "owner"
    module: str = "hands"
    action: str = "exec_command"
    target: Optional[str] = None


@app.post("/policy/test")
def policy_test(body: PolicyTestBody):
    """Test policy: {actor, role, module, action, target} -> {allow, reason}."""
    ctx = ActorContext(actor=body.actor, role=body.role)
    decision = get_policy_engine().can(ctx, body.module, body.action, body.target)
    return {"allow": decision.allow, "reason": decision.reason, "details": decision.details}


@app.get("/audit/tail")
def audit_tail(n: int = 50, module: Optional[str] = None):
    """Last n audit log entries, optional filter by module."""
    entries = get_audit_logger().tail(n=n, module=module)
    if not entries and get_audit_logger()._db is None:
        return {"ok": True, "entries": [], "error": "AUDIT_DB_PATH not set"}
    return {"ok": True, "entries": entries}
