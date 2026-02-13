"""ANS API: status, incidents, report, run-now, live stream, self-model."""
from __future__ import annotations

from fastapi import APIRouter
from fastapi.responses import StreamingResponse
from pydantic import BaseModel
from typing import Optional
import json

router = APIRouter(prefix="/ans", tags=["ANS"])


@router.get("/status")
def ans_status():
    from .engine import get_ans_status
    return get_ans_status()


@router.get("/incidents")
def ans_incidents(status: Optional[str] = None, limit: int = 50):
    try:
        from .incident import get_incidents
        items = get_incidents(status=status, limit=limit)
        return {"ok": True, "data": items}
    except Exception as e:
        return {"ok": False, "data": [], "error": str(e)}


@router.get("/bitacora")
def ans_bitacora(limit: int = 30):
    """Bitácora: problema → acción ejecutada → resultado."""
    try:
        from .incident import get_incidents
        items = get_incidents(status=None, limit=limit * 2)
        entries = []
        for inc in items[:limit]:
            prob = inc.get("message", inc.get("check_id", "?"))
            actions = inc.get("actions_taken", [])
            if actions:
                for a in actions:
                    accion = a.get("heal_id", "—")
                    res = "OK" if a.get("ok") else "Error"
                    entries.append({
                        "timestamp": inc.get("created_at"),
                        "problema": prob[:120],
                        "accion": accion,
                        "resultado": res,
                        "detalle": (a.get("message") or "")[:100],
                    })
            else:
                entries.append({
                    "timestamp": inc.get("created_at"),
                    "problema": prob[:120],
                    "accion": "—",
                    "resultado": "pendiente" if inc.get("status") == "open" else "—",
                    "detalle": "",
                })
        return {"ok": True, "data": entries[:limit]}
    except Exception as e:
        return {"ok": False, "data": [], "error": str(e)}


@router.get("/report/latest")
def ans_report_latest():
    try:
        from .reporter import get_latest_report
        path = get_latest_report()
        return {"ok": True, "path": path}
    except Exception as e:
        return {"ok": False, "path": "", "error": str(e)}


class RunNowBody(BaseModel):
    mode: Optional[str] = None


@router.get("/live/recent")
def ans_live_recent(limit: int = 100):
    """Últimos eventos del stream (polling). Estilo Cursor."""
    try:
        from .live_stream import get_recent
        return {"ok": True, "events": get_recent(limit=limit)}
    except Exception as e:
        return {"ok": False, "events": [], "error": str(e)}


@router.get("/live")
async def ans_live_stream():
    """SSE: señales en vivo de lo que ejecuta el ANS. Estilo Cursor."""
    async def gen():
        from .live_stream import stream_events
        async for ev in stream_events():
            yield f"data: {json.dumps(ev, ensure_ascii=False)}\n\n"
    return StreamingResponse(
        gen(),
        media_type="text/event-stream",
        headers={"Cache-Control": "no-cache", "X-Accel-Buffering": "no"},
    )


@router.get("/scanner-status")
def ans_scanner_status():
    """Último scan (MakePlay) y últimas descargas pip."""
    try:
        from modules.humanoid.comms.scanner_store import get_last_scan, get_last_downloads
        return {
            "ok": True,
            "last_scan": get_last_scan(),
            "last_downloads": get_last_downloads(limit=20),
        }
    except Exception as e:
        return {"ok": False, "last_scan": {}, "last_downloads": [], "error": str(e)}


@router.get("/self-model")
def ans_self_model():
    """Autoconocimiento: anatomía del sistema, checks, heals, dependencias."""
    try:
        from modules.humanoid.self_model import get_manifest
        return {"ok": True, "manifest": get_manifest()}
    except Exception as e:
        return {"ok": False, "manifest": None, "error": str(e)}


@router.post("/run-now")
def ans_run_now(body: Optional[RunNowBody] = None):
    try:
        from modules.humanoid.policy import ActorContext, get_policy_engine
        ctx = ActorContext(actor="api", role="owner")
        decision = get_policy_engine().can(ctx, "ans", "ans_run_now", None)
        if not decision.allow:
            return {"ok": False, "error": decision.reason}
        from .engine import run_ans_cycle
        mode = body.mode if body else None
        r = run_ans_cycle(mode=mode, timeout_sec=60)
        return r
    except Exception as e:
        return {"ok": False, "error": str(e)}
