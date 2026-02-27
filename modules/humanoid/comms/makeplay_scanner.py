"""Scanner permanente MakePlay: envía estado ATLAS en tiempo real a webhook externo.
Habilita feedback loop: ATLAS se supera constantemente, Make.com recibe actualizaciones."""
from __future__ import annotations

import os
from datetime import datetime, timezone
from typing import Any, Dict


def run_scan() -> Dict[str, Any]:
    """Recopila snapshot del estado actual y lo envía al webhook."""
    try:
        from modules.humanoid.ans.live_stream import emit
        emit("scanner_start", message="Scanner MakePlay")
    except Exception:
        pass
    snapshot = _build_snapshot()
    try:
        from modules.humanoid.comms.webhook_bridge import push_scan
        pushed = push_scan(snapshot)
        snapshot["webhook_pushed"] = pushed
    except Exception as e:
        snapshot["webhook_pushed"] = False
        snapshot["webhook_error"] = str(e)
    snapshot["ts"] = datetime.now(timezone.utc).isoformat()
    try:
        from modules.humanoid.comms.scanner_store import record_scan
        record_scan(snapshot)
    except Exception:
        pass
    try:
        from modules.humanoid.ans.live_stream import emit
        emit("scanner_end", message=f"health={snapshot.get('health_score')} incidents={snapshot.get('incidents_open')} webhook={snapshot.get('webhook_pushed')}", details={"health_score": snapshot.get("health_score"), "incidents_open": snapshot.get("incidents_open")})
    except Exception:
        pass
    return {"ok": True, "snapshot": snapshot}


def _build_snapshot() -> Dict[str, Any]:
    """Estado actual: health, incidentes, últimas acciones, autoconocimiento."""
    out: Dict[str, Any] = {
        "health_score": None,
        "health_ok": None,
        "incidents_open": 0,
        "incidents_recent": [],
        "actions_last_hour": 0,
        "last_actions": [],
        "self_model_checks": 0,
        "self_model_heals": 0,
    }
    try:
        from modules.humanoid.deploy.healthcheck import run_health_verbose
        from modules.humanoid.deploy.ports import get_ports
        port = get_ports()[0] if get_ports() else int(os.getenv("ACTIVE_PORT", "8791") or 8791)
        h = run_health_verbose(base_url=None, active_port=port)
        out["health_score"] = h.get("score")
        out["health_ok"] = h.get("ok")
        out["checks"] = h.get("checks", {})
    except Exception:
        pass
    try:
        from modules.humanoid.ans.incident import get_incidents
        open_inc = get_incidents(status="open", limit=10)
        out["incidents_open"] = len(open_inc)
        out["incidents_recent"] = [
            {"check_id": i.get("check_id"), "message": (i.get("message") or "")[:80]}
            for i in open_inc[:5]
        ]
    except Exception:
        pass
    try:
        from modules.humanoid.ans.limits import actions_last_hour
        out["actions_last_hour"] = actions_last_hour()
    except Exception:
        pass
    try:
        from modules.humanoid.ans.live_stream import get_recent
        recent = get_recent(limit=10)
        out["last_actions"] = [
            {"phase": e.get("phase"), "check_id": e.get("check_id"), "heal_id": e.get("heal_id"), "message": (e.get("message") or "")[:60]}
            for e in recent if e.get("phase") in ("heal_end", "incident", "cycle_end")
        ]
    except Exception:
        pass
    try:
        from modules.humanoid.self_model import get_manifest
        m = get_manifest()
        ns = m.get("anatomy", {}).get("nervous_system", {})
        out["self_model_checks"] = len(ns.get("checks", []))
        out["self_model_heals"] = len(ns.get("heals", []))
    except Exception:
        pass
    return out
