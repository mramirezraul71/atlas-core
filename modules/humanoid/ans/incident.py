"""Incident lifecycle + dedupe + cooldown."""
from __future__ import annotations

import time
import uuid
from collections import deque
from typing import Any, Dict, List, Optional

_INCIDENTS: Dict[str, Dict[str, Any]] = {}
_FINGERPRINT_LAST: Dict[str, float] = {}
_MAX_INCIDENTS = 200


def _env_int(name: str, default: int) -> int:
    import os
    try:
        return int(os.getenv(name, str(default)).strip() or default)
    except Exception:
        return default


def create_incident(check_id: str, fingerprint: str, severity: str, message: str, evidence: Dict[str, Any] = None, suggested_heals: List[str] = None) -> str:
    cooldown = _env_int("ANS_COOLDOWN_SECONDS", 30)
    now = time.time()
    last = _FINGERPRINT_LAST.get(fingerprint, 0)
    if now - last < cooldown:
        existing = next((i for i in _INCIDENTS.values() if i.get("fingerprint") == fingerprint and i.get("status") == "open"), None)
        if existing:
            return existing["id"]
    _FINGERPRINT_LAST[fingerprint] = now
    inc_id = str(uuid.uuid4())[:8]
    from datetime import datetime, timezone
    _INCIDENTS[inc_id] = {
        "id": inc_id,
        "fingerprint": fingerprint,
        "check_id": check_id,
        "severity": severity,
        "message": message,
        "status": "open",
        "evidence": evidence or {},
        "suggested_heals": suggested_heals or [],
        "actions_taken": [],
        "created_at": datetime.now(timezone.utc).isoformat(),
        "resolved_at": None,
    }
    if len(_INCIDENTS) > _MAX_INCIDENTS:
        for k in list(_INCIDENTS.keys())[:-_MAX_INCIDENTS]:
            del _INCIDENTS[k]
    return inc_id


def resolve_incident(inc_id: str, actions: List[Dict[str, Any]] = None) -> None:
    from datetime import datetime, timezone
    if inc_id in _INCIDENTS:
        _INCIDENTS[inc_id]["status"] = "resolved"
        _INCIDENTS[inc_id]["resolved_at"] = datetime.now(timezone.utc).isoformat()
        if actions:
            _INCIDENTS[inc_id]["actions_taken"].extend(actions)


def add_action(inc_id: str, heal_id: str, ok: bool, message: str) -> None:
    if inc_id in _INCIDENTS:
        _INCIDENTS[inc_id]["actions_taken"].append({"heal_id": heal_id, "ok": ok, "message": message})


def get_incidents(status: str = None, limit: int = 50) -> List[Dict[str, Any]]:
    items = list(_INCIDENTS.values())
    if status:
        items = [i for i in items if i.get("status") == status]
    items.sort(key=lambda x: x.get("created_at", ""), reverse=True)
    return items[:limit]


def get_incident(inc_id: str) -> Optional[Dict[str, Any]]:
    return _INCIDENTS.get(inc_id)


def resolve_all_open(check_id: Optional[str] = None) -> int:
    """Resuelve todos los incidentes abiertos, opcionalmente filtrados por check_id. Devuelve cantidad resueltos."""
    from datetime import datetime, timezone
    resolved = 0
    for inc in list(_INCIDENTS.values()):
        if inc.get("status") != "open":
            continue
        if check_id is not None and inc.get("check_id") != check_id:
            continue
        inc["status"] = "resolved"
        inc["resolved_at"] = datetime.now(timezone.utc).isoformat()
        resolved += 1
    return resolved
