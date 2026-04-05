"""Observabilidad del router de IA: bitácora + Telegram + estado de degradación."""
from __future__ import annotations

import asyncio
import json
import os
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, Optional

from modules.humanoid.ans.evolution_bitacora import append_evolution_log
from modules.humanoid.notify import send_telegram

REPO_ROOT = Path(__file__).resolve().parent.parent.parent.parent
STATE_PATH = REPO_ROOT / "state" / "atlas_ai_router_state.json"


def _now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def _dedupe_window_sec() -> float:
    try:
        return float(os.getenv("ATLAS_AI_ROUTER_EVENT_DEDUP_SEC", "300") or "300")
    except Exception:
        return 300.0


def _load_state() -> Dict[str, Any]:
    if not STATE_PATH.is_file():
        return {"degraded_routes": {}, "last_events": {}, "updated_at": None}
    try:
        return json.loads(STATE_PATH.read_text(encoding="utf-8"))
    except Exception:
        return {"degraded_routes": {}, "last_events": {}, "updated_at": None}


def _save_state(state: Dict[str, Any]) -> None:
    try:
        STATE_PATH.parent.mkdir(parents=True, exist_ok=True)
        state["updated_at"] = _now_iso()
        STATE_PATH.write_text(
            json.dumps(state, indent=2, ensure_ascii=False),
            encoding="utf-8",
        )
    except Exception:
        pass


def _bitacora(event: str, message: str, ok: bool) -> None:
    append_evolution_log(
        message=f"[AI_ROUTER:{event}] {message}",
        ok=ok,
        source="ai_router",
    )


async def _send_tg(message: str) -> bool:
    return await send_telegram(message)


def _telegram(message: str, severity: str, *, force: bool = False) -> bool:
    sev = (severity or "info").lower()
    if not force and sev not in {"warning", "critical", "emergency"}:
        return False
    try:
        return bool(asyncio.run(_send_tg(message)))
    except Exception:
        return False


def _should_emit(state: Dict[str, Any], key: str, signature: str) -> bool:
    last_events = state.setdefault("last_events", {})
    now = time.time()
    info = last_events.get(key) or {}
    last_sig = str(info.get("signature") or "")
    last_ts = float(info.get("ts") or 0.0)
    if last_sig == signature and (now - last_ts) < _dedupe_window_sec():
        return False
    last_events[key] = {"signature": signature, "ts": now}
    return True


def note_route_degraded(
    *,
    route: str,
    primary_model: str,
    selected_model: str,
    reason: str,
    last_error: str = "",
    severity: str = "warning",
) -> Dict[str, Any]:
    state = _load_state()
    degraded_routes = state.setdefault("degraded_routes", {})
    route_key = (route or "UNKNOWN").upper()
    signature = "|".join(
        [
            route_key,
            primary_model or "",
            selected_model or "",
            reason or "",
            (last_error or "")[:140],
        ]
    )
    if _should_emit(state, f"degraded:{route_key}", signature):
        msg = (
            f"Ruta {route_key} degradada. Primario={primary_model or 'n/a'} "
            f"seleccionado={selected_model or 'null'} razón={reason or 'unknown'}"
        )
        if last_error:
            msg += f" error={last_error[:180]}"
        _bitacora("role_degraded", msg, ok=False)
        _telegram(f"ATLAS AI WARNING\n{msg}", severity)
    degraded_routes[route_key] = {
        "route": route_key,
        "primary_model": primary_model,
        "selected_model": selected_model,
        "reason": reason,
        "last_error": last_error[:500],
        "degraded_at": _now_iso(),
    }
    _save_state(state)
    return degraded_routes[route_key]


def note_route_restored(
    *,
    route: str,
    restored_model: str,
) -> Optional[Dict[str, Any]]:
    state = _load_state()
    degraded_routes = state.setdefault("degraded_routes", {})
    route_key = (route or "UNKNOWN").upper()
    previous = degraded_routes.pop(route_key, None)
    if previous:
        signature = f"{route_key}|{restored_model}"
        if _should_emit(state, f"restored:{route_key}", signature):
            msg = (
                f"Ruta {route_key} restaurada. "
                f"Modelo activo={restored_model or 'n/a'}"
            )
            _bitacora("role_restored", msg, ok=True)
        _save_state(state)
    return previous


def note_route_fallback(
    *,
    route: str,
    primary_model: str,
    fallback_model: str,
    reason: str,
    last_error: str = "",
) -> None:
    state = _load_state()
    route_key = (route or "UNKNOWN").upper()
    signature = "|".join(
        [
            route_key,
            primary_model or "",
            fallback_model or "",
            reason or "",
            (last_error or "")[:140],
        ]
    )
    if _should_emit(state, f"fallback:{route_key}", signature):
        msg = (
            f"Fallback en ruta {route_key}. Primario={primary_model or 'n/a'} "
            f"fallback={fallback_model or 'n/a'} razón={reason or 'unknown'}"
        )
        if last_error:
            msg += f" error={last_error[:180]}"
        _bitacora("router_fallback_triggered", msg, ok=False)
        _telegram(f"ATLAS AI WARNING\n{msg}", "warning")
        _save_state(state)
