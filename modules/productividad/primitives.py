from __future__ import annotations

import json
import os
import sqlite3
import time
from pathlib import Path
from typing import Any, Dict, Optional


def _db_path() -> Path:
    return Path((os.getenv("PRODUCTIVITY_DB_PATH") or r"C:\ATLAS_PUSH\logs\productivity.sqlite").strip())


def _ensure() -> sqlite3.Connection:
    p = _db_path()
    p.parent.mkdir(parents=True, exist_ok=True)
    conn = sqlite3.connect(str(p))
    conn.execute(
        "CREATE TABLE IF NOT EXISTS events (id INTEGER PRIMARY KEY AUTOINCREMENT, ts REAL, title TEXT, when_text TEXT, desc TEXT)"
    )
    conn.commit()
    return conn


def schedule_event(title: str, time_text: str, desc: str = "") -> Dict[str, Any]:
    """
    Crea un recordatorio/evento (offline-first) en SQLite local.
    La sincronización con Google Calendar (si se desea) se puede agregar después.
    """
    if not (title or "").strip():
        return {"ok": False, "error": "missing_title"}
    conn = _ensure()
    cur = conn.execute("INSERT INTO events (ts, title, when_text, desc) VALUES (?,?,?,?)", (time.time(), title[:200], time_text[:200], desc[:1000]))
    conn.commit()
    eid = int(cur.lastrowid)
    try:
        from modules.humanoid.comms.ops_bus import emit
        emit("productividad", f"Recordatorio creado: {title}", level="info", data={"event_id": eid, "when": time_text})
    except Exception:
        pass
    return {"ok": True, "event_id": eid, "title": title, "time": time_text, "desc": desc}


def check_rauli_inventory(camera_id: str = "", *, use_llm_vision: bool = False) -> Dict[str, Any]:
    """
    Análisis visual del stock (best-effort):
    - Si se provee camera_id, usa `eye=ubiq:<camera_id>`
    - Captura WorldState + OCR + métricas de calidad.
    """
    try:
        from modules.humanoid.vision.world_state import capture_world_state

        eye = f"ubiq:{camera_id}" if (camera_id or "").strip() else None
        ws = capture_world_state(eye=eye, include_ocr_items=False, use_llm_vision=bool(use_llm_vision))
        return {"ok": bool(ws.get("ok")), "world_state": ws}
    except Exception as e:
        return {"ok": False, "error": str(e)[:200]}


def digest_notifications() -> Dict[str, Any]:
    """
    Digest de notificaciones (placeholder seguro).
    Para Gmail/Inbox real requiere credenciales y conectividad; se implementa luego.
    """
    return {"ok": False, "error": "digest_not_configured"}


def alert_user(priority: str, message: str) -> Dict[str, Any]:
    """
    Enviar alerta crítica al usuario (teléfono vía Telegram/Audio).
    """
    lvl = (priority or "info").strip().lower()
    lvl = {"low": "low", "med": "med", "medium": "med", "high": "high", "critical": "critical"}.get(lvl, "info")
    try:
        from modules.humanoid.comms.ops_bus import emit
        emit("alerta", message, level=lvl)
        return {"ok": True}
    except Exception as e:
        return {"ok": False, "error": str(e)[:200]}

