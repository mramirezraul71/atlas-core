"""Persist governance mode + emergency_stop in SQLite + runtime cache."""
from __future__ import annotations

import os
import sqlite3
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional

_DB_PATH = os.getenv("GOVERNANCE_DB_PATH", "C:\\ATLAS_PUSH\\logs\\atlas_governance.sqlite")
_SCHEMA = """
CREATE TABLE IF NOT EXISTS governance_state (
    key TEXT PRIMARY KEY,
    value TEXT NOT NULL,
    updated_ts TEXT NOT NULL
);
CREATE TABLE IF NOT EXISTS governance_log (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    action TEXT NOT NULL,
    from_val TEXT,
    to_val TEXT,
    reason TEXT,
    actor TEXT,
    created_ts TEXT NOT NULL
);
"""
_conn: Optional[sqlite3.Connection] = None
_cache: dict = {"mode": "governed", "emergency_stop": "false"}


def _ensure() -> sqlite3.Connection:
    global _conn
    if _conn is not None:
        return _conn
    Path(_DB_PATH).parent.mkdir(parents=True, exist_ok=True)
    _conn = sqlite3.connect(_DB_PATH)
    _conn.executescript(_SCHEMA)
    _conn.commit()
    _load_from_db(_conn)
    return _conn


def _load_from_db(c: sqlite3.Connection) -> None:
    global _cache
    try:
        for row in c.execute("SELECT key, value FROM governance_state").fetchall():
            _cache[row[0]] = row[1]
    except Exception:
        pass


def _ts() -> str:
    return datetime.now(timezone.utc).isoformat()


def _env(name: str, default: str) -> str:
    return (os.getenv(name) or default).strip().lower()


def get_mode() -> str:
    """growth | governed."""
    _ensure()
    m = _cache.get("mode") or _env("GOVERNANCE_MODE", "governed")
    if m in ("growth", "governed"):
        return m
    return "governed"


def get_emergency_stop() -> bool:
    _ensure()
    v = _cache.get("emergency_stop") or _env("EMERGENCY_STOP", "false")
    return v in ("1", "true", "yes", "on")


def set_mode(mode: str, reason: str = "", actor: str = "api") -> bool:
    """Set mode (growth|governed). Returns True on success."""
    mode = (mode or "").strip().lower()
    if mode not in ("growth", "governed"):
        return False
    prev = get_mode()
    if prev == mode:
        return True
    try:
        c = _ensure()
        c.execute(
            "INSERT OR REPLACE INTO governance_state (key, value, updated_ts) VALUES (?, ?, ?)",
            ("mode", mode, _ts()),
        )
        c.execute(
            "INSERT INTO governance_log (action, from_val, to_val, reason, actor, created_ts) VALUES (?, ?, ?, ?, ?, ?)",
            ("set_mode", prev, mode, reason[:500], actor, _ts()),
        )
        c.commit()
        _cache["mode"] = mode
        return True
    except Exception:
        return False


def set_emergency_stop(enable: bool, reason: str = "", actor: str = "api") -> bool:
    """Set emergency stop. Returns True on success."""
    prev = get_emergency_stop()
    val = "true" if enable else "false"
    if prev == enable:
        return True
    try:
        c = _ensure()
        c.execute(
            "INSERT OR REPLACE INTO governance_state (key, value, updated_ts) VALUES (?, ?, ?)",
            ("emergency_stop", val, _ts()),
        )
        c.execute(
            "INSERT INTO governance_log (action, from_val, to_val, reason, actor, created_ts) VALUES (?, ?, ?, ?, ?, ?)",
            ("emergency_stop", "true" if prev else "false", val, reason[:500], actor, _ts()),
        )
        c.commit()
        _cache["emergency_stop"] = val
        return True
    except Exception:
        return False


def get_log(limit: int = 20) -> list:
    _ensure()
    rows = _conn.execute(
        "SELECT action, from_val, to_val, reason, actor, created_ts FROM governance_log ORDER BY id DESC LIMIT ?",
        (limit,),
    ).fetchall()
    return [{"action": r[0], "from_val": r[1], "to_val": r[2], "reason": r[3], "actor": r[4], "created_ts": r[5]} for r in rows]
