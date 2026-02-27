"""Persist governance mode + emergency_stop in SQLite + runtime cache.
Integrado con ATLAS AUTONOMOUS: SurvivalMode (emergency_stop), PerformanceOptimizer (growth/governed).
"""
from __future__ import annotations

import logging
import os
import sys
import sqlite3
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional

logger = logging.getLogger(__name__)

# ATLAS AUTONOMOUS: lazy singletons (opcional)
_survival = None
_optimizer = None


def _get_survival():
    global _survival
    if _survival is not None:
        return _survival
    try:
        base = Path(__file__).resolve().parent.parent.parent
        if str(base) not in sys.path:
            sys.path.insert(0, str(base))
        from autonomous.resilience.survival_mode import SurvivalMode
        _survival = SurvivalMode()
        return _survival
    except Exception as e:
        logger.debug("SurvivalMode no disponible: %s", e)
        return None


def _get_optimizer():
    global _optimizer
    if _optimizer is not None:
        return _optimizer
    try:
        base = Path(__file__).resolve().parent.parent.parent
        if str(base) not in sys.path:
            sys.path.insert(0, str(base))
        from autonomous.learning.performance_optimizer import PerformanceOptimizer
        _optimizer = PerformanceOptimizer()
        return _optimizer
    except Exception as e:
        logger.debug("PerformanceOptimizer no disponible: %s", e)
        return None

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
_db_failed: bool = False  # Si True, no intentar DB y usar solo _cache


def _ensure() -> Optional[sqlite3.Connection]:
    global _conn, _db_failed
    if _db_failed:
        return None
    if _conn is not None:
        return _conn
    try:
        parent = Path(_DB_PATH).resolve().parent
        parent.mkdir(parents=True, exist_ok=True)
        _conn = sqlite3.connect(_DB_PATH, timeout=5)
        _conn.executescript(_SCHEMA)
        _conn.commit()
        _load_from_db(_conn)
        return _conn
    except Exception:
        _conn = None
        _db_failed = True
        return None


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
    """growth | governed. Nunca lanza; si la DB falla usa _cache/env."""
    c = _ensure()
    if c is not None:
        pass  # _load_from_db ya actualizó _cache en _ensure
    m = _cache.get("mode") or _env("GOVERNANCE_MODE", "governed")
    if m in ("growth", "governed"):
        return m
    return "governed"


def get_emergency_stop() -> bool:
    """Nunca lanza; si la DB falla usa _cache/env."""
    _ensure()
    v = _cache.get("emergency_stop") or _env("EMERGENCY_STOP", "false")
    return v in ("1", "true", "yes", "on")


def set_mode(mode: str, reason: str = "", actor: str = "api") -> bool:
    """Set mode (growth|governed). Returns True on success. Integra SurvivalMode y PerformanceOptimizer."""
    mode = (mode or "").strip().lower()
    if mode not in ("growth", "governed"):
        return False
    prev = get_mode()
    # ATLAS AUTONOMOUS: growth → auto-optimizaciones sin aprobación; governed → requieren aprobación
    opt = _get_optimizer()
    if opt is not None:
        if mode == "growth":
            opt.approval_required = False
            logger.info("[GOVERNANCE] Modo Growth → Auto-optimizaciones habilitadas")
        else:
            opt.approval_required = True
            logger.info("[GOVERNANCE] Modo Governed → Optimizaciones requieren aprobación")
    if prev == mode:
        return True
    c = _ensure()
    if c is None:
        _cache["mode"] = mode
        return True
    try:
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
        _cache["mode"] = mode
        return True


def set_emergency_stop(enable: bool, reason: str = "", actor: str = "api") -> bool:
    """Set emergency stop. Returns True on success. Integra SurvivalMode (emergency → enter, off → exit)."""
    prev = get_emergency_stop()
    val = "true" if enable else "false"
    surv = _get_survival()
    if enable:
        if surv is not None:
            surv.enter_survival_mode(reason or "emergency_stop activado por usuario")
            logger.critical("[GOVERNANCE] Modo Emergency Stop → Survival Mode activado")
    else:
        if surv is not None and surv.is_in_survival():
            surv.exit_survival_mode()
    if prev == enable:
        return True
    c = _ensure()
    if c is None:
        _cache["emergency_stop"] = val
        return True
    try:
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
        _cache["emergency_stop"] = val
        return True


def get_log(limit: int = 20) -> list:
    c = _ensure()
    if c is None:
        return []
    rows = c.execute(
        "SELECT action, from_val, to_val, reason, actor, created_ts FROM governance_log ORDER BY id DESC LIMIT ?",
        (limit,),
    ).fetchall()
    return [{"action": r[0], "from_val": r[1], "to_val": r[2], "reason": r[3], "actor": r[4], "created_ts": r[5]} for r in rows]
