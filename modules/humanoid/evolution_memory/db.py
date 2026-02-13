"""Evolution Memory DB: evolution_history, performance_metrics, model_scores, incident_history, refactor_log, dependency_changes."""
from __future__ import annotations

import json
import os
import sqlite3
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional

_DB_PATH = os.getenv("EVOLUTION_MEMORY_DB_PATH", "C:\\ATLAS_PUSH\\logs\\atlas_evolution.sqlite")

_SCHEMA = """
CREATE TABLE IF NOT EXISTS evolution_history (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    kind TEXT NOT NULL,
    goal TEXT,
    spec_json TEXT,
    outcome TEXT,
    ok INTEGER NOT NULL,
    ms INTEGER,
    created_ts TEXT NOT NULL
);
CREATE TABLE IF NOT EXISTS performance_metrics (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    source TEXT,
    metric_name TEXT,
    value_real REAL,
    value_int INTEGER,
    created_ts TEXT NOT NULL
);
CREATE TABLE IF NOT EXISTS model_scores (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    provider_id TEXT NOT NULL,
    model_key TEXT NOT NULL,
    failure_count INTEGER DEFAULT 0,
    success_count INTEGER DEFAULT 0,
    last_latency_ms REAL,
    last_error TEXT,
    updated_ts TEXT NOT NULL
);
CREATE TABLE IF NOT EXISTS incident_history (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    fingerprint TEXT,
    action TEXT,
    ok INTEGER NOT NULL,
    rollback INTEGER DEFAULT 0,
    created_ts TEXT NOT NULL
);
CREATE TABLE IF NOT EXISTS refactor_log (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    target_path TEXT,
    kind TEXT,
    ok INTEGER NOT NULL,
    rollback INTEGER DEFAULT 0,
    created_ts TEXT NOT NULL
);
CREATE TABLE IF NOT EXISTS dependency_changes (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    package TEXT NOT NULL,
    action TEXT NOT NULL,
    ok INTEGER NOT NULL,
    created_ts TEXT NOT NULL
);
CREATE INDEX IF NOT EXISTS idx_evolution_kind ON evolution_history(kind);
CREATE INDEX IF NOT EXISTS idx_model_scores_provider ON model_scores(provider_id, model_key);
CREATE INDEX IF NOT EXISTS idx_incident_fingerprint ON incident_history(fingerprint);
"""

_conn: Optional[sqlite3.Connection] = None


def _ensure() -> sqlite3.Connection:
    global _conn
    if _conn is not None:
        return _conn
    Path(_DB_PATH).parent.mkdir(parents=True, exist_ok=True)
    _conn = sqlite3.connect(_DB_PATH)
    _conn.executescript(_SCHEMA)
    return _conn


def _ts() -> str:
    return datetime.now(timezone.utc).isoformat()


def record_model_failure(provider_id: str, model_key: str, error: str = "", latency_ms: float = 0) -> None:
    c = _ensure()
    cur = c.execute(
        "SELECT id, failure_count, success_count, last_error FROM model_scores WHERE provider_id=? AND model_key=?",
        (provider_id, model_key),
    )
    row = cur.fetchone()
    if row:
        c.execute(
            "UPDATE model_scores SET failure_count=failure_count+1, last_latency_ms=?, last_error=?, updated_ts=? WHERE id=?",
            (latency_ms, error[:500], _ts(), row[0]),
        )
    else:
        c.execute(
            "INSERT INTO model_scores (provider_id, model_key, failure_count, success_count, last_latency_ms, last_error, updated_ts) VALUES (?,?,1,0,?,?,?)",
            (provider_id, model_key, latency_ms, error[:500], _ts()),
        )
    c.commit()


def record_model_success(provider_id: str, model_key: str, latency_ms: float = 0) -> None:
    c = _ensure()
    cur = c.execute(
        "SELECT id FROM model_scores WHERE provider_id=? AND model_key=?",
        (provider_id, model_key),
    )
    row = cur.fetchone()
    if row:
        c.execute(
            "UPDATE model_scores SET success_count=success_count+1, last_latency_ms=?, last_error=NULL, updated_ts=? WHERE id=?",
            (latency_ms, _ts(), row[0]),
        )
    else:
        c.execute(
            "INSERT INTO model_scores (provider_id, model_key, failure_count, success_count, last_latency_ms, last_error, updated_ts) VALUES (?,?,0,1,?,NULL,?)",
            (provider_id, model_key, latency_ms, _ts()),
        )
    c.commit()


def record_evolution(kind: str, goal: str = "", spec_json: Optional[Dict] = None, outcome: str = "", ok: bool = True, ms: int = 0) -> None:
    c = _ensure()
    c.execute(
        "INSERT INTO evolution_history (kind, goal, spec_json, outcome, ok, ms, created_ts) VALUES (?,?,?,?,?,?,?)",
        (kind, goal, json.dumps(spec_json or {}), outcome, 1 if ok else 0, ms, _ts()),
    )
    c.commit()


def record_performance(source: str, metric_name: str, value_real: float = 0, value_int: int = 0) -> None:
    c = _ensure()
    c.execute(
        "INSERT INTO performance_metrics (source, metric_name, value_real, value_int, created_ts) VALUES (?,?,?,?,?)",
        (source, metric_name, value_real, value_int, _ts()),
    )
    c.commit()


def record_incident(fingerprint: str, action: str, ok: bool, rollback: bool = False) -> None:
    c = _ensure()
    c.execute(
        "INSERT INTO incident_history (fingerprint, action, ok, rollback, created_ts) VALUES (?,?,?,?,?)",
        (fingerprint, action, 1 if ok else 0, 1 if rollback else 0, _ts()),
    )
    c.commit()


def record_refactor(target_path: str, kind: str, ok: bool, rollback: bool = False) -> None:
    c = _ensure()
    c.execute(
        "INSERT INTO refactor_log (target_path, kind, ok, rollback, created_ts) VALUES (?,?,?,?,?)",
        (target_path, kind, 1 if ok else 0, 1 if rollback else 0, _ts()),
    )
    c.commit()


def record_dep_change(package: str, action: str, ok: bool) -> None:
    c = _ensure()
    c.execute(
        "INSERT INTO dependency_changes (package, action, ok, created_ts) VALUES (?,?,?,?)",
        (package, action, 1 if ok else 0, _ts()),
    )
    c.commit()


def get_evolution_history(limit: int = 50) -> List[Dict[str, Any]]:
    c = _ensure()
    rows = c.execute(
        "SELECT kind, goal, outcome, ok, ms, created_ts FROM evolution_history ORDER BY id DESC LIMIT ?",
        (limit,),
    ).fetchall()
    return [{"kind": r[0], "goal": r[1], "outcome": r[2], "ok": bool(r[3]), "ms": r[4], "created_ts": r[5]} for r in rows]


def get_model_scores() -> List[Dict[str, Any]]:
    c = _ensure()
    rows = c.execute(
        "SELECT provider_id, model_key, failure_count, success_count, last_latency_ms, last_error, updated_ts FROM model_scores ORDER BY updated_ts DESC"
    ).fetchall()
    return [
        {
            "provider_id": r[0], "model_key": r[1],
            "failure_count": r[2], "success_count": r[3],
            "last_latency_ms": r[4], "last_error": r[5], "updated_ts": r[6],
        }
        for r in rows
    ]


def get_incident_history(limit: int = 50) -> List[Dict[str, Any]]:
    c = _ensure()
    rows = c.execute(
        "SELECT fingerprint, action, ok, rollback, created_ts FROM incident_history ORDER BY id DESC LIMIT ?",
        (limit,),
    ).fetchall()
    return [{"fingerprint": r[0], "action": r[1], "ok": bool(r[2]), "rollback": bool(r[3]), "created_ts": r[4]} for r in rows]
