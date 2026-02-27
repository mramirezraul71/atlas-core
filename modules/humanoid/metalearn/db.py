"""SQLite persistence for meta-learning: events, stats, rules, snapshots."""
from __future__ import annotations

import json
import os
from pathlib import Path
from typing import Any, Dict, List, Optional

import sqlite3

SCHEMA_EVENTS = """
CREATE TABLE IF NOT EXISTS feedback_events (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    ts TEXT NOT NULL,
    action_type TEXT NOT NULL,
    risk_level TEXT NOT NULL,
    decision TEXT NOT NULL,
    outcome TEXT NOT NULL,
    latency_ms INTEGER,
    node_id TEXT,
    model_used TEXT,
    features_json TEXT,
    correlation_id TEXT,
    thread_id TEXT,
    task_id TEXT,
    source TEXT NOT NULL DEFAULT 'unknown',
    event_hash TEXT
);
CREATE INDEX IF NOT EXISTS idx_feedback_ts ON feedback_events(ts);
CREATE INDEX IF NOT EXISTS idx_feedback_action ON feedback_events(action_type);
CREATE INDEX IF NOT EXISTS idx_feedback_source ON feedback_events(source);
"""

SCHEMA_STATS = """
CREATE TABLE IF NOT EXISTS learn_stats (
    bucket_key TEXT PRIMARY KEY,
    approve_count REAL NOT NULL DEFAULT 0,
    reject_count REAL NOT NULL DEFAULT 0,
    success_count REAL NOT NULL DEFAULT 0,
    fail_count REAL NOT NULL DEFAULT 0,
    total_latency_ms REAL NOT NULL DEFAULT 0,
    sample_count INTEGER NOT NULL DEFAULT 0,
    updated_ts TEXT NOT NULL
);
"""

SCHEMA_RULES = """
CREATE TABLE IF NOT EXISTS learned_rules (
    id TEXT PRIMARY KEY,
    conditions_json TEXT NOT NULL,
    risk_adjust REAL NOT NULL DEFAULT 0,
    router_hint TEXT,
    canary_hint REAL,
    approve_rate REAL NOT NULL DEFAULT 0,
    success_rate REAL NOT NULL DEFAULT 0,
    sample_count INTEGER NOT NULL DEFAULT 0,
    created_ts TEXT NOT NULL,
    updated_ts TEXT NOT NULL
);
"""

SCHEMA_SNAPSHOTS = """
CREATE TABLE IF NOT EXISTS tune_snapshots (
    id TEXT PRIMARY KEY,
    params_json TEXT NOT NULL,
    created_ts TEXT NOT NULL,
    comment TEXT
);
"""


def _db_path() -> str:
    p = os.getenv("METALEARN_DB_PATH", "C:\\ATLAS_PUSH\\logs\\atlas_metalearn.sqlite")
    return str(Path(p).resolve())


_conn: Optional[sqlite3.Connection] = None


def _ensure() -> sqlite3.Connection:
    global _conn
    if _conn is not None:
        return _conn
    path = _db_path()
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    _conn = sqlite3.connect(path)
    _conn.executescript(SCHEMA_EVENTS)
    _conn.executescript(SCHEMA_STATS)
    _conn.executescript(SCHEMA_RULES)
    _conn.executescript(SCHEMA_SNAPSHOTS)
    _conn.commit()
    return _conn


def insert_event(
    ts: str,
    action_type: str,
    risk_level: str,
    decision: str,
    outcome: str,
    latency_ms: Optional[int] = None,
    node_id: Optional[str] = None,
    model_used: Optional[str] = None,
    features_json: Optional[Dict[str, Any]] = None,
    correlation_id: Optional[str] = None,
    thread_id: Optional[str] = None,
    task_id: Optional[str] = None,
    source: str = "unknown",
) -> int:
    """Insert feedback event. Returns row id. event_hash for audit (no PII)."""
    import hashlib
    payload = f"{ts}|{action_type}|{risk_level}|{decision}|{outcome}|{source}"
    event_hash = hashlib.sha256(payload.encode()).hexdigest()[:16]
    conn = _ensure()
    cur = conn.execute(
        """INSERT INTO feedback_events (ts, action_type, risk_level, decision, outcome, latency_ms, node_id, model_used, features_json, correlation_id, thread_id, task_id, source, event_hash)
           VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)""",
        (
            ts,
            action_type,
            risk_level,
            decision,
            outcome,
            latency_ms,
            node_id,
            model_used,
            json.dumps(features_json) if features_json else None,
            correlation_id,
            thread_id,
            task_id,
            source,
            event_hash,
        ),
    )
    conn.commit()
    return cur.lastrowid or 0


def get_events_since(ts: str, limit: int = 5000) -> List[Dict[str, Any]]:
    """Events with ts >= ts, for trainer."""
    conn = _ensure()
    rows = conn.execute(
        "SELECT ts, action_type, risk_level, decision, outcome, latency_ms, node_id, model_used, features_json, source FROM feedback_events WHERE ts >= ? ORDER BY ts LIMIT ?",
        (ts, limit),
    ).fetchall()
    out = []
    for r in rows:
        out.append({
            "ts": r[0],
            "action_type": r[1],
            "risk_level": r[2],
            "decision": r[3],
            "outcome": r[4],
            "latency_ms": r[5],
            "node_id": r[6],
            "model_used": r[7],
            "features_json": json.loads(r[8]) if r[8] else None,
            "source": r[9],
        })
    return out


def upsert_stat(bucket_key: str, approve_delta: float, reject_delta: float, success_delta: float, fail_delta: float, latency_delta: float, updated_ts: str, sample_delta: int = 1) -> None:
    """Update decayed stats for a bucket."""
    conn = _ensure()
    row = conn.execute("SELECT approve_count, reject_count, success_count, fail_count, total_latency_ms, sample_count FROM learn_stats WHERE bucket_key = ?", (bucket_key,)).fetchone()
    if row:
        conn.execute(
            """UPDATE learn_stats SET approve_count = approve_count + ?, reject_count = reject_count + ?, success_count = success_count + ?, fail_count = fail_count + ?,
               total_latency_ms = total_latency_ms + ?, sample_count = sample_count + ?, updated_ts = ? WHERE bucket_key = ?""",
            (approve_delta, reject_delta, success_delta, fail_delta, latency_delta, sample_delta, updated_ts, bucket_key),
        )
    else:
        conn.execute(
            """INSERT INTO learn_stats (bucket_key, approve_count, reject_count, success_count, fail_count, total_latency_ms, sample_count, updated_ts)
               VALUES (?, ?, ?, ?, ?, ?, ?, ?)""",
            (bucket_key, approve_delta, reject_delta, success_delta, fail_delta, latency_delta, sample_delta, updated_ts),
        )
    conn.commit()


def get_all_stats() -> List[Dict[str, Any]]:
    conn = _ensure()
    rows = conn.execute("SELECT bucket_key, approve_count, reject_count, success_count, fail_count, total_latency_ms, sample_count, updated_ts FROM learn_stats").fetchall()
    return [
        {
            "bucket_key": r[0],
            "approve_count": r[1],
            "reject_count": r[2],
            "success_count": r[3],
            "fail_count": r[4],
            "total_latency_ms": r[5],
            "sample_count": r[6],
            "updated_ts": r[7],
        }
        for r in rows
    ]


def save_rules(rules: List[Dict[str, Any]], updated_ts: str) -> None:
    """Replace learned rules (up to MAX_RULES)."""
    conn = _ensure()
    conn.execute("DELETE FROM learned_rules")
    for r in rules[: int(os.getenv("METALEARN_MAX_RULES", "100") or 100)]:
        conn.execute(
            """INSERT INTO learned_rules (id, conditions_json, risk_adjust, router_hint, canary_hint, approve_rate, success_rate, sample_count, created_ts, updated_ts)
               VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)""",
            (
                r.get("id", ""),
                json.dumps(r.get("conditions", {})),
                r.get("risk_adjust", 0),
                r.get("router_hint"),
                r.get("canary_hint"),
                r.get("approve_rate", 0),
                r.get("success_rate", 0),
                r.get("sample_count", 0),
                r.get("created_ts", updated_ts),
                updated_ts,
            ),
        )
    conn.commit()


def get_rules(limit: int = 100) -> List[Dict[str, Any]]:
    conn = _ensure()
    rows = conn.execute(
        "SELECT id, conditions_json, risk_adjust, router_hint, canary_hint, approve_rate, success_rate, sample_count, created_ts FROM learned_rules ORDER BY sample_count DESC LIMIT ?",
        (limit,),
    ).fetchall()
    return [
        {
            "id": r[0],
            "conditions": json.loads(r[1]) if r[1] else {},
            "risk_adjust": r[2],
            "router_hint": r[3],
            "canary_hint": r[4],
            "approve_rate": r[5],
            "success_rate": r[6],
            "sample_count": r[7],
            "created_ts": r[8],
        }
        for r in rows
    ]


def save_snapshot(snapshot_id: str, params: Dict[str, Any], created_ts: str, comment: Optional[str] = None) -> None:
    conn = _ensure()
    conn.execute("INSERT INTO tune_snapshots (id, params_json, created_ts, comment) VALUES (?, ?, ?, ?)", (snapshot_id, json.dumps(params), created_ts, comment or ""))
    conn.commit()


def get_snapshot(snapshot_id: str) -> Optional[Dict[str, Any]]:
    conn = _ensure()
    row = conn.execute("SELECT params_json, created_ts, comment FROM tune_snapshots WHERE id = ?", (snapshot_id,)).fetchone()
    if not row:
        return None
    return {"params": json.loads(row[0]), "created_ts": row[1], "comment": row[2]}


def list_snapshots(limit: int = 20) -> List[Dict[str, Any]]:
    conn = _ensure()
    rows = conn.execute("SELECT id, created_ts, comment FROM tune_snapshots ORDER BY created_ts DESC LIMIT ?", (limit,)).fetchall()
    return [{"id": r[0], "created_ts": r[1], "comment": r[2]} for r in rows]


def event_count() -> int:
    conn = _ensure()
    row = conn.execute("SELECT COUNT(*) FROM feedback_events").fetchone()
    return row[0] if row else 0
