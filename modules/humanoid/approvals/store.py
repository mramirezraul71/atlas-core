"""SQLite store for approval queue."""
from __future__ import annotations

import json
import os
import sqlite3
import uuid
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional

SCHEMA = """
CREATE TABLE IF NOT EXISTS approvals (
    id TEXT PRIMARY KEY,
    created_ts TEXT NOT NULL,
    action TEXT NOT NULL,
    payload_json TEXT,
    risk TEXT NOT NULL DEFAULT 'medium',
    status TEXT NOT NULL DEFAULT 'pending',
    job_id TEXT,
    run_id INTEGER,
    resolved_ts TEXT,
    resolved_by TEXT
);
CREATE INDEX IF NOT EXISTS idx_approvals_status ON approvals(status);
CREATE INDEX IF NOT EXISTS idx_approvals_created ON approvals(created_ts);
"""


def _db_path() -> str:
    p = os.getenv("ATLAS_APPROVALS_DB_PATH") or os.getenv("ATLAS_MEMORY_DB_PATH")
    if p:
        return str(Path(p).parent / "atlas_approvals.sqlite")
    return str(Path(os.getcwd()) / "logs" / "atlas_approvals.sqlite")


_conn: Optional[sqlite3.Connection] = None


def _ensure() -> sqlite3.Connection:
    global _conn
    if _conn is not None:
        return _conn
    path = _db_path()
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    _conn = sqlite3.connect(path)
    _conn.executescript(SCHEMA)
    _conn.commit()
    return _conn


def _now() -> str:
    return datetime.now(timezone.utc).isoformat()


def create(action: str, payload: Dict[str, Any], risk: str = "medium", job_id: Optional[str] = None, run_id: Optional[int] = None) -> Dict[str, Any]:
    aid = str(uuid.uuid4())[:12]
    now = _now()
    conn = _ensure()
    conn.execute(
        "INSERT INTO approvals (id, created_ts, action, payload_json, risk, status, job_id, run_id) VALUES (?, ?, ?, ?, ?, 'pending', ?, ?)",
        (aid, now, action, json.dumps(payload), risk, job_id, run_id),
    )
    conn.commit()
    return {"id": aid, "created_ts": now, "action": action, "payload": payload, "risk": risk, "status": "pending", "job_id": job_id, "run_id": run_id}


def list_items(status: Optional[str] = None, limit: int = 50) -> List[Dict[str, Any]]:
    conn = _ensure()
    if status:
        rows = conn.execute("SELECT id, created_ts, action, payload_json, risk, status, job_id, run_id, resolved_ts, resolved_by FROM approvals WHERE status = ? ORDER BY created_ts DESC LIMIT ?", (status, limit)).fetchall()
    else:
        rows = conn.execute("SELECT id, created_ts, action, payload_json, risk, status, job_id, run_id, resolved_ts, resolved_by FROM approvals ORDER BY created_ts DESC LIMIT ?", (limit,)).fetchall()
    out = []
    for r in rows:
        out.append({
            "id": r[0], "created_ts": r[1], "action": r[2],
            "payload": json.loads(r[3]) if r[3] else {},
            "risk": r[4], "status": r[5], "job_id": r[6], "run_id": r[7],
            "resolved_ts": r[8], "resolved_by": r[9],
        })
    return out


def get(aid: str) -> Optional[Dict[str, Any]]:
    conn = _ensure()
    row = conn.execute("SELECT id, created_ts, action, payload_json, risk, status, job_id, run_id, resolved_ts, resolved_by FROM approvals WHERE id = ?", (aid,)).fetchone()
    if not row:
        return None
    return {
        "id": row[0], "created_ts": row[1], "action": row[2],
        "payload": json.loads(row[3]) if row[3] else {},
        "risk": row[4], "status": row[5], "job_id": row[6], "run_id": row[7],
        "resolved_ts": row[8], "resolved_by": row[9],
    }


def approve(aid: str, resolved_by: str = "api") -> bool:
    conn = _ensure()
    now = _now()
    cur = conn.execute("UPDATE approvals SET status = 'approved', resolved_ts = ?, resolved_by = ? WHERE id = ? AND status = 'pending'", (now, resolved_by, aid))
    conn.commit()
    return cur.rowcount > 0


def reject(aid: str, resolved_by: str = "api") -> bool:
    conn = _ensure()
    now = _now()
    cur = conn.execute("UPDATE approvals SET status = 'rejected', resolved_ts = ?, resolved_by = ? WHERE id = ? AND status = 'pending'", (now, resolved_by, aid))
    conn.commit()
    return cur.rowcount > 0
