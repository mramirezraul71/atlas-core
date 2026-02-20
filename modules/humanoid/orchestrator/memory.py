"""Task memory: simple SQLite store for orchestrator context."""
from __future__ import annotations

import json
import os
import sqlite3
import threading
import uuid
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional

SCHEMA = """
CREATE TABLE IF NOT EXISTS task_memory (
    id TEXT PRIMARY KEY,
    goal TEXT NOT NULL,
    plan_json TEXT,
    execution_log_json TEXT,
    status TEXT,
    created_ts TEXT NOT NULL,
    updated_ts TEXT NOT NULL
)
"""


def _db_path() -> str:
    p = os.getenv("AUDIT_DB_PATH") or os.getenv("SCHED_DB_PATH") or "C:\\ATLAS_PUSH\\logs"
    return str(Path(p).parent / "atlas_task_memory.sqlite")


_conn: Optional[sqlite3.Connection] = None
_LOCK = threading.Lock()


def _ensure() -> sqlite3.Connection:
    global _conn
    with _LOCK:
        if _conn is not None:
            return _conn
        path = _db_path()
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        _conn = sqlite3.connect(path, check_same_thread=False)
        _conn.execute(SCHEMA)
        _conn.commit()
        return _conn


def save_task(task_id: str, goal: str, plan: Dict[str, Any], execution_log: List[Dict], status: str) -> None:
    now = datetime.now(timezone.utc).isoformat()
    conn = _ensure()
    with _LOCK:
        row = conn.execute("SELECT created_ts FROM task_memory WHERE id = ?", (task_id,)).fetchone()
        created = row[0] if row else now
        conn.execute(
            """INSERT OR REPLACE INTO task_memory (id, goal, plan_json, execution_log_json, status, created_ts, updated_ts)
               VALUES (?, ?, ?, ?, ?, ?, ?)""",
            (task_id, goal, json.dumps(plan), json.dumps(execution_log), status, created, now),
        )
        conn.commit()


def load_task(task_id: str) -> Optional[Dict[str, Any]]:
    conn = _ensure()
    with _LOCK:
        row = conn.execute("SELECT id, goal, plan_json, execution_log_json, status FROM task_memory WHERE id = ?", (task_id,)).fetchone()
    if not row:
        return None
    return {
        "task_id": row[0],
        "goal": row[1],
        "plan": json.loads(row[2]) if row[2] else {},
        "execution_log": json.loads(row[3]) if row[3] else [],
        "status": row[4],
    }


def new_task_id() -> str:
    return str(uuid.uuid4())
