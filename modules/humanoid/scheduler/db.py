"""Scheduler SQLite persistence: jobs, job_runs."""
from __future__ import annotations

import json
import os
import sqlite3
import uuid
from datetime import datetime, timezone
from pathlib import Path
from typing import TYPE_CHECKING, Any, Dict, List, Optional

if TYPE_CHECKING:
    from .models import JobSpec

JOBS_SCHEMA = """
CREATE TABLE IF NOT EXISTS jobs (
    id TEXT PRIMARY KEY,
    name TEXT NOT NULL,
    kind TEXT NOT NULL,
    payload_json TEXT,
    cron TEXT,
    run_at TEXT,
    interval_seconds INTEGER,
    enabled INTEGER NOT NULL DEFAULT 1,
    status TEXT NOT NULL DEFAULT 'queued',
    retries INTEGER NOT NULL DEFAULT 0,
    max_retries INTEGER NOT NULL DEFAULT 3,
    backoff_seconds INTEGER NOT NULL DEFAULT 5,
    last_run_ts TEXT,
    next_run_ts TEXT,
    last_error TEXT,
    created_ts TEXT NOT NULL,
    updated_ts TEXT NOT NULL
)
"""

RUNS_SCHEMA = """
CREATE TABLE IF NOT EXISTS job_runs (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    job_id TEXT NOT NULL,
    ts_start TEXT NOT NULL,
    ts_end TEXT,
    ok INTEGER NOT NULL,
    ms INTEGER,
    result_json TEXT,
    error TEXT,
    FOREIGN KEY (job_id) REFERENCES jobs(id)
)
"""


class SchedulerDB:
    def __init__(self, path: Optional[str] = None) -> None:
        self._path = path or os.getenv("SCHED_DB_PATH")
        self._connection: Optional[sqlite3.Connection] = None

    def _ensure(self) -> sqlite3.Connection:
        if self._connection is not None:
            return self._connection
        p = self._path or os.getenv("SCHED_DB_PATH")
        if not p:
            raise RuntimeError("SCHED_DB_PATH not set")
        Path(p).parent.mkdir(parents=True, exist_ok=True)
        self._connection = sqlite3.connect(p)
        self._connection.execute(JOBS_SCHEMA)
        self._connection.execute(RUNS_SCHEMA)
        self._connection.commit()
        return self._connection

    def create_job(self, spec: "JobSpec") -> Dict[str, Any]:
        now = datetime.now(timezone.utc).isoformat()
        jid = str(uuid.uuid4())
        run_at = spec.run_at
        next_run = run_at or now
        conn = self._ensure()
        conn.execute(
            """INSERT INTO jobs (id, name, kind, payload_json, cron, run_at, interval_seconds, enabled, status,
               retries, max_retries, backoff_seconds, last_run_ts, next_run_ts, last_error, created_ts, updated_ts)
               VALUES (?, ?, ?, ?, ?, ?, ?, 1, 'queued', 0, ?, ?, NULL, ?, NULL, ?, ?)""",
            (jid, spec.name, spec.kind, json.dumps(spec.payload or {}), None, spec.run_at, spec.interval_seconds,
             spec.max_retries, spec.backoff_seconds, next_run, now, now),
        )
        conn.commit()
        return self.get_job(jid)

    def get_job(self, job_id: str) -> Optional[Dict[str, Any]]:
        conn = self._ensure()
        row = conn.execute("SELECT * FROM jobs WHERE id = ?", (job_id,)).fetchone()
        if not row:
            return None
        return self._row_to_job(row)

    def list_jobs(self, status: Optional[str] = None) -> List[Dict[str, Any]]:
        conn = self._ensure()
        if status:
            rows = conn.execute("SELECT * FROM jobs WHERE status = ? ORDER BY next_run_ts", (status,)).fetchall()
        else:
            rows = conn.execute("SELECT * FROM jobs ORDER BY next_run_ts").fetchall()
        return [self._row_to_job(r) for r in rows]

    def due_jobs(self, now_iso: str, limit: int) -> List[Dict[str, Any]]:
        conn = self._ensure()
        rows = conn.execute(
            """SELECT * FROM jobs WHERE enabled = 1 AND status IN ('queued', 'failed')
               AND next_run_ts <= ? ORDER BY next_run_ts LIMIT ?""",
            (now_iso, limit),
        ).fetchall()
        return [self._row_to_job(r) for r in rows]

    def set_running(self, job_id: str) -> None:
        now = datetime.now(timezone.utc).isoformat()
        self._ensure().execute("UPDATE jobs SET status = 'running', updated_ts = ? WHERE id = ?", (now, job_id))
        self._ensure().commit()

    def set_finished(self, job_id: str, ok: bool, last_error: Optional[str], next_run_ts: Optional[str], retries: int) -> None:
        now = datetime.now(timezone.utc).isoformat()
        status = "success" if ok else "failed"
        conn = self._ensure()
        conn.execute(
            "UPDATE jobs SET status = ?, last_run_ts = ?, next_run_ts = ?, last_error = ?, retries = ?, updated_ts = ? WHERE id = ?",
            (status, now, next_run_ts, last_error, retries, now, job_id),
        )
        conn.commit()

    def set_paused(self, job_id: str) -> None:
        now = datetime.now(timezone.utc).isoformat()
        self._ensure().execute("UPDATE jobs SET status = 'paused', updated_ts = ? WHERE id = ?", (now, job_id))
        self._ensure().commit()

    def set_queued(self, job_id: str, next_run_ts: Optional[str] = None) -> None:
        now = datetime.now(timezone.utc).isoformat()
        if next_run_ts:
            self._ensure().execute("UPDATE jobs SET status = 'queued', next_run_ts = ?, updated_ts = ? WHERE id = ?", (next_run_ts, now, job_id))
        else:
            self._ensure().execute("UPDATE jobs SET status = 'queued', updated_ts = ? WHERE id = ?", (now, job_id))
        self._ensure().commit()

    def insert_run(self, job_id: str, ts_start: str, ts_end: str, ok: bool, ms: int, result_json: Optional[str], error: Optional[str]) -> None:
        conn = self._ensure()
        conn.execute(
            "INSERT INTO job_runs (job_id, ts_start, ts_end, ok, ms, result_json, error) VALUES (?, ?, ?, ?, ?, ?, ?)",
            (job_id, ts_start, ts_end, 1 if ok else 0, ms, result_json, error),
        )
        conn.commit()

    def get_runs(self, job_id: str, limit: int = 50) -> List[Dict[str, Any]]:
        conn = self._ensure()
        rows = conn.execute(
            "SELECT id, job_id, ts_start, ts_end, ok, ms, result_json, error FROM job_runs WHERE job_id = ? ORDER BY id DESC LIMIT ?",
            (job_id, limit),
        ).fetchall()
        return [
            {"id": r[0], "job_id": r[1], "ts_start": r[2], "ts_end": r[3], "ok": bool(r[4]), "ms": r[5], "result": json.loads(r[6]) if r[6] else None, "error": r[7]}
            for r in rows
        ]

    def _row_to_job(self, r: tuple) -> Dict[str, Any]:
        return {
            "id": r[0], "name": r[1], "kind": r[2], "payload": json.loads(r[3]) if r[3] else {},
            "cron": r[4], "run_at": r[5], "interval_seconds": r[6], "enabled": r[7], "status": r[8],
            "retries": r[9], "max_retries": r[10], "backoff_seconds": r[11], "last_run_ts": r[12],
            "next_run_ts": r[13], "last_error": r[14], "created_ts": r[15], "updated_ts": r[16],
        }
