"""Persistent memory SQLite: threads, tasks, runs, artifacts, decisions, summaries. FTS or LIKE fallback."""
from __future__ import annotations

import json
import os
import sqlite3
import uuid
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional

MEMORY_DB_PATH = os.getenv("ATLAS_MEMORY_DB_PATH", "C:\\ATLAS_PUSH\\logs\\atlas_memory.sqlite")

SCHEMA = """
CREATE TABLE IF NOT EXISTS threads (
    id TEXT PRIMARY KEY,
    title TEXT,
    created_ts TEXT NOT NULL,
    updated_ts TEXT NOT NULL
);
CREATE TABLE IF NOT EXISTS tasks (
    id TEXT PRIMARY KEY,
    thread_id TEXT,
    goal TEXT NOT NULL,
    plan_json TEXT,
    status TEXT,
    created_ts TEXT NOT NULL,
    updated_ts TEXT NOT NULL,
    FOREIGN KEY (thread_id) REFERENCES threads(id)
);
CREATE TABLE IF NOT EXISTS runs (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    task_id TEXT NOT NULL,
    step_id TEXT,
    ok INTEGER NOT NULL,
    result_json TEXT,
    error TEXT,
    ms INTEGER,
    created_ts TEXT NOT NULL,
    FOREIGN KEY (task_id) REFERENCES tasks(id)
);
CREATE TABLE IF NOT EXISTS artifacts (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    task_id TEXT,
    run_id INTEGER,
    kind TEXT,
    path_or_uri TEXT,
    content_preview TEXT,
    created_ts TEXT NOT NULL,
    FOREIGN KEY (task_id) REFERENCES tasks(id)
);
CREATE TABLE IF NOT EXISTS decisions (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    thread_id TEXT,
    task_id TEXT,
    decision_type TEXT,
    payload_json TEXT,
    created_ts TEXT NOT NULL,
    FOREIGN KEY (thread_id) REFERENCES threads(id)
);
CREATE TABLE IF NOT EXISTS summaries (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    thread_id TEXT,
    content TEXT NOT NULL,
    created_ts TEXT NOT NULL,
    FOREIGN KEY (thread_id) REFERENCES threads(id)
);
CREATE INDEX IF NOT EXISTS idx_tasks_thread ON tasks(thread_id);
CREATE INDEX IF NOT EXISTS idx_runs_task ON runs(task_id);
CREATE INDEX IF NOT EXISTS idx_artifacts_task ON artifacts(task_id);
CREATE INDEX IF NOT EXISTS idx_decisions_thread ON decisions(thread_id);
"""

# FTS virtual table for search (optional)
FTS_SCHEMA = """
CREATE VIRTUAL TABLE IF NOT EXISTS memory_fts USING fts5(
    content,
    thread_id,
    task_id,
    kind,
    tokenize='porter'
);
"""

_conn: Optional[sqlite3.Connection] = None
_fts_available: Optional[bool] = None


def _db_path() -> str:
    return os.getenv("ATLAS_MEMORY_DB_PATH", "C:\\ATLAS_PUSH\\logs\\atlas_memory.sqlite").strip()


def _ensure() -> sqlite3.Connection:
    global _conn
    if _conn is not None:
        return _conn
    path = _db_path()
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    _conn = sqlite3.connect(path, check_same_thread=False)
    _conn.execute("PRAGMA journal_mode=WAL")
    for stmt in SCHEMA.split(";"):
        stmt = stmt.strip()
        if stmt:
            _conn.execute(stmt)
    _conn.commit()
    _init_fts()
    return _conn


def _init_fts() -> bool:
    global _fts_available
    if _fts_available is not None:
        return _fts_available
    try:
        _conn.execute(FTS_SCHEMA)
        _conn.commit()
        _fts_available = True
    except sqlite3.OperationalError:
        _fts_available = False
    return _fts_available


def fts_available() -> bool:
    _ensure()
    return _fts_available or False


def new_id() -> str:
    return str(uuid.uuid4())


def now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()
