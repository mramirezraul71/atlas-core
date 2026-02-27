"""Cluster registry SQLite: nodes + node_events."""
from __future__ import annotations

import json
import os
import sqlite3
import time
from pathlib import Path
from typing import Any, Dict, List, Optional

_SCHEMA_NODES = """
CREATE TABLE IF NOT EXISTS nodes (
    node_id TEXT PRIMARY KEY,
    role TEXT NOT NULL,
    base_url TEXT NOT NULL,
    capabilities_json TEXT,
    last_seen_ts TEXT,
    health_score INTEGER DEFAULT 0,
    status TEXT DEFAULT 'offline',
    tags_json TEXT,
    created_ts TEXT,
    updated_ts TEXT
)
"""

_SCHEMA_EVENTS = """
CREATE TABLE IF NOT EXISTS node_events (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    ts TEXT NOT NULL,
    node_id TEXT NOT NULL,
    kind TEXT NOT NULL,
    data_json TEXT
)
"""

_SCHEMA_NONCES = """
CREATE TABLE IF NOT EXISTS nonces (
    nonce TEXT PRIMARY KEY,
    ts REAL NOT NULL
)
"""


def _db_path() -> Path:
    p = os.getenv("CLUSTER_REGISTRY_DB") or ""
    if p:
        return Path(p)
    return Path(os.getenv("ATLAS_PUSH_ROOT", os.getcwd())) / "logs" / "atlas_cluster.sqlite"


_conn: Optional[sqlite3.Connection] = None


def _ensure_conn() -> sqlite3.Connection:
    global _conn
    if _conn is not None:
        return _conn
    path = _db_path()
    path.parent.mkdir(parents=True, exist_ok=True)
    _conn = sqlite3.connect(str(path))
    _conn.execute(_SCHEMA_NODES)
    _conn.execute(_SCHEMA_EVENTS)
    _conn.execute(_SCHEMA_NONCES)
    _conn.commit()
    return _conn


def upsert_node(
    node_id: str,
    role: str,
    base_url: str,
    capabilities: Dict[str, bool],
    last_seen_ts: Optional[str] = None,
    health_score: int = 0,
    status: str = "offline",
    tags: Optional[Dict[str, str]] = None,
) -> None:
    conn = _ensure_conn()
    now = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
    caps_json = json.dumps(capabilities or {})
    tags_json = json.dumps(tags or {})
    conn.execute(
        """INSERT INTO nodes (node_id, role, base_url, capabilities_json, last_seen_ts, health_score, status, tags_json, created_ts, updated_ts)
           VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
           ON CONFLICT(node_id) DO UPDATE SET
             role=excluded.role, base_url=excluded.base_url, capabilities_json=excluded.capabilities_json,
             last_seen_ts=excluded.last_seen_ts, health_score=excluded.health_score, status=excluded.status,
             tags_json=excluded.tags_json, updated_ts=excluded.updated_ts""",
        (node_id, role, base_url, caps_json, last_seen_ts or now, health_score, status, tags_json, now, now),
    )
    conn.commit()


def list_nodes(status_filter: Optional[str] = None) -> List[Dict[str, Any]]:
    conn = _ensure_conn()
    if status_filter:
        rows = conn.execute("SELECT * FROM nodes WHERE status = ? ORDER BY last_seen_ts DESC", (status_filter,)).fetchall()
    else:
        rows = conn.execute("SELECT * FROM nodes ORDER BY last_seen_ts DESC").fetchall()
    cols = [c[0] for c in conn.execute("PRAGMA table_info(nodes)").fetchall()]
    out = []
    for row in rows:
        d = dict(zip(cols, row))
        if d.get("capabilities_json"):
            try:
                d["capabilities"] = json.loads(d["capabilities_json"])
            except Exception:
                d["capabilities"] = {}
        else:
            d["capabilities"] = {}
        if d.get("tags_json"):
            try:
                d["tags"] = json.loads(d["tags_json"])
            except Exception:
                d["tags"] = {}
        else:
            d["tags"] = {}
        out.append(d)
    return out


def get_node(node_id: str) -> Optional[Dict[str, Any]]:
    conn = _ensure_conn()
    row = conn.execute("SELECT * FROM nodes WHERE node_id = ?", (node_id,)).fetchone()
    if not row:
        return None
    cols = [c[0] for c in conn.execute("PRAGMA table_info(nodes)").fetchall()]
    d = dict(zip(cols, row))
    if d.get("capabilities_json"):
        try:
            d["capabilities"] = json.loads(d["capabilities_json"])
        except Exception:
            d["capabilities"] = {}
    if d.get("tags_json"):
        try:
            d["tags"] = json.loads(d["tags_json"])
        except Exception:
            d["tags"] = {}
    return d


def log_event(node_id: str, kind: str, data: Optional[Dict[str, Any]] = None) -> None:
    conn = _ensure_conn()
    now = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
    data_json = json.dumps(data or {}) if data else None
    conn.execute("INSERT INTO node_events (ts, node_id, kind, data_json) VALUES (?, ?, ?, ?)", (now, node_id, kind, data_json))
    conn.commit()


def get_events(limit: int = 50, node_id: Optional[str] = None) -> List[Dict[str, Any]]:
    conn = _ensure_conn()
    if node_id:
        rows = conn.execute("SELECT id, ts, node_id, kind, data_json FROM node_events WHERE node_id = ? ORDER BY id DESC LIMIT ?", (node_id, limit)).fetchall()
    else:
        rows = conn.execute("SELECT id, ts, node_id, kind, data_json FROM node_events ORDER BY id DESC LIMIT ?", (limit,)).fetchall()
    out = []
    for row in rows:
        id_, ts, nid, kind, data_json = row
        data = json.loads(data_json) if data_json else {}
        out.append({"id": id_, "ts": ts, "node_id": nid, "kind": kind, "data": data})
    return out


def nonce_seen(nonce: str, ttl_sec: float = 3600) -> bool:
    """Return True if nonce already seen (replay). Then record it."""
    conn = _ensure_conn()
    now = time.time()
    existing = conn.execute("SELECT 1 FROM nonces WHERE nonce = ?", (nonce,)).fetchone()
    if existing:
        return True
    conn.execute("INSERT INTO nonces (nonce, ts) VALUES (?, ?)", (nonce, now))
    conn.execute("DELETE FROM nonces WHERE ts < ?", (now - ttl_sec,))
    conn.commit()
    return False
