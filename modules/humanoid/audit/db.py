"""Audit persistence (e.g. SQLite)."""
from __future__ import annotations

import json
import sqlite3
import time
from pathlib import Path
from typing import Any, Dict, List, Optional


class AuditDB:
    """Simple audit log storage. Optional SQLite backend."""

    def __init__(self, path: str | Path | None = None) -> None:
        self._path = Path(path) if path else None
        self._conn: Optional[sqlite3.Connection] = None

    def _ensure_conn(self) -> sqlite3.Connection:
        if self._conn is not None:
            return self._conn
        if self._path is None:
            raise RuntimeError("AuditDB: no path configured")
        self._path.parent.mkdir(parents=True, exist_ok=True)
        self._conn = sqlite3.connect(str(self._path))
        self._conn.execute(
            """CREATE TABLE IF NOT EXISTS audit (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                ts REAL,
                scope TEXT,
                action TEXT,
                payload TEXT,
                result TEXT
            )"""
        )
        self._conn.commit()
        return self._conn

    def log(self, scope: str, action: str, payload: Dict[str, Any], result: str = "ok") -> None:
        """Append one audit entry."""
        try:
            conn = self._ensure_conn()
            conn.execute(
                "INSERT INTO audit (ts, scope, action, payload, result) VALUES (?, ?, ?, ?, ?)",
                (time.time(), scope, action, json.dumps(payload), result),
            )
            conn.commit()
        except Exception:
            pass

    def query(self, scope: str | None = None, limit: int = 100) -> List[Dict[str, Any]]:
        """Return recent audit entries, optionally filtered by scope."""
        try:
            conn = self._ensure_conn()
            if scope:
                cur = conn.execute(
                    "SELECT ts, scope, action, payload, result FROM audit WHERE scope = ? ORDER BY id DESC LIMIT ?",
                    (scope, limit),
                )
            else:
                cur = conn.execute(
                    "SELECT ts, scope, action, payload, result FROM audit ORDER BY id DESC LIMIT ?",
                    (limit,),
                )
            rows = cur.fetchall()
            return [
                {
                    "ts": r[0],
                    "scope": r[1],
                    "action": r[2],
                    "payload": json.loads(r[3]) if r[3] else {},
                    "result": r[4],
                }
                for r in rows
            ]
        except Exception:
            return []
