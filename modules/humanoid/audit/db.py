"""Audit persistence: SQLite audit_log table. Path from AUDIT_DB_PATH."""
from __future__ import annotations

import json
import sqlite3
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional


_SCHEMA = """
CREATE TABLE IF NOT EXISTS audit_log (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    ts TEXT,
    actor TEXT,
    role TEXT,
    module TEXT,
    action TEXT,
    ok INTEGER,
    ms INTEGER,
    error TEXT,
    payload_json TEXT,
    result_json TEXT
)
"""


class AuditDB:
    """SQLite audit log. Schema: id, ts ISO, actor, role, module, action, ok, ms, error, payload_json, result_json."""

    def __init__(self, path: str | Path | None = None) -> None:
        import os
        self._path: Optional[Path] = None
        if path is not None:
            self._path = Path(path)
        else:
            env_path = os.getenv("AUDIT_DB_PATH")
            if env_path:
                self._path = Path(env_path)
        self._conn: Optional[sqlite3.Connection] = None

    def _ensure_conn(self) -> sqlite3.Connection:
        if self._conn is not None:
            return self._conn
        if self._path is None:
            raise RuntimeError("AuditDB: no path configured")
        self._path.parent.mkdir(parents=True, exist_ok=True)
        self._conn = sqlite3.connect(str(self._path))
        self._conn.execute(_SCHEMA)
        self._conn.commit()
        return self._conn

    def log_event(
        self,
        actor: str,
        role: str,
        module: str,
        action: str,
        ok: bool,
        ms: int = 0,
        error: Optional[str] = None,
        payload: Optional[Dict[str, Any]] = None,
        result: Optional[Dict[str, Any]] = None,
    ) -> None:
        """Insert one audit_log row. ts = ISO UTC."""
        try:
            conn = self._ensure_conn()
            ts = datetime.now(timezone.utc).isoformat()
            conn.execute(
                """INSERT INTO audit_log (ts, actor, role, module, action, ok, ms, error, payload_json, result_json)
                   VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)""",
                (
                    ts,
                    actor,
                    role,
                    module,
                    action,
                    1 if ok else 0,
                    ms,
                    error,
                    json.dumps(payload) if payload is not None else None,
                    json.dumps(result) if result is not None else None,
                ),
            )
            conn.commit()
        except Exception:
            pass

    def tail(self, n: int = 50, module: Optional[str] = None) -> List[Dict[str, Any]]:
        """Last n rows, optional filter by module."""
        try:
            conn = self._ensure_conn()
            if module:
                cur = conn.execute(
                    """SELECT id, ts, actor, role, module, action, ok, ms, error, payload_json, result_json
                       FROM audit_log WHERE module = ? ORDER BY id DESC LIMIT ?""",
                    (module, n),
                )
            else:
                cur = conn.execute(
                    """SELECT id, ts, actor, role, module, action, ok, ms, error, payload_json, result_json
                       FROM audit_log ORDER BY id DESC LIMIT ?""",
                    (n,),
                )
            rows = cur.fetchall()
            return [
                {
                    "id": r[0],
                    "ts": r[1],
                    "actor": r[2],
                    "role": r[3],
                    "module": r[4],
                    "action": r[5],
                    "ok": bool(r[6]),
                    "ms": r[7],
                    "error": r[8],
                    "payload": json.loads(r[9]) if r[9] else {},
                    "result": json.loads(r[10]) if r[10] else {},
                }
                for r in rows
            ]
        except Exception:
            return []
