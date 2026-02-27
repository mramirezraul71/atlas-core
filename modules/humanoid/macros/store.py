"""Macro store: SQLite with versioning."""
from __future__ import annotations

import json
import os
from pathlib import Path
from typing import Any, Dict, List, Optional


def _db_path() -> Path:
    base = os.getenv("ATLAS_REPO_PATH") or os.getenv("POLICY_ALLOWED_PATHS", "C:\\ATLAS_PUSH")
    base = base.strip().split(",")[0].strip()
    return Path(base) / "logs" / "atlas_macros.sqlite"


def _ensure_db() -> Any:
    import sqlite3
    p = _db_path()
    p.parent.mkdir(parents=True, exist_ok=True)
    conn = sqlite3.connect(str(p))
    conn.execute("""
        CREATE TABLE IF NOT EXISTS macros (
            id TEXT PRIMARY KEY,
            name TEXT NOT NULL,
            actions TEXT NOT NULL,
            created_at TEXT NOT NULL,
            version INTEGER DEFAULT 1
        )
    """)
    conn.commit()
    return conn


def list_macros(limit: int = 50) -> List[Dict[str, Any]]:
    conn = _ensure_db()
    try:
        rows = conn.execute(
            "SELECT id, name, created_at, version FROM macros ORDER BY created_at DESC LIMIT ?",
            (limit,),
        ).fetchall()
        return [{"id": r[0], "name": r[1], "created_at": r[2], "version": r[3]} for r in rows]
    finally:
        conn.close()


def get_macro(macro_id: str) -> Optional[Dict[str, Any]]:
    conn = _ensure_db()
    try:
        row = conn.execute(
            "SELECT id, name, actions, created_at, version FROM macros WHERE id = ?",
            (macro_id,),
        ).fetchone()
        if not row:
            return None
        actions = json.loads(row[2]) if isinstance(row[2], str) else row[2]
        return {"id": row[0], "name": row[1], "actions": actions, "created_at": row[3], "version": row[4]}
    finally:
        conn.close()


def save_macro(macro_id: str, name: str, actions: List[Dict[str, Any]]) -> Dict[str, Any]:
    import sqlite3
    from datetime import datetime, timezone
    conn = _ensure_db()
    try:
        now = datetime.now(timezone.utc).isoformat()
        actions_json = json.dumps(actions)
        cur = conn.execute("SELECT version FROM macros WHERE id = ?", (macro_id,))
        row = cur.fetchone()
        ver = (row[0] + 1) if row else 1
        conn.execute(
            "INSERT OR REPLACE INTO macros (id, name, actions, created_at, version) VALUES (?, ?, ?, ?, ?)",
            (macro_id, name, actions_json, now, ver),
        )
        conn.commit()
        return {"ok": True, "id": macro_id, "name": name, "version": ver}
    except sqlite3.Error as e:
        return {"ok": False, "error": str(e)}
    finally:
        conn.close()


def delete_macro(macro_id: str) -> Dict[str, Any]:
    conn = _ensure_db()
    try:
        conn.execute("DELETE FROM macros WHERE id = ?", (macro_id,))
        conn.commit()
        return {"ok": True, "id": macro_id}
    except Exception as e:
        return {"ok": False, "error": str(e)}
    finally:
        conn.close()
