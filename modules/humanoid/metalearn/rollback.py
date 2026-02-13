"""Snapshot and rollback of learned/tuned params."""
from __future__ import annotations

import uuid
from datetime import datetime, timezone
from typing import Any, Dict, Optional

from . import db


def _now() -> str:
    return datetime.now(timezone.utc).isoformat()


def save_snapshot(params: Dict[str, Any], comment: Optional[str] = None) -> str:
    """Save current params to snapshots. Returns snapshot_id."""
    sid = str(uuid.uuid4())[:12]
    db.save_snapshot(sid, params, _now(), comment)
    return sid


def list_snapshots(limit: int = 20) -> list:
    return db.list_snapshots(limit=limit)


def rollback(snapshot_id: str) -> Dict[str, Any]:
    """
    Restore params from snapshot and set as current. Does not delete snapshot.
    Returns {ok, snapshot_id, restored: params, error}.
    """
    try:
        snap = db.get_snapshot(snapshot_id)
        if not snap:
            return {"ok": False, "snapshot_id": snapshot_id, "restored": None, "error": "snapshot not found"}
        params = snap.get("params") or {}
        from .tuner import restore_params
        restore_params(params)
        return {"ok": True, "snapshot_id": snapshot_id, "restored": params, "error": None}
    except Exception as e:
        return {"ok": False, "snapshot_id": snapshot_id, "restored": None, "error": str(e)}
