from __future__ import annotations

import json
import os
import threading
from datetime import datetime, timedelta, timezone
from pathlib import Path
from typing import Any, Dict, List

BASE_DIR = Path(__file__).resolve().parent.parent
HISTORY_PATH = Path(
    os.getenv("ATLAS_EXEC_HISTORY_PATH")
    or str(BASE_DIR / "logs" / "agent_execution_history.jsonl")
)
_LOCK = threading.Lock()
MAX_RECORDS = max(100, int(os.getenv("ATLAS_EXEC_HISTORY_MAX_RECORDS", "5000")))
MAX_AGE_DAYS = max(1, int(os.getenv("ATLAS_EXEC_HISTORY_MAX_DAYS", "30")))


def _parse_ts(value: str) -> datetime | None:
    if not value:
        return None
    try:
        dt = datetime.fromisoformat(str(value).replace("Z", "+00:00"))
        if dt.tzinfo is None:
            dt = dt.replace(tzinfo=timezone.utc)
        return dt.astimezone(timezone.utc)
    except Exception:
        return None


def _compact_records(records: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    cutoff = datetime.now(timezone.utc) - timedelta(days=MAX_AGE_DAYS)
    kept: List[Dict[str, Any]] = []
    for rec in records:
        ts = _parse_ts(str(rec.get("ts") or ""))
        if ts is None:
            # Keep undated records as legacy compatibility.
            kept.append(rec)
            continue
        if ts >= cutoff:
            kept.append(rec)
    if len(kept) > MAX_RECORDS:
        kept = kept[-MAX_RECORDS:]
    return kept


def append_execution_record(record: Dict[str, Any]) -> bool:
    """Append one execution record as JSONL line."""
    try:
        HISTORY_PATH.parent.mkdir(parents=True, exist_ok=True)
        with _LOCK:
            current = _read_all_unlocked()
            current.append(record)
            compacted = _compact_records(current)
            _write_all_unlocked(compacted)
        return True
    except Exception:
        return False


def list_recent_executions(limit: int = 20) -> List[Dict[str, Any]]:
    """Return latest execution records, newest first."""
    safe_limit = max(1, min(int(limit), 200))
    with _LOCK:
        items = _read_all_unlocked()
    if not items:
        return []
    out = list(reversed(items))
    return out[:safe_limit]


def get_execution_by_id(execution_id: str) -> Dict[str, Any] | None:
    eid = str(execution_id or "").strip()
    if not eid:
        return None
    with _LOCK:
        items = _read_all_unlocked()
    for rec in reversed(items):
        if str(rec.get("id") or "") == eid:
            return rec
    return None


def _read_all_unlocked() -> List[Dict[str, Any]]:
    if not HISTORY_PATH.exists():
        return []
    try:
        lines = HISTORY_PATH.read_text(encoding="utf-8", errors="replace").splitlines()
    except Exception:
        return []
    items: List[Dict[str, Any]] = []
    for raw in lines:
        if not raw.strip():
            continue
        try:
            items.append(json.loads(raw))
        except Exception:
            continue
    return items


def _write_all_unlocked(records: List[Dict[str, Any]]) -> None:
    HISTORY_PATH.parent.mkdir(parents=True, exist_ok=True)
    payload = "\n".join(json.dumps(r, ensure_ascii=False, default=str) for r in records)
    if payload:
        payload += "\n"
    HISTORY_PATH.write_text(payload, encoding="utf-8")

