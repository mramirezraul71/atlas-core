"""Last cursor run state for GET /cursor/status."""
from __future__ import annotations

from typing import Any, Dict, Optional

_last_run: Optional[Dict[str, Any]] = None


def set_last_run(data: Dict[str, Any]) -> None:
    global _last_run
    _last_run = data


def get_cursor_status() -> Dict[str, Any]:
    """Return last run state or empty."""
    if _last_run is None:
        return {"ok": True, "data": None, "message": "no run yet"}
    return {"ok": True, "data": _last_run, "message": None}
