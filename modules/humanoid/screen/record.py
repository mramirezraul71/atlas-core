"""Record screen actions (macro)."""
from __future__ import annotations

import time
from typing import Any, Dict, List, Optional

_recording: bool = False
_recorded_actions: List[Dict[str, Any]] = []
_record_start_ts: Optional[float] = None


def start_recording() -> Dict[str, Any]:
    global _recording, _recorded_actions, _record_start_ts
    if _recording:
        return {"ok": False, "error": "already_recording"}
    _recorded_actions = []
    _record_start_ts = time.perf_counter()
    _recording = True
    return {"ok": True, "message": "recording_started"}


def stop_recording() -> Dict[str, Any]:
    global _recording
    if not _recording:
        return {"ok": False, "error": "not_recording", "actions": []}
    _recording = False
    return {"ok": True, "actions": list(_recorded_actions), "count": len(_recorded_actions)}


def record_action(action: str, payload: Dict[str, Any]) -> None:
    global _recorded_actions, _record_start_ts
    if not _recording:
        return
    ts = (time.perf_counter() - (_record_start_ts or 0)) * 1000
    _recorded_actions.append({"action": action, "payload": payload, "ts_ms": ts})


def is_recording() -> bool:
    return _recording


def get_recorded_actions() -> List[Dict[str, Any]]:
    return list(_recorded_actions)
