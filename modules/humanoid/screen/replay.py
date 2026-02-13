"""Replay macro with verification."""
from __future__ import annotations

import time
from typing import Any, Dict, List

from .actions import execute_action


def replay_actions(actions: List[Dict[str, Any]], delay_ms: int = 100) -> Dict[str, Any]:
    """Replay a list of actions. Returns {ok, results, errors}."""
    results: List[Dict[str, Any]] = []
    errors: List[str] = []
    for i, item in enumerate(actions):
        action = item.get("action", "")
        payload = item.get("payload") or {}
        if i > 0 and delay_ms > 0:
            time.sleep(delay_ms / 1000.0)
        r = execute_action(action, payload)
        results.append({"index": i, "action": action, "result": r})
        if not r.get("ok"):
            errors.append(f"step {i}: {r.get('error', 'unknown')}")
    return {"ok": len(errors) == 0, "results": results, "errors": errors}
