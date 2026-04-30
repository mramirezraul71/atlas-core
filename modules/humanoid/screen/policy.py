"""Screen action policy: allowlist apps/windows, rate limit, screen_act gate."""
from __future__ import annotations

import os
import time
from typing import Any, Dict, List

# Rate: max N actions per minute
_rate_ts: List[float] = []
_RATE_LIMIT = 60
_RATE_WINDOW = 60.0


def _env_list(name: str, default: str) -> List[str]:
    v = os.getenv(name, default)
    return [x.strip() for x in (v or "").split(",") if x.strip()]


def get_screen_allowlist_apps() -> List[str]:
    """Allowed app/window titles (partial match). Default: empty = allow all when policy allows."""
    return _env_list("POLICY_SCREEN_ALLOWED_APPS", "")


def check_rate_limit() -> tuple[bool, str]:
    """Returns (allowed, reason)."""
    global _rate_ts
    now = time.monotonic()
    _rate_ts = [t for t in _rate_ts if now - t < _RATE_WINDOW]
    if len(_rate_ts) >= _RATE_LIMIT:
        return False, "rate_limit_exceeded"
    _rate_ts.append(now)
    return True, "ok"


def record_screen_act() -> None:
    """Call after policy allowed action (for rate limit)."""
    check_rate_limit()


def is_destructive_action(action: str, payload: Dict[str, Any]) -> bool:
    """Heuristic: type with long text or hotkey ctrl+v / delete-like could be destructive."""
    if action == "type" and len((payload.get("text") or "")) > 200:
        return True
    if action == "hotkey":
        keys = [str(k).lower() for k in (payload.get("keys") or [])]
        if "delete" in keys or "backspace" in keys or ("ctrl" in keys and "v" in keys):
            return True
    return False
