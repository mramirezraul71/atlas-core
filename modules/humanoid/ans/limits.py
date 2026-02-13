"""Rate limit + max actions per hour + cooldown."""
from __future__ import annotations

import os
import time
from collections import deque

_ACTIONS_TS: deque = deque(maxlen=1000)
_LAST_RESTART_TS: float = 0
_RESTART_COOLDOWN = 300  # 5 min between restarts


def _env_int(name: str, default: int) -> int:
    try:
        return int(os.getenv(name, str(default)).strip() or default)
    except Exception:
        return default


def can_auto_action(heal_id: str = "") -> tuple[bool, str]:
    max_per_hour = _env_int("ANS_MAX_AUTO_ACTIONS_PER_HOUR", 10)
    cooldown = _env_int("ANS_COOLDOWN_SECONDS", 60)
    now = time.time()
    cutoff = now - 3600
    recent = [t for t in _ACTIONS_TS if t > cutoff]
    if len(recent) >= max_per_hour:
        return False, "max_actions_per_hour_exceeded"
    if heal_id == "restart_api":
        global _LAST_RESTART_TS
        if now - _LAST_RESTART_TS < _RESTART_COOLDOWN:
            return False, "restart_cooldown"
        _LAST_RESTART_TS = now
    return True, "ok"


def record_action() -> None:
    _ACTIONS_TS.append(time.time())


def actions_last_hour() -> int:
    cutoff = time.time() - 3600
    return sum(1 for t in _ACTIONS_TS if t > cutoff)
