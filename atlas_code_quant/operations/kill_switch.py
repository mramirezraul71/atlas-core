"""Central kill switch helpers for transition runtime."""
from __future__ import annotations

from typing import Any


def kill_switch_health(*, active: bool, available: bool = True, last_error: str | None = None) -> dict[str, Any]:
    healthy = bool(available) and (last_error is None or str(last_error).strip() == "")
    return {
        "active": bool(active),
        "available": bool(available),
        "healthy": healthy,
        "last_error": str(last_error or ""),
    }
