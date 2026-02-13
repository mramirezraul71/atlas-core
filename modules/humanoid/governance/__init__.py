"""Dual Governance Mode: GROWTH / GOVERNED / EMERGENCY STOP. Control por botones, persistencia, auditoría."""
from __future__ import annotations

from .state import get_mode, get_emergency_stop, set_mode, set_emergency_stop
from .gates import decide, is_blocked_by_emergency, needs_approval

__all__ = [
    "get_mode",
    "get_emergency_stop",
    "set_mode",
    "set_emergency_stop",
    "decide",
    "is_blocked_by_emergency",
    "needs_approval",
]
