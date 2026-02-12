"""Emergency mode: suspend non-critical, block deploys, only health + status."""
from __future__ import annotations

import os
from typing import Any, Dict

_EMERGENCY: bool = False


def is_emergency() -> bool:
    global _EMERGENCY
    if os.getenv("EMERGENCY_MODE", "").strip().lower() in ("1", "true", "yes"):
        return True
    return _EMERGENCY


def set_emergency(enable: bool) -> None:
    global _EMERGENCY
    _EMERGENCY = bool(enable)


def set_emergency_from_env() -> None:
    global _EMERGENCY
    _EMERGENCY = os.getenv("EMERGENCY_MODE", "").strip().lower() in ("1", "true", "yes")
