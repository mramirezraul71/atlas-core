"""Helpers de kill-switch para F1."""
from __future__ import annotations

from pathlib import Path


def is_kill_switch_active(path: str) -> bool:
    """Retorna True si existe archivo de kill-switch."""
    if not path:
        return False
    return Path(path).exists()
