"""Bitácora ANS: entradas del módulo ATLAS_EVOLUTION para registro industrial. Sin silencio operativo."""
from __future__ import annotations

from collections import deque
from datetime import datetime, timezone
from typing import Dict, List

_MAX_EVOLUTION_ENTRIES = 300
_evolution_entries: deque = deque(maxlen=_MAX_EVOLUTION_ENTRIES)


def append_evolution_log(message: str, ok: bool = True) -> None:
    """Append one evolution step to the bitácora. Called by POST /ans/evolution-log."""
    entry = {
        "timestamp": datetime.now(timezone.utc).isoformat(),
        "message": (message or "")[:500],
        "ok": ok,
        "source": "evolution",
    }
    _evolution_entries.append(entry)


def get_evolution_entries(limit: int = 100) -> List[Dict]:
    """Return the most recent evolution log entries (newest first for merge)."""
    return list(_evolution_entries)[-limit:][::-1]
