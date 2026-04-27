"""Kill switch — F8.

Detecta la presencia de un archivo de kill-switch. Soporta polling síncrono
con cache temporal para evitar pegarse al disco.

Compatibilidad F1: ``is_kill_switch_active(path)`` se mantiene.
"""
from __future__ import annotations

import os
import time
from dataclasses import dataclass
from pathlib import Path


# ── F1 compat ───────────────────────────────────────────────────────────────
def is_kill_switch_active(path: str) -> bool:
    if not path:
        return False
    return Path(path).exists()


# ── F8 watcher ──────────────────────────────────────────────────────────────
@dataclass(slots=True)
class KillSwitchWatcher:
    """Watcher con cache de TTL para checkear el archivo de kill switch."""

    path: str = ""
    ttl_seconds: float = 1.0
    _last_check_ts: float = 0.0
    _last_value: bool = False

    @classmethod
    def from_env(cls) -> "KillSwitchWatcher":
        return cls(
            path=os.environ.get("ATLAS_KILL_SWITCH_FILE", ""),
            ttl_seconds=float(os.environ.get("ATLAS_KILL_SWITCH_TTL_SECONDS", "1.0")),
        )

    def is_active(self) -> bool:
        if not self.path:
            return False
        now = time.time()
        if now - self._last_check_ts < self.ttl_seconds:
            return self._last_value
        self._last_check_ts = now
        self._last_value = Path(self.path).exists()
        return self._last_value

    def force_refresh(self) -> bool:
        self._last_check_ts = 0.0
        return self.is_active()
