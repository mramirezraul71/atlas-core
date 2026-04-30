"""Estado de conectores multi-venue (reconexiones, último error)."""
from __future__ import annotations

import time
from dataclasses import dataclass, field


@dataclass
class ConnectorStats:
    last_event_ts: float = 0.0
    reconnects: int = 0
    last_error: str = ""


@dataclass
class ConnectorRegistry:
    """Registro en memoria; el orquestador pide snapshot para API/metrics."""

    _stats: dict[str, ConnectorStats] = field(default_factory=dict)

    def touch_event(self, source: str) -> None:
        s = self._stats.setdefault(source, ConnectorStats())
        s.last_event_ts = time.time()

    def record_reconnect(self, source: str, err: str = "") -> None:
        s = self._stats.setdefault(source, ConnectorStats())
        s.reconnects += 1
        s.last_error = (err or "")[:500]

    def snapshot(self) -> dict[str, dict[str, object]]:
        return {
            k: {
                "last_event_ts": v.last_event_ts,
                "reconnects": v.reconnects,
                "last_error": v.last_error,
            }
            for k, v in self._stats.items()
        }
