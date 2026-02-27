"""
PatternAnalyzer - Detecta patrones de uso: horarios, queries frecuentes, tools más usados.
"""
from __future__ import annotations

import logging
import time
from collections import Counter
from pathlib import Path
from typing import Any

logger = logging.getLogger(__name__)


def _load_config() -> dict:
    cfg_path = Path(__file__).resolve().parent.parent.parent / "config" / "autonomous.yaml"
    if not cfg_path.exists():
        return {}
    try:
        import yaml
        with open(cfg_path, encoding="utf-8") as f:
            return yaml.safe_load(f) or {}
    except Exception:
        return {}


class PatternAnalyzer:
    def __init__(self, config: dict | None = None):
        self._config = config or _load_config().get("learning", {})
        self._min_data_points = int(self._config.get("pattern_analysis", {}).get("min_data_points", 100))
        self._events: list[dict] = []
        self._max_events = 5000

    def record_event(self, event_type: str, data: dict | None = None) -> None:
        """Registra un evento de uso para análisis."""
        self._events.append({"type": event_type, "data": data or {}, "ts": time.time()})
        if len(self._events) > self._max_events:
            self._events.pop(0)

    def analyze_usage_patterns(self, time_range_sec: float = 86400 * 7) -> dict[str, Any]:
        """Agrupa por tipo de evento, horario, etc."""
        cutoff = time.time() - time_range_sec
        recent = [e for e in self._events if e.get("ts", 0) >= cutoff]
        if len(recent) < self._min_data_points:
            return {"patterns": {}, "data_points": len(recent), "message": "insufficient data"}
        types = Counter(e.get("type", "unknown") for e in recent)
        return {"patterns": {"by_type": dict(types)}, "data_points": len(recent)}

    def detect_anomalous_behavior(self) -> list[str]:
        """Detecta uso atípico (p. ej. pico repentino). Por ahora vacío."""
        return []

    def generate_insights(self) -> list[str]:
        """Insights accionables a partir de patrones."""
        p = self.analyze_usage_patterns()
        insights = []
        if p.get("patterns", {}).get("by_type"):
            top = sorted(p["patterns"]["by_type"].items(), key=lambda x: -x[1])[:3]
            insights.append(f"Top event types: {', '.join(f'{k}({v})' for k, v in top)}")
        return insights
