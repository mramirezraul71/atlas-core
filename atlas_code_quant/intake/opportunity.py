"""Contratos base de oportunidades para intake Radar.

F1: define modelos tipados sin acoplar todavía al flujo productivo.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime, timezone
from typing import Any


@dataclass(slots=True)
class RadarOpportunity:
    """Representa una oportunidad emitida por Radar."""

    symbol: str
    score: float
    classification: str = "watchlist"
    direction: str = "neutral"
    source: str = "stub"
    trace_id: str = ""
    payload: dict[str, Any] = field(default_factory=dict)
    generated_at: str = field(
        default_factory=lambda: datetime.now(timezone.utc).isoformat()
    )


@dataclass(slots=True)
class RadarOpportunityBatch:
    """Lote de oportunidades para procesos batch/multi-símbolo futuros."""

    items: list[RadarOpportunity] = field(default_factory=list)
    truncated: bool = False
    universe_size: int = 0
    generated_at: str = field(
        default_factory=lambda: datetime.now(timezone.utc).isoformat()
    )
