"""LEAN AlphaModel que consume oportunidades del Radar Atlas.

Lectura conservadora: si LEAN no está disponible (entorno test ATLAS),
las clases QuantConnect se importan en demanda y este módulo solo expone
una clase Python pura ``RadarAlphaSignal`` para tests/lints.
"""
from __future__ import annotations

from dataclasses import dataclass
from datetime import timedelta
from typing import Any


@dataclass(slots=True)
class RadarAlphaSignal:
    """Representación pura de una señal Radar consumida por el AlphaModel."""

    symbol: str
    score: float
    direction: str             # "long" | "short" | "neutral"
    horizon_min: int
    trace_id: str = ""

    def is_actionable(self, min_score: float) -> bool:
        return self.score >= min_score and self.direction in {"long", "short"}


def _import_lean():  # pragma: no cover — solo se ejecuta dentro de LEAN
    from QuantConnect.Algorithm.Framework.Alphas import AlphaModel, Insight, InsightDirection
    return AlphaModel, Insight, InsightDirection


def build_radar_alpha_model(min_score: float = 80.0):  # pragma: no cover
    AlphaModel, Insight, InsightDirection = _import_lean()

    class RadarAlphaModel(AlphaModel):
        """Genera Insights desde Custom Data ``radar_opportunities``.

        Las oportunidades se inyectan como CustomData o se leen vía REST en
        modo external. Cada insight respeta ``horizon_min`` declarado por
        el Radar.
        """

        def __init__(self, min_score: float = min_score) -> None:
            super().__init__()
            self._min_score = float(min_score)

        def Update(self, algorithm: Any, data: Any):
            insights: list[Any] = []
            opps = getattr(algorithm, "radar_opportunities", []) or []
            for raw in opps:
                sig = RadarAlphaSignal(
                    symbol=str(raw.get("symbol", "")).upper(),
                    score=float(raw.get("score", 0.0)),
                    direction=str(raw.get("direction", "neutral")),
                    horizon_min=int(raw.get("horizon_min", 30)),
                    trace_id=str(raw.get("trace_id", "")),
                )
                if not sig.is_actionable(self._min_score):
                    continue
                direction = (
                    InsightDirection.Up if sig.direction == "long" else InsightDirection.Down
                )
                insights.append(
                    Insight.Price(sig.symbol, timedelta(minutes=sig.horizon_min), direction)
                )
            return insights

    return RadarAlphaModel
