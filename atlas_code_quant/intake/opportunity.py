"""Contratos tipados de intake Radar (F3 cutover)."""
from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime, timezone
from typing import Any


@dataclass(slots=True)
class RadarOpportunity:
    """Oportunidad emitida por ``/api/radar/opportunities``."""

    symbol: str
    score: float
    asset_class: str = "unknown"
    classification: str = "watchlist"
    timestamp: str = ""
    horizon_min: int = 0
    direction: str = "neutral"
    snapshot: dict[str, Any] = field(default_factory=dict)
    degradations_active: list[dict[str, Any]] = field(default_factory=list)
    source: str = "stub"
    trace_id: str = ""
    payload: dict[str, Any] = field(default_factory=dict)  # extensiones futuras
    generated_at: str = field(default_factory=lambda: datetime.now(timezone.utc).isoformat())

    @classmethod
    def from_dict(cls, raw: dict[str, Any]) -> "RadarOpportunity":
        if not isinstance(raw, dict):
            raise TypeError("RadarOpportunity.from_dict requiere dict")
        symbol = str(raw.get("symbol") or "").strip().upper()
        if not symbol:
            raise ValueError("RadarOpportunity inválida: symbol vacío")
        score_raw = raw.get("score", 0.0)
        try:
            score = float(score_raw)
        except (TypeError, ValueError):
            score = 0.0
        horizon_raw = raw.get("horizon_min", 0)
        try:
            horizon = int(horizon_raw)
        except (TypeError, ValueError):
            horizon = 0
        return cls(
            symbol=symbol,
            asset_class=str(raw.get("asset_class") or "unknown"),
            score=score,
            classification=str(raw.get("classification") or "watchlist"),
            timestamp=str(raw.get("timestamp") or ""),
            horizon_min=max(0, horizon),
            direction=str(raw.get("direction") or "neutral"),
            snapshot=dict(raw.get("snapshot") or {}),
            degradations_active=list(raw.get("degradations_active") or []),
            source=str(raw.get("source") or "stub"),
            trace_id=str(raw.get("trace_id") or ""),
            payload={
                k: v
                for k, v in raw.items()
                if k
                not in {
                    "symbol",
                    "asset_class",
                    "score",
                    "classification",
                    "timestamp",
                    "horizon_min",
                    "direction",
                    "snapshot",
                    "degradations_active",
                    "source",
                    "trace_id",
                }
            },
        )


@dataclass(slots=True)
class RadarOpportunityBatch:
    """Respuesta tipada de ``/api/radar/opportunities``."""

    ok: bool = True
    items: list[RadarOpportunity] = field(default_factory=list)
    truncated: bool = False
    universe_size: int = 0
    min_score: float = 0.0
    degraded_globally: bool = False
    global_degradations: list[dict[str, Any]] = field(default_factory=list)
    trace_id: str = ""
    message: str | None = None
    source: str = "radar"
    generated_at: str = field(default_factory=lambda: datetime.now(timezone.utc).isoformat())

    @classmethod
    def from_response(cls, payload: dict[str, Any]) -> "RadarOpportunityBatch":
        if not isinstance(payload, dict):
            raise TypeError("RadarOpportunityBatch.from_response requiere dict")
        opps = [RadarOpportunity.from_dict(x) for x in list(payload.get("opportunities") or []) if isinstance(x, dict)]
        try:
            min_score = float(payload.get("min_score") or 0.0)
        except (TypeError, ValueError):
            min_score = 0.0
        try:
            universe_size = int(payload.get("universe_evaluated") or 0)
        except (TypeError, ValueError):
            universe_size = 0
        return cls(
            ok=bool(payload.get("ok", True)),
            items=opps,
            truncated=bool(payload.get("truncated")),
            universe_size=max(0, universe_size),
            min_score=min_score,
            degraded_globally=bool(payload.get("degraded_globally")),
            global_degradations=list(payload.get("global_degradations") or []),
            trace_id=str(payload.get("trace_id") or ""),
            message=(str(payload.get("message")) if payload.get("message") is not None else None),
            source=str(payload.get("source") or "radar"),
        )
