"""Contratos Pydantic compartidos por la Strategy Factory (F5).

Diseño:
- ``StrategyOpportunityRef``: subset mínimo (símbolo, score, dirección, horizonte)
  para que las estrategias no dependan del intake completo.
- ``StrategyConfig``: parámetros comunes (DTE, capital, max_loss).
- ``OptionLeg``: una pierna (call/put, buy/sell, strike, qty, expiry).
- ``StrategyPlan``: plan de estrategia auditable, con ``trace_id``,
  ``legs``, ``notional_estimate`` y ``rationale``.
"""
from __future__ import annotations

from typing import Literal

from pydantic import BaseModel, ConfigDict, Field


OptionRight = Literal["call", "put"]
OptionSide = Literal["buy", "sell"]
StrategyDirection = Literal["long", "short", "neutral"]


class StrategyOpportunityRef(BaseModel):
    """Subset minimal de una RadarOpportunity necesario para construir plan."""

    model_config = ConfigDict(extra="allow")

    symbol: str
    score: float = Field(ge=0.0, le=100.0)
    direction: StrategyDirection = "neutral"
    horizon_min: int = Field(default=30, ge=1, le=24 * 60 * 7)
    trace_id: str = ""

    @classmethod
    def from_dict(cls, d: dict) -> "StrategyOpportunityRef":
        return cls(
            symbol=str(d.get("symbol", "")).upper(),
            score=float(d.get("score", 0.0)),
            direction=str(d.get("direction", "neutral")) or "neutral",  # type: ignore[arg-type]
            horizon_min=int(d.get("horizon_min", 30) or 30),
            trace_id=str(d.get("trace_id", "")),
        )


class StrategyConfig(BaseModel):
    """Parámetros comunes a las estrategias de opciones."""

    model_config = ConfigDict(extra="allow")

    dte_min: int = Field(default=7, ge=0, le=365)
    dte_max: int = Field(default=45, ge=1, le=365)
    cash_alloc_usd: float = Field(default=2_500.0, ge=0.0)
    max_loss_usd: float = Field(default=250.0, ge=0.0)
    width_usd: float = Field(default=5.0, ge=0.5, description="Ancho strikes vertical/condor.")
    qty: int = Field(default=1, ge=1)


class OptionLeg(BaseModel):
    """Una pierna de la estrategia (placeholder de selección de strike)."""

    model_config = ConfigDict(extra="allow")

    side: OptionSide
    right: OptionRight
    strike_offset: float = Field(
        default=0.0,
        description="Offset relativo (USD) sobre el subyacente. Selección final en F8.",
    )
    qty: int = Field(default=1, ge=1)
    expiry_dte: int = Field(default=14, ge=0, le=365)


class StrategyPlan(BaseModel):
    """Plan de estrategia auditable. Sin lanzar broker."""

    model_config = ConfigDict(extra="allow")

    strategy: str
    symbol: str
    direction: StrategyDirection = "neutral"
    legs: list[OptionLeg] = Field(default_factory=list)
    notional_estimate_usd: float = 0.0
    max_loss_estimate_usd: float = 0.0
    horizon_min: int = 30
    rationale: str = ""
    status: Literal["planned", "rejected", "stub"] = "planned"
    trace_id: str = ""
    metadata: dict = Field(default_factory=dict)

    def n_legs(self) -> int:
        return len(self.legs)

    def is_actionable(self) -> bool:
        return self.status == "planned" and len(self.legs) >= 1
