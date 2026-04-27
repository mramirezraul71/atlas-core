"""Risk Limits — F6 (esqueleto).

Política sencilla y testeable que envuelve los gates duros de riesgo:
- max_position_notional_usd
- max_daily_loss_usd
- max_open_positions
- per_symbol_max_notional_usd

Las reglas se aplican vía ``RiskLimits.check(intent)`` y devuelven
``RiskDecision(allowed, reason)`` para decisión transparente y auditable.
"""
from __future__ import annotations

import os
from dataclasses import dataclass
from typing import Literal


def _env_float(name: str, default: float) -> float:
    try:
        return float(os.environ.get(name, default))
    except (TypeError, ValueError):
        return default


def _env_int(name: str, default: int) -> int:
    try:
        return int(os.environ.get(name, default))
    except (TypeError, ValueError):
        return default


@dataclass(slots=True)
class RiskLimits:
    max_position_notional_usd: float = 5_000.0
    max_daily_loss_usd: float = 1_000.0
    max_open_positions: int = 8
    per_symbol_max_notional_usd: float = 2_500.0

    @classmethod
    def from_env(cls) -> "RiskLimits":
        return cls(
            max_position_notional_usd=_env_float("ATLAS_MAX_POSITION_NOTIONAL_USD", 5_000.0),
            max_daily_loss_usd=_env_float("ATLAS_MAX_DAILY_LOSS_USD", 1_000.0),
            max_open_positions=_env_int("ATLAS_MAX_OPEN_POSITIONS", 8),
            per_symbol_max_notional_usd=_env_float("ATLAS_PER_SYMBOL_MAX_NOTIONAL_USD", 2_500.0),
        )


@dataclass(slots=True)
class RiskDecision:
    allowed: bool
    reason: str = ""


@dataclass(slots=True)
class RiskState:
    realized_pnl_today_usd: float = 0.0
    open_positions: int = 0
    notional_per_symbol_usd: dict[str, float] | None = None

    def notional_of(self, symbol: str) -> float:
        if not self.notional_per_symbol_usd:
            return 0.0
        return float(self.notional_per_symbol_usd.get(symbol, 0.0))


@dataclass(slots=True)
class TradeIntent:
    symbol: str
    notional_usd: float
    side: Literal["open", "close"] = "open"


def check(intent: TradeIntent, state: RiskState, limits: RiskLimits) -> RiskDecision:
    """Devuelve la primera razón de bloqueo o ``allowed=True``."""
    if intent.side == "close":
        return RiskDecision(allowed=True, reason="closing")

    if state.realized_pnl_today_usd <= -abs(limits.max_daily_loss_usd):
        return RiskDecision(allowed=False, reason="max_daily_loss_breached")

    if state.open_positions >= limits.max_open_positions:
        return RiskDecision(allowed=False, reason="max_open_positions_reached")

    if intent.notional_usd > limits.max_position_notional_usd:
        return RiskDecision(allowed=False, reason="position_notional_exceeds_limit")

    if (
        state.notional_of(intent.symbol) + intent.notional_usd
        > limits.per_symbol_max_notional_usd
    ):
        return RiskDecision(allowed=False, reason="per_symbol_notional_exceeds_limit")

    return RiskDecision(allowed=True, reason="ok")
