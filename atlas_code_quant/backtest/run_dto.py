"""DTO canónico motor de backtest (plan Radar → Quant → LEAN → ejecución).

``EngineBacktestRequest`` / ``EngineBacktestResult`` son los nombres de
tipo en código para no colisionar con :class:`BacktestRunRequest` del
API HTTP en ``atlas_code_quant.api.main``.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Literal

EngineName = Literal["gbm", "lean_cli", "lean_external", "mock", "disabled", "error"]


@dataclass(frozen=True, slots=True)
class EngineBacktestRequest:
    """Petición mínima al motor unificado (símbolo + preferencia LEAN)."""

    symbol: str
    prefer_lean: bool = False
    gbm_years: float = 0.03
    extras: dict[str, Any] = field(default_factory=dict)


@dataclass(frozen=True, slots=True)
class EngineBacktestResult:
    """Resultado normalizado aguas abajo (riesgo / paper / visión)."""

    success: bool
    engine: EngineName
    sharpe: float = 0.0
    win_rate: float = 0.0
    max_drawdown: float = 0.0
    total_return: float = 0.0
    expectancy: float = 0.0
    num_orders: int = 0
    error_code: str | None = None
    error_message: str | None = None
    degraded_from: EngineName | None = None
    raw: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return {
            "success": self.success,
            "engine": self.engine,
            "sharpe": self.sharpe,
            "win_rate": self.win_rate,
            "max_drawdown": self.max_drawdown,
            "total_return": self.total_return,
            "expectancy": self.expectancy,
            "num_orders": self.num_orders,
            "error_code": self.error_code,
            "error_message": self.error_message,
            "degraded_from": self.degraded_from,
        }


__all__ = [
    "EngineBacktestRequest",
    "EngineBacktestResult",
    "EngineName",
]
