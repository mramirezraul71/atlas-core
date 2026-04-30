"""
Planner lógico de estrategias de opciones a partir de un contexto de mercado simple.

No conecta con intents del brain principal; solo mapea contexto → (StrategyType, params).
La apertura paper y riesgo siguen en ``AtlasOptionsService``.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Literal

from atlas_options_brain.integration.atlas_adapter import StrategyType

from .options_client import AtlasOptionsService

Trend = Literal["bull", "bear", "sideways", "slightly_bull"]

_DEFAULT_PLANNER_CONFIG: dict[str, Any] = {
    "iv_high_threshold": 60.0,
    "iv_low_threshold": 30.0,
    "default_width_credit": 10.0,
    "default_width_debit": 10.0,
    "default_qty": 1,
    "iron_condor_wing_delta": 0.18,
    "iron_condor_wing_width": 10.0,
    "credit_short_delta": 0.25,
    "debit_long_delta": 0.50,
    "covered_short_delta": 0.25,
}


@dataclass
class MarketContext:
    """Contexto mínimo para elegir estrategia (sin series históricas)."""

    symbol: str
    spot: float
    trend: Trend
    iv_rank: float | None = None
    regime: str | None = None


class OptionsStrategyPlanner:
    """
    Reglas explícitas tendencia + IV rank → tipo de estrategia y params DSL.

    IV: ``iv_rank`` en [0,100] o ``None`` (se trata como rango intermedio: ni alto ni bajo).
    """

    def __init__(self, config: dict[str, Any] | None = None) -> None:
        self._cfg: dict[str, Any] = dict(_DEFAULT_PLANNER_CONFIG)
        if config:
            self._cfg.update({k: v for k, v in config.items() if v is not None})

    def choose_strategy(self, context: MarketContext) -> tuple[StrategyType, dict[str, Any]]:
        c = self._cfg
        qty = int(c["default_qty"])
        w_cred = float(c["default_width_credit"])
        w_deb = float(c["default_width_debit"])
        iv_h = float(c["iv_high_threshold"])
        iv_l = float(c["iv_low_threshold"])

        # Covered call: stock largo + mercado lateral o leve alcista
        if context.regime == "has_stock" and context.trend in ("sideways", "slightly_bull"):
            return (
                "covered_call",
                {
                    "short_delta": float(c["covered_short_delta"]),
                    "stock_basis": float(context.spot),
                    "qty": qty,
                },
            )

        is_high = context.iv_rank is not None and float(context.iv_rank) >= iv_h
        is_low = context.iv_rank is not None and float(context.iv_rank) <= iv_l

        if context.trend == "sideways":
            return (
                "iron_condor",
                {
                    "wing_delta": float(c["iron_condor_wing_delta"]),
                    "wing_width": float(c["iron_condor_wing_width"]),
                    "qty": qty,
                },
            )

        if context.trend in ("bull", "slightly_bull"):
            if is_high:
                return (
                    "bull_put",
                    {
                        "short_delta": float(c["credit_short_delta"]),
                        "width": w_cred,
                        "qty": qty,
                    },
                )
            return (
                "bull_call",
                {
                    "long_delta": float(c["debit_long_delta"]),
                    "width": w_deb,
                    "qty": qty,
                },
            )

        if context.trend == "bear":
            if is_high:
                return (
                    "bear_call",
                    {
                        "short_delta": float(c["credit_short_delta"]),
                        "width": w_cred,
                        "qty": qty,
                    },
                )
            return (
                "bear_put",
                {
                    "long_delta": float(c["debit_long_delta"]),
                    "width": w_deb,
                    "qty": qty,
                },
            )

        raise ValueError(f"trend no soportado: {context.trend!r}")


class AtlasOptionsPlannerService:
    """
    Orquesta planner + ``AtlasOptionsService``: contexto → estrategia → apertura paper.

    Propaga ``AtlasOptionsRiskError`` sin modificarla.
    """

    def __init__(
        self,
        options_service: AtlasOptionsService,
        planner: OptionsStrategyPlanner | None = None,
    ) -> None:
        self._options = options_service
        self._planner = planner or OptionsStrategyPlanner()

    @property
    def options_service(self) -> AtlasOptionsService:
        return self._options

    @property
    def planner(self) -> OptionsStrategyPlanner:
        return self._planner

    def plan_and_open(self, context: MarketContext) -> str:
        strategy_type, params = self._planner.choose_strategy(context)
        return self._options.build_and_open(context.symbol, strategy_type, params)
