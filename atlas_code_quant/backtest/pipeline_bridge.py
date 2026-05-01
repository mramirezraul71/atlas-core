"""Puente ejecución: fitness → DTO → riesgo (F19) + visión (F18)."""

from __future__ import annotations

from typing import Mapping

from atlas_code_quant.autonomy.gates import GateResult
from atlas_code_quant.backtest.run_dto import EngineBacktestResult
from atlas_code_quant.backtest.unified_engine import engine_result_from_fitness
from atlas_code_quant.intake.opportunity import RadarOpportunityInternal
from atlas_code_quant.lean.runner.launcher import StrategyFitnessResult
from atlas_code_quant.risk.limits.checks import RiskLimitsConfig, check_all_limits
from atlas_code_quant.vision.timing_gate.gate import (
    GateInput,
    GateOutput,
    VisionTimingGate,
)


def fitness_to_engine_result(fitness: StrategyFitnessResult) -> EngineBacktestResult:
    """Alias explícito para consumo paper / riesgo."""
    return engine_result_from_fitness(fitness)


def run_risk_preview_for_fitness(
    fitness: StrategyFitnessResult,
    *,
    realized_pnl_usd: float = 0.0,
    notional_usd: float = 0.0,
    orders_in_last_minute: int | None = None,
    config: RiskLimitsConfig | None = None,
) -> GateResult:
    """Aplica límites F19 usando contadores observables del fitness."""
    orders = (
        int(fitness.num_orders)
        if orders_in_last_minute is None
        else orders_in_last_minute
    )
    return check_all_limits(
        realized_pnl_usd=realized_pnl_usd,
        notional_usd=notional_usd,
        orders_in_last_minute=orders,
        config=config,
    )


def run_vision_preview_for_opportunity(
    opp: RadarOpportunityInternal,
    *,
    strategy_id: str = "",
    requires_visual_confirmation: bool = False,
) -> GateOutput:
    """Evalúa :class:`VisionTimingGate` sin hardware (probe por defecto)."""
    gate = VisionTimingGate()
    payload: Mapping[str, Any] = {
        "symbol": opp.symbol,
        "score": opp.score,
        "direction": opp.direction,
        "classification": opp.classification,
        "trace_id": opp.trace_id,
    }
    inp = GateInput(
        opportunity=payload,
        strategy_id=strategy_id,
        requires_visual_confirmation=requires_visual_confirmation,
        market_open=True,
        extras={},
    )
    return gate.evaluate(inp)


__all__ = [
    "fitness_to_engine_result",
    "run_risk_preview_for_fitness",
    "run_vision_preview_for_opportunity",
]
