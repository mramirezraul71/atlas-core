"""Backtest: simulador GBM interno, DTO de motor unificado y puente a LEAN."""

from atlas_code_quant.backtest.pipeline_bridge import (
    fitness_to_engine_result,
    run_risk_preview_for_fitness,
    run_vision_preview_for_opportunity,
)
from atlas_code_quant.backtest.run_dto import (
    EngineBacktestRequest,
    EngineBacktestResult,
    EngineName,
)
from atlas_code_quant.backtest.unified_engine import (
    engine_result_from_fitness,
    run_unified_backtest_for_intent,
)

__all__ = [
    "EngineBacktestRequest",
    "EngineBacktestResult",
    "EngineName",
    "engine_result_from_fitness",
    "run_unified_backtest_for_intent",
    "fitness_to_engine_result",
    "run_risk_preview_for_fitness",
    "run_vision_preview_for_opportunity",
]
