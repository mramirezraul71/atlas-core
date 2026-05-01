"""Motor unificado: LEAN CLI → adapter F13 → degradación GBM interna."""

from __future__ import annotations

import logging
import time
from typing import Any

from atlas_code_quant.backtest.internal_gbm_simulator import LeanSimulator, SimConfig
from atlas_code_quant.backtest.run_dto import EngineBacktestResult, EngineName
from atlas_code_quant.lean.cli_backtest import run_lean_cli_backtest
from atlas_code_quant.lean.config import LeanAdapterConfig, load_config_from_env
from atlas_code_quant.lean.runner.launcher import (
    StrategyFitnessResult,
    run_backtest_for_strategy_intent,
)
from atlas_code_quant.strategies.options.intent import StrategyIntent

logger = logging.getLogger("atlas.code_quant.backtest.unified")


def _gbm_fitness_from_intent(
    intent: StrategyIntent,
    *,
    years: float = 0.03,
) -> StrategyFitnessResult:
    """Backtest sintético rápido vía GBM (no QuantConnect)."""
    sym = intent.opportunity.symbol
    sim = LeanSimulator(symbols=[sym], config=SimConfig(random_seed=42))
    ohlcv = sim.generate_historical_data(sym, years=years)  # type: ignore[arg-type]
    trades_df, _feat = sim.run_atlas_strategy(sym, ohlcv)
    if trades_df.empty:
        return StrategyFitnessResult(
            success=True,
            mode="mock",
            sharpe=0.0,
            win_rate=0.0,
            max_drawdown=0.0,
            total_return=0.0,
            expectancy=0.0,
            num_orders=0,
            config_snapshot={"engine": "gbm", "gbm_note": "no_trades"},
        )
    m = sim.compute_metrics(trades_df)
    sharpe = float(m.get("sharpe") or 0.0)
    win_rate = float(m.get("win_rate") or 0.0)
    max_dd_pct = float(m.get("max_drawdown_pct") or 0.0)
    total_pnl = float(m.get("total_pnl") or 0.0)
    n_trades = int(m.get("n_trades") or 0)
    return StrategyFitnessResult(
        success=True,
        mode="mock",
        sharpe=sharpe,
        win_rate=win_rate,
        max_drawdown=-abs(max_dd_pct),
        total_return=total_pnl / max(float(sim.cfg.capital), 1.0) * 100.0,
        expectancy=float(m.get("avg_pnl") or 0.0),
        num_orders=n_trades,
        config_snapshot={"engine": "gbm", "degraded": True},
    )


def run_unified_backtest_for_intent(
    intent: StrategyIntent,
    *,
    config: LeanAdapterConfig | None = None,
) -> StrategyFitnessResult:
    """Orquesta CLI LEAN, F13 y degradación GBM según flags."""
    cfg = config if config is not None else load_config_from_env()
    t0 = time.perf_counter()

    if cfg.is_cli_ready:
        logger.info(
            "unified_backtest: lean_cli symbol=%s algorithm=%s",
            intent.opportunity.symbol,
            cfg.algorithm_name,
        )
        cli_res = run_lean_cli_backtest(intent, cfg)
        elapsed_ms = (time.perf_counter() - t0) * 1000
        logger.info(
            "unified_backtest: lean_cli done ok=%s elapsed_ms=%.1f",
            cli_res.success,
            elapsed_ms,
        )
        if cli_res.success:
            return cli_res
        if cfg.degrade_to_gbm and cfg.enabled:
            logger.warning(
                "unified_backtest: degrading to GBM after CLI failure code=%s",
                cli_res.error_code,
            )
            gbm = _gbm_fitness_from_intent(intent)
            snap = dict(gbm.config_snapshot or {})
            snap["degraded_from"] = "lean_cli"
            snap["prior_lean_error"] = cli_res.error_code
            return StrategyFitnessResult(
                success=gbm.success,
                mode=gbm.mode,
                sharpe=gbm.sharpe,
                win_rate=gbm.win_rate,
                max_drawdown=gbm.max_drawdown,
                total_return=gbm.total_return,
                expectancy=gbm.expectancy,
                num_orders=gbm.num_orders,
                config_snapshot=snap,
            )
        return cli_res

    primary = run_backtest_for_strategy_intent(intent, config=cfg)
    elapsed_ms = (time.perf_counter() - t0) * 1000
    logger.info(
        "unified_backtest: f13 path ok=%s mode=%s elapsed_ms=%.1f",
        primary.success,
        primary.mode,
        elapsed_ms,
    )
    if primary.success:
        return primary
    if (
        cfg.degrade_to_gbm
        and cfg.enabled
        and cfg.is_external
        and not primary.success
    ):
        logger.warning(
            "unified_backtest: degrading to GBM after external F13 failure code=%s",
            primary.error_code,
        )
        gbm = _gbm_fitness_from_intent(intent)
        snap = dict(gbm.config_snapshot or {})
        snap["degraded_from"] = "lean_external"
        snap["prior_lean_error"] = primary.error_code
        return StrategyFitnessResult(
            success=gbm.success,
            mode=gbm.mode,
            sharpe=gbm.sharpe,
            win_rate=gbm.win_rate,
            max_drawdown=gbm.max_drawdown,
            total_return=gbm.total_return,
            expectancy=gbm.expectancy,
            num_orders=gbm.num_orders,
            config_snapshot=snap,
        )
    return primary


def engine_result_from_fitness(
    fitness: StrategyFitnessResult,
    *,
    engine: EngineName | None = None,
    degraded_from: EngineName | None = None,
) -> EngineBacktestResult:
    """Mapea :class:`StrategyFitnessResult` al DTO ``EngineBacktestResult``."""
    snap0 = fitness.config_snapshot or {}
    deg = degraded_from
    if deg is None and snap0.get("degraded_from") in (
        "lean_cli",
        "lean_external",
        "gbm",
        "mock",
        "disabled",
        "error",
    ):
        deg = snap0["degraded_from"]  # type: ignore[assignment]
    eng: EngineName
    if engine is not None:
        eng = engine
    elif fitness.mode == "external":
        eng = "lean_external"
    elif fitness.mode == "mock":
        if snap0.get("engine") == "gbm":
            eng = "gbm"
        else:
            eng = "mock"
    elif fitness.mode == "disabled":
        eng = "disabled"
    else:
        eng = "error"
    raw: dict[str, Any] = dict(snap0)
    return EngineBacktestResult(
        success=fitness.success,
        engine=eng,
        sharpe=float(fitness.sharpe),
        win_rate=float(fitness.win_rate),
        max_drawdown=float(fitness.max_drawdown),
        total_return=float(fitness.total_return),
        expectancy=float(fitness.expectancy),
        num_orders=int(fitness.num_orders),
        error_code=fitness.error_code,
        error_message=fitness.error_message,
        degraded_from=deg,
        raw=raw,
    )


__all__ = [
    "run_unified_backtest_for_intent",
    "engine_result_from_fitness",
    "_gbm_fitness_from_intent",
]
