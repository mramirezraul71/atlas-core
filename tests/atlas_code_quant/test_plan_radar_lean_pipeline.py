"""Plan Radar → Quant → LEAN: contrato intake, motor unificado, puente riesgo/visión."""

from __future__ import annotations

from unittest.mock import patch

import pytest

from atlas_code_quant.backtest.internal_gbm_simulator import LeanSimulator, SimConfig
from atlas_code_quant.backtest.pipeline_bridge import (
    fitness_to_engine_result,
    run_risk_preview_for_fitness,
    run_vision_preview_for_opportunity,
)
from atlas_code_quant.backtest.unified_engine import (
    run_unified_backtest_for_intent,
)
from atlas_code_quant.intake.opportunity import RadarOpportunityInternal, from_radar_payload
from atlas_code_quant.lean.config import LeanAdapterConfig, load_config_from_env
from atlas_code_quant.lean.runner.launcher import (
    ERROR_PROCESS_FAILED,
    StrategyFitnessResult,
)
from atlas_code_quant.strategies.factory.dispatch import build_strategies_for_opportunity
from atlas_code_quant.strategies.options.intent import StrategyIntent


def test_f0_contract_payload_maps_to_internal() -> None:
    """Payload alineado con docs/RADAR_TO_QUANT_CONTRACT.md + campos intake."""
    raw = {
        "symbol": "SPY",
        "asset_class": "optionable_equity",
        "direction": "long",
        "score": 82.4,
        "classification": "high_conviction",
        "timeframe": "intraday",
        "has_options": True,
        "criteria_passed": ["liquidity", "trend"],
        "snapshot": {"signal_strength_pct": 77.1, "price": 512.32},
        "degradations_active": [],
        "source": "quant",
        "trace_id": "radar-contract-1",
        "timestamp": "2026-05-01T12:00:00+00:00",
        "horizon_min": 240,
        "optionable": True,
    }
    opp = from_radar_payload(raw)
    assert opp.symbol == "SPY"
    assert opp.score == pytest.approx(82.4)
    assert opp.direction == "long"
    assert opp.trace_id == "radar-contract-1"


def test_f0_gbm_baseline_generates_trades() -> None:
    sim = LeanSimulator(symbols=["SPY"], config=SimConfig(random_seed=7))
    ohlcv = sim.generate_historical_data("SPY", years=0.02)  # type: ignore[arg-type]
    trades_df, _ = sim.run_atlas_strategy("SPY", ohlcv)
    assert len(ohlcv) > 10
    assert "pnl" in trades_df.columns or trades_df.empty


def _spy_intent() -> StrategyIntent:
    opp = RadarOpportunityInternal.from_payload(
        {
            "symbol": "SPY",
            "asset_class": "etf",
            "sector": None,
            "optionable": True,
            "score": 80.0,
            "classification": "high_conviction",
            "direction": "long",
            "horizon_min": 240,
            "snapshot": {},
            "degradations_active": [],
            "source": "quant",
            "trace_id": "plan-pipeline",
            "timestamp": "2026-05-01T12:00:00+00:00",
        }
    )
    intents = build_strategies_for_opportunity(opp)
    assert intents
    return intents[0]


def test_unified_disabled_matches_f13(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.delenv("ATLAS_LEAN_ENABLED", raising=False)
    intent = _spy_intent()
    res = run_unified_backtest_for_intent(intent)
    assert res.success is False
    assert res.error_code == "LEAN_DISABLED"


def test_unified_cli_degrades_to_gbm(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv("ATLAS_LEAN_ENABLED", "true")
    monkeypatch.setenv("ATLAS_LEAN_USE_CLI", "true")
    monkeypatch.setenv("ATLAS_LEAN_PROJECT_ROOT", "C:\\nonexistent_lean_project")
    monkeypatch.setenv("ATLAS_LEAN_ALGORITHM", "AtlasMinimal")
    monkeypatch.setenv("ATLAS_LEAN_DEGRADE_TO_GBM", "true")
    monkeypatch.delenv("ATLAS_LEAN_CLI", raising=False)
    intent = _spy_intent()
    cfg = load_config_from_env()
    assert cfg.is_cli_ready is True
    with patch(
        "atlas_code_quant.backtest.unified_engine.run_lean_cli_backtest",
        return_value=StrategyFitnessResult(
            success=False,
            mode="external",
            error_code=ERROR_PROCESS_FAILED,
            error_message="lean failed",
            config_snapshot=cfg.to_dict(),
        ),
    ):
        out = run_unified_backtest_for_intent(intent, config=cfg)
    assert out.success is True
    assert out.mode == "mock"
    assert (out.config_snapshot or {}).get("degraded_from") == "lean_cli"
    assert (out.config_snapshot or {}).get("engine") == "gbm"


def test_fitness_to_engine_result_and_risk_vision() -> None:
    fit = StrategyFitnessResult(
        success=True,
        mode="mock",
        sharpe=1.2,
        win_rate=55.0,
        max_drawdown=-8.0,
        total_return=12.0,
        expectancy=0.2,
        num_orders=2,
        config_snapshot={"engine": "gbm"},
    )
    dto = fitness_to_engine_result(fit)
    assert dto.success and dto.engine == "gbm"
    risk = run_risk_preview_for_fitness(fit, orders_in_last_minute=1)
    assert risk.ok is True
    opp = RadarOpportunityInternal.from_payload(
        {
            "symbol": "QQQ",
            "asset_class": "etf",
            "sector": None,
            "optionable": True,
            "score": 75.0,
            "classification": "watchlist",
            "direction": "long",
            "horizon_min": 60,
            "snapshot": {},
            "degradations_active": [],
            "source": "quant",
            "trace_id": "v1",
            "timestamp": "2026-05-01T12:00:00+00:00",
        }
    )
    vout = run_vision_preview_for_opportunity(opp, requires_visual_confirmation=False)
    assert vout.decision.value == "allow"


def test_load_config_cli_flags(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv("ATLAS_LEAN_ENABLED", "true")
    monkeypatch.setenv("ATLAS_LEAN_USE_CLI", "true")
    monkeypatch.setenv("ATLAS_LEAN_PROJECT_ROOT", "D:\\lean\\proj")
    monkeypatch.setenv("ATLAS_LEAN_ALGORITHM", "Main")
    monkeypatch.setenv("ATLAS_LEAN_DEGRADE_TO_GBM", "true")
    monkeypatch.setenv("ATLAS_LEAN_CLI", "lean.cmd")
    cfg = load_config_from_env()
    assert cfg.use_cli is True
    assert cfg.project_root == "D:\\lean\\proj"
    assert cfg.algorithm_name == "Main"
    assert cfg.degrade_to_gbm is True
    assert cfg.lean_cli_executable == "lean.cmd"
