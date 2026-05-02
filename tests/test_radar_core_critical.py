from __future__ import annotations

from pathlib import Path

import pandas as pd
import pytest

from modules.atlas_radar_kalshi.brain import LLMReading, RadarBrain
from modules.atlas_radar_kalshi.calibration import append_outcome, refit_from_disk_temporal
from modules.atlas_radar_kalshi.executor_v2 import KalshiExecutorV2
from modules.atlas_radar_kalshi.exit_manager import ExitManager, Position
from modules.atlas_radar_kalshi.gating import GateConfig, Gating
from modules.atlas_radar_kalshi.risk_engine import RiskEngine, RiskLimits
from modules.atlas_radar_kalshi.scanner import OrderBookSnapshot
from modules.atlas_radar_kalshi.signals import SignalReadout


def test_gating_uses_executable_side_prices() -> None:
    g = Gating(
        GateConfig(
            edge_net_min=0.01,
            confidence_min=0.1,
            spread_max_ticks=10,
            min_depth_yes=1,
            min_depth_no=1,
            cooldown_seconds=0,
        )
    )
    r = SignalReadout(
        p_ensemble=0.48,
        confidence=0.9,
        spread_ticks=1,
        depth_yes=100,
        depth_no=100,
        liquidity_score=0.9,
    )
    out = g.evaluate(
        "T1",
        r,
        p_market=0.5,
        quote_age_ms=10,
        latency_ms=0,
        p_market_yes=0.52,
        p_market_no=0.47,
    )
    assert out.accepted
    assert out.side == "NO"
    assert out.price_cents == 47


def test_risk_on_close_updates_balance_and_recovers_safe_mode() -> None:
    risk = RiskEngine(
        RiskLimits(
            max_consecutive_losses=2,
            max_open_positions=5,
            kelly_fraction=0.25,
        )
    )
    risk.update_balance(100_000)
    risk.state.safe_mode = True
    risk.state.safe_mode_reason = "old"
    risk.on_order("M1", 5_000)
    risk.on_close("M1", 5_000, 500)
    assert risk.state.balance_cents == 100_500
    assert risk.state.safe_mode is False
    assert risk.state.safe_mode_reason == ""


def test_risk_manual_recover_sets_probation() -> None:
    risk = RiskEngine(RiskLimits(max_consecutive_losses=5))
    risk.update_balance(100_000)
    risk.state.safe_mode = True
    ok = risk.request_safe_mode_recovery(probation_seconds=120)
    assert ok is True
    assert risk.state.safe_mode is False
    assert risk.state.probation_until_ts > 0


def test_executor_idempotency_subsecond_changes_bucket() -> None:
    coid_fast = KalshiExecutorV2.make_client_order_id("T1", "YES", 55, 3, ts_bucket_ms=10)
    coid_slow = KalshiExecutorV2.make_client_order_id("T1", "YES", 55, 3, ts_bucket_ms=1000)
    assert coid_fast.startswith("radar-")
    assert coid_slow.startswith("radar-")
    assert coid_fast != coid_slow


def test_exit_manager_forced_exit() -> None:
    mgr = ExitManager()
    mgr.open(
        Position(
            ticker="T1",
            side="YES",
            entry_price=50,
            size=2,
            entry_ts=0,
            edge_at_entry=0.05,
            target_price=55,
            stop_price=45,
        )
    )
    sig = mgr.evaluate("T1", current_price=49, current_edge=-0.01, forced=True)
    assert sig.should_exit is True
    assert sig.reason == "forced"


def test_calibration_refit_temporal_split(tmp_path: Path) -> None:
    path = tmp_path / "radar_calibration.jsonl"
    for i in range(120):
        p = 0.2 + (0.6 * (i / 119))
        y = 1 if p > 0.5 else 0
        append_outcome(path, p, y)
    cal, stats = refit_from_disk_temporal(path, min_samples=50, min_holdout=20, train_ratio=0.8)
    assert stats["updated"] is True
    assert stats["test_size"] >= 20
    pred = cal.predict(0.7)
    assert 0.0 <= pred <= 1.0


@pytest.mark.asyncio
async def test_brain_uses_executable_side_for_edge() -> None:
    brain = RadarBrain()
    brain.settings.edge_threshold = 0.01

    async def _fake_ollama_prompt(prompt: str) -> LLMReading:
        return LLMReading(p_yes=0.55, confidence=0.9, rationale="ok")

    brain._call_ollama_prompt = _fake_ollama_prompt  # type: ignore[method-assign]
    brain._markov_p_yes = lambda hist: 0.55  # type: ignore[method-assign]
    brain._monte_carlo = lambda p_mix, hist: 0.50  # type: ignore[method-assign]

    book = OrderBookSnapshot(
        market_ticker="T1",
        yes_bids=[(47, 10)],
        yes_asks=[(53, 10)],
        no_bids=[(46, 10)],
        no_asks=[(48, 10)],
    )
    hist = pd.DataFrame({"yes_mid": [0.5] * 35})
    dec = await brain.evaluate("T1", book, hist)
    assert dec.side == "NO"
    assert dec.p_market_no == 0.48
    assert dec.edge > 0
