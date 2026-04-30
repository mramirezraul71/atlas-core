from __future__ import annotations

import sys
from pathlib import Path

import pandas as pd
import pytest

QUANT_ROOT = Path(__file__).resolve().parents[2]
if str(QUANT_ROOT) not in sys.path:
    sys.path.insert(0, str(QUANT_ROOT))

from scanner.asset_classifier import classify_asset  # noqa: E402
from scanner.opportunity_scanner import OpportunityScannerService  # noqa: E402


def test_phase3_fundamental_hard_reject(monkeypatch) -> None:
    monkeypatch.setattr("scanner.opportunity_scanner.settings.scanner_fundamental_enabled", True)
    monkeypatch.setattr("scanner.opportunity_scanner.settings.scanner_onchain_enabled", False)
    monkeypatch.setattr("market_context.fundamental_score.calculate_fundamental_score", lambda _t, sector_medians=None: 5.0)
    svc = OpportunityScannerService()
    ctx = svc._phase3_symbol_gates("MSFT", classify_asset("MSFT"))
    assert ctx["hard_reject"] is True
    assert ctx["reasons"]


def test_phase3_onchain_hard_reject(monkeypatch) -> None:
    monkeypatch.setattr("scanner.opportunity_scanner.settings.scanner_fundamental_enabled", False)
    monkeypatch.setattr("scanner.opportunity_scanner.settings.scanner_onchain_enabled", True)

    def _bad_bundle(_sym: str) -> dict:
        return {
            "gate": {"gate_pass": False, "reasons": ["MVRV extreme"]},
            "bonus": 0.0,
        }

    monkeypatch.setattr("market_context.onchain_metrics.bundle_for_symbol", _bad_bundle)
    svc = OpportunityScannerService()
    ctx = svc._phase3_symbol_gates("BTC/USDT", classify_asset("BTC/USDT"))
    assert ctx["hard_reject"] is True
    assert "MVRV extreme" in ctx["reasons"]


def test_phase3_onchain_bonus_when_pass(monkeypatch) -> None:
    monkeypatch.setattr("scanner.opportunity_scanner.settings.scanner_fundamental_enabled", False)
    monkeypatch.setattr("scanner.opportunity_scanner.settings.scanner_onchain_enabled", True)

    def _ok_bundle(_sym: str) -> dict:
        return {"gate": {"gate_pass": True, "reasons": []}, "bonus": 10.0}

    monkeypatch.setattr("market_context.onchain_metrics.bundle_for_symbol", _ok_bundle)
    svc = OpportunityScannerService()
    ctx = svc._phase3_symbol_gates("ETH/USDT", classify_asset("ETH/USDT"))
    assert ctx["hard_reject"] is False
    assert ctx["onchain_bonus"] == 10.0


def test_cnn_boost_points_align_with_direction() -> None:
    from learning.cnn_lstm_pattern import pattern_boost_points

    pred = {"no_pattern": 0.05, "bullish_pattern": 0.9, "bearish_pattern": 0.05}
    assert pattern_boost_points(pred, 1, 0.65) > 0
    assert pattern_boost_points(pred, -1, 0.65) == 0


def test_prophet_soft_signals_structure(monkeypatch) -> None:
    pytest.importorskip("prophet")
    from forecasting.prophet_detector import prophet_soft_signals

    rng = __import__("numpy").random.default_rng(1)
    y = 100 + __import__("numpy").cumsum(rng.normal(0, 0.2, size=100))
    close = pd.Series(y, index=pd.date_range("2021-01-01", periods=100, freq="D", tz="UTC"))
    sig = prophet_soft_signals(close)
    assert "score_bonus_low_vol" in sig
    assert "breakpoint" in sig
