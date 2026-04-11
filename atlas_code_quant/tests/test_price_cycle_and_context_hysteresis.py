"""Ciclos en precio y histéresis del régimen contextual."""
from __future__ import annotations

import math

import numpy as np
import pytest

from atlas_code_quant.context.market_regime_classifier import MarketRegimeClassifier
from atlas_code_quant.context.price_cycle_analysis import analyze_price_cycles


def test_analyze_price_cycles_insufficient_data() -> None:
    out = analyze_price_cycles([100.0, 101.0, 99.5])
    assert out["available"] is False


def test_analyze_price_cycles_detects_structure() -> None:
    t = np.arange(120, dtype=float)
    # Precio con componente cíclica suave + deriva
    price = 100.0 + 0.3 * t + 1.2 * np.sin(2 * math.pi * t / 20.0)
    out = analyze_price_cycles(price.tolist(), min_lag=3, max_lag=40)
    assert out["available"] is True
    assert out["n"] == 120
    assert 0.0 <= out["hurst_exponent"] <= 1.0
    assert out["dominant_lag_bars"] >= 0


def test_context_regime_hysteresis_holds_primary_until_margin() -> None:
    clf = MarketRegimeClassifier()
    clf._hyst_margin = 15.0
    clf._strong_score = 90.0

    p1 = clf.classify(
        {
            "symbol": "AAA",
            "direction": "alcista",
            "timeframe": "1h",
            "selection_score": 70.0,
            "local_win_rate_pct": 55.0,
            "predicted_move_pct": 1.0,
            "liquidity_score": 0.9,
            "iv_rank": 30.0,
            "iv_hv_ratio": 1.0,
            "regime": "BULL",
            "relative_strength_pct": 5.0,
        }
    )
    assert p1["context_hysteresis_applied"] is False
    first_primary = p1["primary_regime"]

    p2 = clf.classify(
        {
            "symbol": "BBB",
            "direction": "bajista",
            "timeframe": "1h",
            "selection_score": 70.0,
            "local_win_rate_pct": 55.0,
            "predicted_move_pct": 0.5,
            "liquidity_score": 0.9,
            "iv_rank": 30.0,
            "iv_hv_ratio": 1.0,
            "regime": "RANGE",
        }
    )
    # Sin salto fuerte de score, se mantiene el régimen estable previo
    assert p2["context_hysteresis_applied"] is True
    assert p2["primary_regime"] == first_primary


def test_decision_gate_cycle_soft_degrades_when_enabled(monkeypatch: pytest.MonkeyPatch) -> None:
    from atlas_code_quant.config.settings import settings
    from atlas_code_quant.context.market_context_engine import MarketContextEngine

    monkeypatch.setattr(settings, "context_cycle_soft_gate", True, raising=False)
    monkeypatch.setattr(settings, "context_cycle_soft_gate_max_regime_confidence", 55.0, raising=False)

    eng = MarketContextEngine()
    regime = {
        "states": {
            "risk_extreme": False,
            "low_liquidity": False,
            "macro_event": False,
            "trending": True,
            "sideways": False,
            "high_volatility": False,
        },
        "confidence_pct": 45.0,
        "primary_regime": "trending",
        "price_cycles": {"available": True, "cycle_hint": "noise_like"},
    }
    risk = {"clarity_score_pct": 75.0, "adverse_conditions": []}
    gate = eng._decision_gate(regime=regime, risk_assessment=risk, advisory={}, premarket={"acceptance_state": "accepted"})
    assert gate["degraded"] is True
    assert gate["blocked"] is False
    assert any("ciclos" in r for r in gate["reasons"])


def test_decision_gate_cycle_soft_skips_when_macro_event(monkeypatch: pytest.MonkeyPatch) -> None:
    from atlas_code_quant.config.settings import settings
    from atlas_code_quant.context.market_context_engine import MarketContextEngine

    monkeypatch.setattr(settings, "context_cycle_soft_gate", True, raising=False)

    eng = MarketContextEngine()
    regime = {
        "states": {
            "risk_extreme": False,
            "low_liquidity": False,
            "macro_event": True,
            "trending": False,
            "sideways": True,
            "high_volatility": False,
        },
        "confidence_pct": 30.0,
        "primary_regime": "sideways",
        "price_cycles": {"available": True, "cycle_hint": "noise_like"},
    }
    risk = {"clarity_score_pct": 75.0, "adverse_conditions": []}
    gate = eng._decision_gate(regime=regime, risk_assessment=risk, advisory={}, premarket={"acceptance_state": "accepted"})
    assert not any("ciclos" in r for r in gate["reasons"])


def test_reset_context_hysteresis() -> None:
    clf = MarketRegimeClassifier()
    clf.classify(
        {
            "symbol": "ZZZ",
            "direction": "alcista",
            "timeframe": "1h",
            "selection_score": 80.0,
            "local_win_rate_pct": 60.0,
            "predicted_move_pct": 1.0,
            "liquidity_score": 0.9,
            "iv_rank": 30.0,
            "iv_hv_ratio": 1.0,
            "regime": "BULL",
            "relative_strength_pct": 4.0,
        }
    )
    clf.reset_hysteresis()
    assert clf._last_primary_label is None
