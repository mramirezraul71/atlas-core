from __future__ import annotations

from atlas_code_quant.context.market_regime_classifier import MarketRegimeClassifier


def test_classifies_risk_extreme_when_event_high_vol_and_low_liquidity() -> None:
    classifier = MarketRegimeClassifier()

    payload = classifier.classify(
        {
            "symbol": "TSLA",
            "direction": "alcista",
            "timeframe": "15m",
            "selection_score": 58.0,
            "local_win_rate_pct": 49.0,
            "predicted_move_pct": 3.1,
            "iv_rank": 82.0,
            "iv_hv_ratio": 1.48,
            "liquidity_score": 0.41,
            "event_near": True,
            "regime": "BULL",
        }
    )

    assert payload["states"]["high_volatility"] is True
    assert payload["states"]["macro_event"] is True
    assert payload["states"]["risk_extreme"] is True
    assert payload["primary_regime"] in {"risk_extreme", "low_liquidity", "high_volatility", "macro_event"}


def test_classifies_trending_when_strength_and_regime_align() -> None:
    classifier = MarketRegimeClassifier()

    payload = classifier.classify(
        {
            "symbol": "MSFT",
            "direction": "alcista",
            "timeframe": "4h",
            "selection_score": 82.0,
            "local_win_rate_pct": 61.0,
            "predicted_move_pct": 1.8,
            "relative_strength_pct": 4.2,
            "liquidity_score": 0.91,
            "iv_rank": 36.0,
            "iv_hv_ratio": 1.0,
            "regime": "BULL",
        }
    )

    assert payload["states"]["trending"] is True
    assert payload["confidence_pct"] > 60.0
