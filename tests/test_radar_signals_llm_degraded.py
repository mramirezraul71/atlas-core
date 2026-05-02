"""Ensemble: cuando la confianza LLM es muy baja, no debe hundir el gating."""

from __future__ import annotations

import pandas as pd

from modules.atlas_radar_kalshi.scanner import OrderBookSnapshot
from modules.atlas_radar_kalshi.signals import SignalEnsemble


def test_llm_degraded_keeps_reasonable_confidence() -> None:
    book = OrderBookSnapshot(
        market_ticker="X",
        yes_bids=[(48, 100)],
        yes_asks=[(52, 100)],
        no_bids=[(48, 100)],
        no_asks=[(52, 100)],
    )
    hist = pd.DataFrame({"yes_mid": [0.5] * 60})
    out = SignalEnsemble().evaluate(book, hist, p_llm=0.5, llm_confidence=0.1)
    assert out.confidence > 0.30


def test_llm_healthy_uses_full_ensemble_weights() -> None:
    book = OrderBookSnapshot(
        market_ticker="X",
        yes_bids=[(48, 100)],
        yes_asks=[(52, 100)],
        no_bids=[(48, 100)],
        no_asks=[(52, 100)],
    )
    hist = pd.DataFrame({"yes_mid": [0.5] * 60})
    out = SignalEnsemble().evaluate(book, hist, p_llm=0.55, llm_confidence=0.85)
    assert out.confidence >= 0.4
