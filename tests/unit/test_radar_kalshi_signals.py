"""Tests del :mod:`modules.atlas_radar_kalshi.signals` y ``calibration``.

Cubre:
- ``EnsembleWeights.normalized()``.
- ``SignalEnsemble.evaluate`` produce probabilidades en [0.01, 0.99].
- Microprice coherente con el lado dominante del libro.
- Calibrador identidad (n < min_samples) → no toca p.
- Calibrador Platt → monotonía y rango [0, 1].
"""
from __future__ import annotations

import math

import numpy as np
import pandas as pd
import pytest

from modules.atlas_radar_kalshi.calibration import Calibrator
from modules.atlas_radar_kalshi.scanner import OrderBookSnapshot
from modules.atlas_radar_kalshi.signals import (
    EnsembleWeights,
    SignalEnsemble,
)


def _book(yes_bid: int = 50, yes_ask: int = 52,
          depth: int = 200) -> OrderBookSnapshot:
    return OrderBookSnapshot(
        market_ticker="DEMO",
        yes_bids=[(yes_bid, depth)],
        yes_asks=[(yes_ask, depth)],
        no_bids=[(100 - yes_ask, depth)],
        no_asks=[(100 - yes_bid, depth)],
    )


def _hist(n: int = 60, base: float = 0.5) -> pd.DataFrame:
    rng = np.random.default_rng(0)
    yes_mid = np.clip(base + 0.01 * rng.standard_normal(n), 0.05, 0.95)
    return pd.DataFrame({"yes_mid": yes_mid})


# ---------------------------------------------------------------------------
class TestEnsembleWeights:
    def test_normalized_sums_to_one(self) -> None:
        w = EnsembleWeights(0.6, 0.6, 0.6, 0.6).normalized()
        s = w.micro + w.markov + w.llm + w.momentum
        assert math.isclose(s, 1.0, abs_tol=1e-9)

    def test_normalized_zero_falls_back_uniform(self) -> None:
        w = EnsembleWeights(0, 0, 0, 0).normalized()
        assert w.micro == w.markov == w.llm == w.momentum == 0.25


# ---------------------------------------------------------------------------
class TestEvaluate:
    def test_probability_in_bounds(self) -> None:
        ens = SignalEnsemble()
        out = ens.evaluate(_book(), _hist(), p_llm=0.7, llm_confidence=0.6)
        assert 0.01 <= out.p_ensemble <= 0.99
        assert 0.0 <= out.confidence <= 1.0
        assert out.spread_ticks == 2
        assert out.depth_yes > 0 and out.depth_no > 0

    def test_micro_skewed_to_high_yes(self) -> None:
        # libro inclinado fuerte hacia YES (precio 80¢)
        ens = SignalEnsemble()
        out = ens.evaluate(_book(yes_bid=78, yes_ask=82), _hist(base=0.8),
                           p_llm=0.85, llm_confidence=0.8)
        assert out.p_micro > 0.6

    def test_micro_skewed_to_low_yes(self) -> None:
        ens = SignalEnsemble()
        out = ens.evaluate(_book(yes_bid=18, yes_ask=22), _hist(base=0.2),
                           p_llm=0.15, llm_confidence=0.8)
        assert out.p_micro < 0.4

    def test_no_history_keeps_markov_neutral(self) -> None:
        ens = SignalEnsemble()
        out = ens.evaluate(_book(), pd.DataFrame({"yes_mid": []}),
                           p_llm=0.5, llm_confidence=0.5)
        assert out.p_markov == 0.5

    def test_high_agreement_high_confidence(self) -> None:
        ens = SignalEnsemble()
        # todas las señales en el mismo lado → agreement alto
        agree = ens.evaluate(_book(yes_bid=70, yes_ask=72), _hist(base=0.7),
                             p_llm=0.7, llm_confidence=0.9)
        # señales dispersas → agreement bajo
        disagree = ens.evaluate(_book(yes_bid=70, yes_ask=72), _hist(base=0.3),
                                p_llm=0.2, llm_confidence=0.2)
        assert agree.confidence > disagree.confidence


# ---------------------------------------------------------------------------
class TestCalibrator:
    def test_identity_when_under_min_samples(self) -> None:
        cal = Calibrator(method="platt", min_samples=50)
        cal.fit([0.5, 0.6], [0, 1])
        # no se entrena: predict ≡ identity
        for p in [0.1, 0.5, 0.9]:
            assert cal.predict(p) == pytest.approx(p, abs=1e-9)

    def test_platt_fit_monotonic_and_bounded(self) -> None:
        rng = np.random.default_rng(42)
        # generamos un dataset con relación monótona p_raw vs outcome
        p_raw = rng.uniform(0.1, 0.9, size=400)
        outcomes = (rng.uniform(size=400) < p_raw).astype(int)
        cal = Calibrator(method="platt", min_samples=50).fit(p_raw, outcomes)
        assert cal._fitted is True
        ys = [cal.predict(p) for p in [0.1, 0.3, 0.5, 0.7, 0.9]]
        # rango
        for y in ys:
            assert 0.0 <= y <= 1.0
        # monotonía no estricta (Platt es estrictamente monótona)
        assert ys == sorted(ys)

    def test_to_dict_serializable(self) -> None:
        cal = Calibrator()
        d = cal.to_dict()
        assert {"method", "fitted", "a", "b", "n_samples"} <= set(d)
