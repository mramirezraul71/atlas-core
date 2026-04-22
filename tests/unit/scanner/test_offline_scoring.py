from __future__ import annotations

import pytest

from atlas_scanner.config_loader import (
    OfflineScoringConfig,
    OfflineScoringThresholds,
    OfflineScoringWeights,
)
from atlas_scanner.features.gamma import StrikeGamma
from atlas_scanner.models import SymbolSnapshot
from atlas_scanner.scoring.offline import (
    compute_component_weighted_score,
    normalize_value,
    rank_symbols,
    score_symbol,
)


def _snapshot(
    symbol: str,
    *,
    liquidity_score: float,
    ref_price: float,
    event_risk: float = 0.0,
    bid_ask_spread: float = 0.01,
) -> SymbolSnapshot:
    return SymbolSnapshot(
        symbol=symbol,
        asset_type="equity",
        base_currency="USD",
        ref_price=ref_price,
        volatility_lookback=0.20,
        liquidity_score=liquidity_score,
        meta={
            "event_risk": event_risk,
            "bid_ask_spread": bid_ask_spread,
        },
    )


def test_normalize_value_clamps_to_zero_one() -> None:
    assert normalize_value(-10.0, 0.0, 100.0) == 0.0
    assert normalize_value(50.0, 0.0, 100.0) == 0.5
    assert normalize_value(110.0, 0.0, 100.0) == 1.0
    assert normalize_value(5.0, 10.0, 10.0) == 0.0


def test_score_symbol_returns_components_and_score_in_range() -> None:
    result = score_symbol(
        _snapshot(
            "BASE",
            liquidity_score=0.75,
            ref_price=120.0,
            event_risk=0.20,
            bid_ask_spread=0.02,
        )
    )
    assert isinstance(result.score, float)
    assert 0.0 <= result.score <= 1.0
    assert set(result.component_scores.keys()) == {
        "liquidity",
        "price",
        "event_risk",
        "spread",
        "vol",
        "gamma",
    }
    for component in result.component_scores.values():
        assert 0.0 <= component <= 1.0
    assert isinstance(result.explanation, str)
    assert isinstance(result.strengths, tuple)
    assert isinstance(result.penalties, tuple)


def test_score_symbol_penalizes_high_event_risk_and_spread() -> None:
    better = score_symbol(
        _snapshot(
            "BETTER",
            liquidity_score=0.70,
            ref_price=100.0,
            event_risk=0.0,
            bid_ask_spread=0.01,
        )
    )
    worse = score_symbol(
        _snapshot(
            "WORSE",
            liquidity_score=0.70,
            ref_price=100.0,
            event_risk=1.0,
            bid_ask_spread=0.20,
        )
    )
    assert better.score > worse.score


def test_rank_symbols_orders_descending_by_score() -> None:
    ranked = rank_symbols(
        (
            _snapshot("LOW", liquidity_score=0.20, ref_price=8.0, event_risk=1.0, bid_ask_spread=0.20),
            _snapshot("MID", liquidity_score=0.60, ref_price=90.0, event_risk=0.40, bid_ask_spread=0.03),
            _snapshot("HIGH", liquidity_score=0.95, ref_price=150.0, event_risk=0.0, bid_ask_spread=0.01),
        )
    )
    scores = tuple(item.score for item in ranked)
    assert scores == tuple(sorted(scores, reverse=True))
    assert ranked[0].symbol_snapshot.symbol == "HIGH"


def test_rank_symbols_is_stable_on_ties() -> None:
    first = _snapshot("FIRST", liquidity_score=0.50, ref_price=100.0, event_risk=0.5, bid_ask_spread=0.05)
    second = _snapshot("SECOND", liquidity_score=0.50, ref_price=100.0, event_risk=0.5, bid_ask_spread=0.05)
    ranked = rank_symbols((first, second))
    assert tuple(item.symbol_snapshot.symbol for item in ranked) == ("FIRST", "SECOND")
    assert all(item.explanation for item in ranked)


def test_score_symbol_accepts_runtime_scoring_config() -> None:
    snapshot = _snapshot(
        "RUNTIME",
        liquidity_score=0.80,
        ref_price=100.0,
        event_risk=0.20,
        bid_ask_spread=0.02,
    )
    config = OfflineScoringConfig(
        weights=OfflineScoringWeights(liquidity=1.0, price=0.0, event_risk=0.0, spread=0.0),
        thresholds=OfflineScoringThresholds(),
    )
    scored = score_symbol(snapshot, scoring_config=config)
    assert scored.score == scored.component_scores["liquidity"]
    assert scored.explanation != ""


def test_score_symbol_uses_vol_gamma_components_when_available() -> None:
    snapshot = SymbolSnapshot(
        symbol="PRO",
        asset_type="equity",
        base_currency="USD",
        ref_price=200.0,
        volatility_lookback=0.35,
        liquidity_score=0.2,
        meta={
            "event_risk": 1.0,
            "bid_ask_spread": 0.2,
            "iv_current": 40.0,
            "iv_history": [10.0, 20.0, 30.0, 40.0, 50.0],
            "rv_annualized": {"20d": 30.0},
            "net_gamma": -500_000.0,
            "strike_gamma": [
                StrikeGamma(strike=3900.0, call_gamma=100.0, put_gamma=-20.0),
                StrikeGamma(strike=3950.0, call_gamma=300.0, put_gamma=-40.0),
                StrikeGamma(strike=4000.0, call_gamma=150.0, put_gamma=-400.0),
            ],
        },
    )
    scored = score_symbol(snapshot)
    assert scored.component_scores["vol"] > 0.0
    assert scored.component_scores["gamma"] > 0.0
    assert scored.score > scored.component_scores["liquidity"]


def test_component_weighted_score_renormalizes_available_components() -> None:
    config = OfflineScoringConfig()
    both = compute_component_weighted_score(
        vol_score=90.0,
        gamma_score=50.0,
        component_weights=config.component_weights,
    )
    vol_only = compute_component_weighted_score(
        vol_score=90.0,
        gamma_score=None,
        component_weights=config.component_weights,
    )
    none = compute_component_weighted_score(
        vol_score=None,
        gamma_score=None,
        component_weights=config.component_weights,
    )
    assert both == pytest.approx(76.6666666667)
    assert vol_only == 90.0
    assert none == 0.0

