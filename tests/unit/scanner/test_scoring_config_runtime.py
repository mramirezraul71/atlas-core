from __future__ import annotations

from atlas_scanner.config_loader import (
    DEFAULT_OFFLINE_SCORING_CONFIG,
    OfflineScoringConfig,
    OfflineScoringThresholds,
    OfflineScoringWeights,
)
from atlas_scanner.models import SymbolSnapshot
from atlas_scanner.scoring.offline import rank_symbols, score_symbol


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


def test_default_scoring_config_matches_s21_weights_and_thresholds() -> None:
    weights = DEFAULT_OFFLINE_SCORING_CONFIG.weights
    thresholds = DEFAULT_OFFLINE_SCORING_CONFIG.thresholds

    assert weights.liquidity == 0.40
    assert weights.price == 0.20
    assert weights.event_risk == 0.25
    assert weights.spread == 0.15

    assert thresholds.liquidity_lower == 0.0
    assert thresholds.liquidity_upper == 1.0
    assert thresholds.price_lower == 5.0
    assert thresholds.price_upper == 500.0
    assert thresholds.event_risk_lower == 0.0
    assert thresholds.event_risk_upper == 1.0
    assert thresholds.spread_lower == 0.0
    assert thresholds.spread_upper == 0.10


def test_score_symbol_uses_custom_weights() -> None:
    high_liquidity_low_price = _snapshot(
        "LQ",
        liquidity_score=0.95,
        ref_price=20.0,
        event_risk=0.1,
        bid_ask_spread=0.01,
    )
    low_liquidity_high_price = _snapshot(
        "PX",
        liquidity_score=0.30,
        ref_price=450.0,
        event_risk=0.1,
        bid_ask_spread=0.01,
    )

    liquidity_favoring = OfflineScoringConfig(
        weights=OfflineScoringWeights(liquidity=0.90, price=0.05, event_risk=0.03, spread=0.02),
        thresholds=OfflineScoringThresholds(),
    )
    price_favoring = OfflineScoringConfig(
        weights=OfflineScoringWeights(liquidity=0.05, price=0.90, event_risk=0.03, spread=0.02),
        thresholds=OfflineScoringThresholds(),
    )

    rank_liquidity = rank_symbols(
        (high_liquidity_low_price, low_liquidity_high_price),
        scoring_config=liquidity_favoring,
    )
    rank_price = rank_symbols(
        (high_liquidity_low_price, low_liquidity_high_price),
        scoring_config=price_favoring,
    )

    assert rank_liquidity[0].symbol_snapshot.symbol == "LQ"
    assert rank_price[0].symbol_snapshot.symbol == "PX"


def test_score_symbol_uses_custom_thresholds() -> None:
    snapshot = _snapshot(
        "THRESH",
        liquidity_score=0.5,
        ref_price=200.0,
        event_risk=0.2,
        bid_ask_spread=0.02,
    )

    wide_thresholds = OfflineScoringConfig(
        weights=OfflineScoringWeights(),
        thresholds=OfflineScoringThresholds(price_lower=5.0, price_upper=500.0),
    )
    tight_thresholds = OfflineScoringConfig(
        weights=OfflineScoringWeights(),
        thresholds=OfflineScoringThresholds(price_lower=5.0, price_upper=200.0),
    )

    wide_scored = score_symbol(snapshot, scoring_config=wide_thresholds)
    tight_scored = score_symbol(snapshot, scoring_config=tight_thresholds)

    assert tight_scored.component_scores["price"] > wide_scored.component_scores["price"]


def test_rank_symbols_uses_passed_scoring_config() -> None:
    left = _snapshot("LEFT", liquidity_score=0.90, ref_price=20.0, event_risk=0.2, bid_ask_spread=0.01)
    right = _snapshot("RIGHT", liquidity_score=0.20, ref_price=450.0, event_risk=0.2, bid_ask_spread=0.01)

    custom = OfflineScoringConfig(
        weights=OfflineScoringWeights(liquidity=0.0, price=1.0, event_risk=0.0, spread=0.0),
        thresholds=OfflineScoringThresholds(),
    )
    ranked = rank_symbols((left, right), scoring_config=custom)
    assert ranked[0].symbol_snapshot.symbol == "RIGHT"

