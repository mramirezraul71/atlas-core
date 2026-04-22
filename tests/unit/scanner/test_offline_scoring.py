from __future__ import annotations

from atlas_scanner.models import SymbolSnapshot
from atlas_scanner.scoring.offline import normalize_value, rank_symbols, score_symbol


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
    }
    for component in result.component_scores.values():
        assert 0.0 <= component <= 1.0


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

