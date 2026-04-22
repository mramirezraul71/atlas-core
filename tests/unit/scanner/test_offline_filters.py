from __future__ import annotations

from atlas_scanner.filters.offline import (
    event_risk_filter,
    liquidity_filter,
    tradability_filter,
)
from atlas_scanner.models import SymbolSnapshot


def _symbol(
    name: str,
    *,
    ref_price: float,
    liquidity_score: float,
    meta: dict[str, object],
) -> SymbolSnapshot:
    return SymbolSnapshot(
        symbol=name,
        asset_type="equity",
        base_currency="USD",
        ref_price=ref_price,
        volatility_lookback=0.20,
        liquidity_score=liquidity_score,
        meta=meta,
    )


def test_liquidity_filter_filters_by_volume_and_spread_and_score() -> None:
    snapshots = (
        _symbol(
            "PASS_A",
            ref_price=100.0,
            liquidity_score=0.70,
            meta={"volume_20d": 1_000_000, "bid_ask_spread": 0.02},
        ),
        _symbol(
            "FAIL_VOLUME",
            ref_price=100.0,
            liquidity_score=0.80,
            meta={"volume_20d": 10_000, "bid_ask_spread": 0.01},
        ),
        _symbol(
            "FAIL_SPREAD",
            ref_price=100.0,
            liquidity_score=0.80,
            meta={"volume_20d": 2_000_000, "bid_ask_spread": 0.80},
        ),
        _symbol(
            "PASS_B",
            ref_price=80.0,
            liquidity_score=0.65,
            meta={"volume_20d": 500_000, "bid_ask_spread": 0.03},
        ),
    )

    filtered = liquidity_filter(
        snapshots=snapshots,
        min_volume_20d=100_000,
        max_bid_ask_spread=0.05,
        min_liquidity_score=0.60,
    )

    assert tuple(snapshot.symbol for snapshot in filtered) == ("PASS_A", "PASS_B")


def test_liquidity_filter_excludes_missing_fields() -> None:
    snapshots = (
        _symbol(
            "MISSING_VOLUME",
            ref_price=100.0,
            liquidity_score=0.90,
            meta={"bid_ask_spread": 0.01},
        ),
        _symbol(
            "MISSING_SPREAD",
            ref_price=100.0,
            liquidity_score=0.90,
            meta={"volume_20d": 2_000_000},
        ),
    )

    filtered = liquidity_filter(
        snapshots=snapshots,
        min_volume_20d=100_000,
        max_bid_ask_spread=0.05,
        min_liquidity_score=0.60,
    )
    assert filtered == ()


def test_tradability_filter_min_price_only() -> None:
    snapshots = (
        _symbol("LOW", ref_price=2.0, liquidity_score=0.9, meta={}),
        _symbol("OK1", ref_price=12.0, liquidity_score=0.9, meta={}),
        _symbol("OK2", ref_price=55.0, liquidity_score=0.9, meta={}),
    )

    filtered = tradability_filter(
        snapshots=snapshots,
        min_ref_price=10.0,
        max_ref_price=None,
    )
    assert tuple(snapshot.symbol for snapshot in filtered) == ("OK1", "OK2")


def test_tradability_filter_min_and_max_price() -> None:
    snapshots = (
        _symbol("LOW", ref_price=4.0, liquidity_score=0.9, meta={}),
        _symbol("MID", ref_price=25.0, liquidity_score=0.9, meta={}),
        _symbol("HIGH", ref_price=300.0, liquidity_score=0.9, meta={}),
    )

    filtered = tradability_filter(
        snapshots=snapshots,
        min_ref_price=10.0,
        max_ref_price=100.0,
    )
    assert tuple(snapshot.symbol for snapshot in filtered) == ("MID",)


def test_event_risk_filter_respects_max_threshold() -> None:
    snapshots = (
        _symbol("SAFE", ref_price=50.0, liquidity_score=0.8, meta={"event_risk": 0}),
        _symbol("RISKY", ref_price=50.0, liquidity_score=0.8, meta={"event_risk": 1}),
        _symbol("MID", ref_price=50.0, liquidity_score=0.8, meta={"event_risk": 0.4}),
    )

    filtered = event_risk_filter(snapshots=snapshots, max_event_risk=0.5)
    assert tuple(snapshot.symbol for snapshot in filtered) == ("SAFE", "MID")


def test_event_risk_filter_excludes_missing_risk() -> None:
    snapshots = (
        _symbol("MISSING", ref_price=50.0, liquidity_score=0.8, meta={}),
        _symbol("HAS", ref_price=50.0, liquidity_score=0.8, meta={"event_risk": 0}),
    )

    filtered = event_risk_filter(snapshots=snapshots, max_event_risk=0.5)
    assert tuple(snapshot.symbol for snapshot in filtered) == ("HAS",)

