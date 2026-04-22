from __future__ import annotations

from atlas_scanner.config import SCORING_CONFIG
from atlas_scanner.config_loader import build_scan_config_offline
from atlas_scanner.fixtures.offline import OFFLINE_EXTENDED_SYMBOLS, OFFLINE_REFERENCE_DATETIME
from atlas_scanner.runner.offline import run_offline_scan
from atlas_scanner.universe.offline import select_offline_universe


def test_run_offline_scan_default_config_no_filters() -> None:
    result = run_offline_scan()

    expected_universe = select_offline_universe(
        config=result.config,
        snapshots=result.selected_symbols,
    )

    assert result.reference_datetime == OFFLINE_REFERENCE_DATETIME
    assert result.universe_name == "default"
    assert result.data_source_path == ("mem",)
    assert len(result.selected_symbols) == len(expected_universe)
    assert (
        result.meta["total_symbols_snapshot"]
        >= result.meta["total_symbols_universe"]
        >= result.meta["total_symbols_final"]
    )
    assert result.meta["filters_applied"] == ()


def test_run_offline_scan_with_liquidity_filter() -> None:
    result = run_offline_scan(
        min_volume_20d=30_000_000,
        max_bid_ask_spread=0.02,
        min_liquidity_score=0.95,
    )

    assert "liquidity" in result.meta["filters_applied"]
    for snapshot in result.selected_symbols:
        assert snapshot.meta["volume_20d"] >= 30_000_000
        assert snapshot.meta["bid_ask_spread"] <= 0.02
        assert snapshot.liquidity_score >= 0.95

    observed_symbols = tuple(snapshot.symbol for snapshot in result.selected_symbols)
    assert observed_symbols == ("SPY", "QQQ")


def test_run_offline_scan_with_tradability_and_event_risk() -> None:
    result = run_offline_scan(
        min_ref_price=400.0,
        max_event_risk=0,
    )

    assert result.meta["filters_applied"] == ("tradability", "event_risk")
    for snapshot in result.selected_symbols:
        assert snapshot.ref_price >= 400.0
        assert snapshot.meta["event_risk"] <= 0


def test_run_offline_scan_uses_passed_config() -> None:
    config = build_scan_config_offline(scoring_config=SCORING_CONFIG, universe_name="extended")

    result = run_offline_scan(config=config, min_ref_price=1.0)
    assert result.config.universe_name == "extended"
    assert result.reference_datetime == OFFLINE_REFERENCE_DATETIME
    assert result.data_source_path == ("mem",)
    assert result.meta["total_symbols_universe"] == len(OFFLINE_EXTENDED_SYMBOLS)

