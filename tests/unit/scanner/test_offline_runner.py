from __future__ import annotations

from dataclasses import replace

from atlas_scanner.config import SCORING_CONFIG
from atlas_scanner.config_loader import (
    OfflineScoringConfig,
    OfflineScoringThresholds,
    OfflineScoringWeights,
    build_scan_config_offline,
)
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


def test_run_offline_scan_returns_ranked_symbols() -> None:
    result = run_offline_scan()

    assert hasattr(result, "ranked_symbols")
    assert len(result.ranked_symbols) == len(result.selected_symbols)
    assert result.meta["ranking_applied"] is True
    assert result.meta["explanations_applied"] is True
    assert result.meta["total_ranked_symbols"] == len(result.ranked_symbols)


def test_run_offline_scan_ranked_symbols_are_sorted() -> None:
    result = run_offline_scan()
    scores = tuple(item.score for item in result.ranked_symbols)
    assert scores == tuple(sorted(scores, reverse=True))


def test_run_offline_scan_ranked_symbols_include_explanations() -> None:
    result = run_offline_scan()
    assert all(item.explanation for item in result.ranked_symbols)


def test_run_offline_scan_populates_candidate_opportunities() -> None:
    result = run_offline_scan()
    assert len(result.candidate_opportunities) == len(result.ranked_symbols)
    assert result.meta["candidate_features_enriched"] is True
    if result.candidate_opportunities:
        first = result.candidate_opportunities[0]
        assert first.symbol == result.ranked_symbols[0].symbol_snapshot.symbol
        assert first.vol_features is not None


def test_run_offline_scan_uses_config_scoring() -> None:
    config = build_scan_config_offline(scoring_config=SCORING_CONFIG, universe_name="default")
    custom_scoring = OfflineScoringConfig(
        weights=OfflineScoringWeights(
            liquidity=0.0,
            price=0.0,
            event_risk=1.0,
            spread=0.0,
        ),
        thresholds=OfflineScoringThresholds(),
    )
    custom_config = replace(config, scoring=custom_scoring)

    result = run_offline_scan(config=custom_config)
    top_symbol = result.ranked_symbols[0].symbol_snapshot.symbol
    assert top_symbol == "SPX"
    assert result.ranked_symbols[0].component_scores["event_risk"] == 1.0

