from __future__ import annotations

from dataclasses import replace

from atlas_scanner.api.offline import (
    OfflineScanner,
    scan_offline,
    scan_offline_and_explain,
    scan_offline_top_n,
)
from atlas_scanner.config import SCORING_CONFIG
from atlas_scanner.config_loader import (
    OfflineScoringConfig,
    OfflineScoringThresholds,
    OfflineScoringWeights,
    build_default_scan_config_offline,
    build_scan_config_offline,
)
from atlas_scanner.runner.offline import OfflineScanResult


def test_offline_scanner_scan_uses_default_config_when_none() -> None:
    scanner = OfflineScanner()
    result = scanner.scan()
    default_config = build_default_scan_config_offline()

    assert isinstance(result, OfflineScanResult)
    assert result.config.universe_name == default_config.universe_name
    assert result.ranked_symbols
    assert all(item.explanation for item in result.ranked_symbols)


def test_offline_scanner_scan_respects_custom_config() -> None:
    base = build_scan_config_offline(scoring_config=SCORING_CONFIG, universe_name="extended")
    custom_scoring = OfflineScoringConfig(
        weights=OfflineScoringWeights(liquidity=0.10, price=0.10, event_risk=0.70, spread=0.10),
        thresholds=OfflineScoringThresholds(),
    )
    custom_config = replace(base, scoring=custom_scoring)

    result = OfflineScanner().scan(config=custom_config)
    assert result.config.universe_name == "extended"
    assert result.config.scoring == custom_scoring


def test_offline_scanner_scan_top_limits_results() -> None:
    scanner = OfflineScanner()
    top = scanner.scan_top(top_n=3)
    assert len(top) <= 3
    scores = tuple(item.score for item in top)
    assert scores == tuple(sorted(scores, reverse=True))

    assert scanner.scan_top(top_n=0) == ()
    assert scanner.scan_top(top_n=-2) == ()


def test_scan_offline_and_explain_truncates_ranked_symbols_when_top_n_given() -> None:
    result = scan_offline_and_explain(top_n=2)
    assert isinstance(result, OfflineScanResult)
    assert len(result.ranked_symbols) <= 2
    assert all(item.explanation for item in result.ranked_symbols)
    assert result.meta.get("top_n") == 2


def test_module_level_functions_delegate_to_default_scanner() -> None:
    scanner = OfflineScanner()
    scan_from_object = scanner.scan()
    scan_from_module = scan_offline()
    assert tuple(item.symbol_snapshot.symbol for item in scan_from_module.ranked_symbols) == tuple(
        item.symbol_snapshot.symbol for item in scan_from_object.ranked_symbols
    )

    top_from_object = scanner.scan_top(top_n=2)
    top_from_module = scan_offline_top_n(top_n=2)
    assert tuple(item.symbol_snapshot.symbol for item in top_from_module) == tuple(
        item.symbol_snapshot.symbol for item in top_from_object
    )

