from __future__ import annotations

from atlas_scanner.config import SCORING_CONFIG
from atlas_scanner.config_loader import build_scan_config_offline
from atlas_scanner.fixtures.offline import (
    OFFLINE_DEFAULT_SYMBOLS,
    OFFLINE_EXTENDED_SYMBOLS,
    build_offline_snapshot,
)
from atlas_scanner.universe.offline import select_offline_universe


def test_select_offline_universe_default_uses_default_symbols() -> None:
    config = build_scan_config_offline(scoring_config=SCORING_CONFIG, universe_name="default")
    snapshots = build_offline_snapshot(config=config).symbols

    selected = select_offline_universe(config=config, snapshots=snapshots)
    selected_symbols = tuple(snapshot.symbol for snapshot in selected)

    assert all(symbol in OFFLINE_DEFAULT_SYMBOLS for symbol in selected_symbols)
    assert selected_symbols == OFFLINE_DEFAULT_SYMBOLS
    assert len(selected) == len(OFFLINE_DEFAULT_SYMBOLS)


def test_select_offline_universe_extended_uses_extended_symbols() -> None:
    config = build_scan_config_offline(scoring_config=SCORING_CONFIG, universe_name="extended")
    snapshots = build_offline_snapshot(config=config).symbols

    selected = select_offline_universe(config=config, snapshots=snapshots)
    selected_symbols = tuple(snapshot.symbol for snapshot in selected)

    assert selected_symbols == OFFLINE_EXTENDED_SYMBOLS
    assert len(selected) == len(OFFLINE_EXTENDED_SYMBOLS)


def test_select_offline_universe_other_universe_passthrough() -> None:
    config = build_scan_config_offline(scoring_config=SCORING_CONFIG, universe_name="custom")
    snapshots = build_offline_snapshot(config=config).symbols

    selected = select_offline_universe(config=config, snapshots=snapshots)
    assert selected == snapshots

