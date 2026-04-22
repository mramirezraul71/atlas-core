from __future__ import annotations

from datetime import datetime, timezone

from atlas_scanner.config_loader import build_default_scan_config_offline
from atlas_scanner.fixtures.offline import (
    OFFLINE_DEFAULT_SYMBOLS,
    OFFLINE_REFERENCE_DATETIME,
    build_offline_snapshot,
)
from atlas_scanner.models import ScanSnapshot, SymbolSnapshot


def test_offline_reference_datetime_is_fixed() -> None:
    config = build_default_scan_config_offline()
    snapshot = build_offline_snapshot(config=config)

    assert snapshot.meta["reference_datetime"] == OFFLINE_REFERENCE_DATETIME
    assert OFFLINE_REFERENCE_DATETIME == datetime(2026, 1, 1, tzinfo=timezone.utc)


def test_build_offline_snapshot_default_symbols() -> None:
    config = build_default_scan_config_offline()
    snapshot = build_offline_snapshot(config=config)

    assert isinstance(snapshot, ScanSnapshot)
    assert snapshot.meta["config"] is config
    assert len(snapshot.symbols) == len(OFFLINE_DEFAULT_SYMBOLS)

    observed_symbols = tuple(symbol.symbol for symbol in snapshot.symbols)
    assert observed_symbols == OFFLINE_DEFAULT_SYMBOLS


def test_build_offline_snapshot_custom_symbols() -> None:
    config = build_default_scan_config_offline()
    custom_symbols = (
        SymbolSnapshot(
            symbol="CUSTOM_A",
            asset_type="equity",
            base_currency="USD",
            ref_price=10.0,
            volatility_lookback=0.30,
            liquidity_score=0.50,
        ),
        SymbolSnapshot(
            symbol="CUSTOM_B",
            asset_type="equity",
            base_currency="USD",
            ref_price=20.0,
            volatility_lookback=0.20,
            liquidity_score=0.60,
        ),
    )

    snapshot = build_offline_snapshot(config=config, symbols=custom_symbols)
    assert snapshot.symbols == custom_symbols


def test_offline_snapshots_are_deterministic() -> None:
    config = build_default_scan_config_offline()
    left = build_offline_snapshot(config=config)
    right = build_offline_snapshot(config=config)

    assert left.meta["reference_datetime"] == right.meta["reference_datetime"]
    assert tuple(symbol.symbol for symbol in left.symbols) == tuple(
        symbol.symbol for symbol in right.symbols
    )
    assert left.meta["data_source_path"] == right.meta["data_source_path"]


def test_data_source_path_is_mem() -> None:
    config = build_default_scan_config_offline()
    snapshot = build_offline_snapshot(config=config)

    assert snapshot.meta["data_source_path"] == ("mem",)

