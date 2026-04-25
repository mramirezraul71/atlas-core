from __future__ import annotations

from datetime import datetime, timezone

from atlas_scanner.config import SCORING_CONFIG
from atlas_scanner.config_loader import ScanConfig
from atlas_scanner.models import ScanSnapshot, SymbolSnapshot

OFFLINE_REFERENCE_DATETIME = datetime(2026, 1, 1, tzinfo=timezone.utc)

OFFLINE_DEFAULT_SYMBOLS: tuple[str, ...] = (
    "SPX",
    "NDX",
    "SPY",
    "QQQ",
    "IWM",
    "DIA",
)

OFFLINE_EXTENDED_SYMBOLS: tuple[str, ...] = (
    *OFFLINE_DEFAULT_SYMBOLS,
    "TINY_CO",
    "WIDESP",
    "LOWVOL",
    "HIIV",
)

_OFFLINE_SNAPSHOT_BY_SYMBOL: dict[str, SymbolSnapshot] = {
    "SPX": SymbolSnapshot(
        symbol="SPX",
        asset_type="index",
        base_currency="USD",
        ref_price=5100.0,
        volatility_lookback=0.19,
        liquidity_score=0.95,
        meta={
            "volume_20d": 3_800_000,
            "bid_ask_spread": 0.02,
            "event_risk": False,
        },
    ),
    "NDX": SymbolSnapshot(
        symbol="NDX",
        asset_type="index",
        base_currency="USD",
        ref_price=18150.0,
        volatility_lookback=0.24,
        liquidity_score=0.90,
        meta={
            "volume_20d": 2_400_000,
            "bid_ask_spread": 0.03,
            "event_risk": False,
        },
    ),
    "SPY": SymbolSnapshot(
        symbol="SPY",
        asset_type="etf",
        base_currency="USD",
        ref_price=515.0,
        volatility_lookback=0.18,
        liquidity_score=0.98,
        meta={
            "volume_20d": 95_000_000,
            "bid_ask_spread": 0.01,
            "event_risk": False,
        },
    ),
    "QQQ": SymbolSnapshot(
        symbol="QQQ",
        asset_type="etf",
        base_currency="USD",
        ref_price=440.0,
        volatility_lookback=0.22,
        liquidity_score=0.97,
        meta={
            "volume_20d": 58_000_000,
            "bid_ask_spread": 0.01,
            "event_risk": False,
        },
    ),
    "IWM": SymbolSnapshot(
        symbol="IWM",
        asset_type="etf",
        base_currency="USD",
        ref_price=205.0,
        volatility_lookback=0.26,
        liquidity_score=0.92,
        meta={
            "volume_20d": 35_000_000,
            "bid_ask_spread": 0.02,
            "event_risk": False,
        },
    ),
    "DIA": SymbolSnapshot(
        symbol="DIA",
        asset_type="etf",
        base_currency="USD",
        ref_price=390.0,
        volatility_lookback=0.16,
        liquidity_score=0.91,
        meta={
            "volume_20d": 5_300_000,
            "bid_ask_spread": 0.02,
            "event_risk": False,
        },
    ),
    "TINY_CO": SymbolSnapshot(
        symbol="TINY_CO",
        asset_type="equity",
        base_currency="USD",
        ref_price=4.8,
        volatility_lookback=0.48,
        liquidity_score=0.20,
        meta={
            "volume_20d": 45_000,
            "bid_ask_spread": 0.25,
            "event_risk": True,
        },
    ),
    "WIDESP": SymbolSnapshot(
        symbol="WIDESP",
        asset_type="equity",
        base_currency="USD",
        ref_price=23.5,
        volatility_lookback=0.34,
        liquidity_score=0.35,
        meta={
            "volume_20d": 120_000,
            "bid_ask_spread": 0.60,
            "event_risk": True,
        },
    ),
    "LOWVOL": SymbolSnapshot(
        symbol="LOWVOL",
        asset_type="equity",
        base_currency="USD",
        ref_price=82.0,
        volatility_lookback=0.09,
        liquidity_score=0.73,
        meta={
            "volume_20d": 1_450_000,
            "bid_ask_spread": 0.05,
            "event_risk": False,
        },
    ),
    "HIIV": SymbolSnapshot(
        symbol="HIIV",
        asset_type="equity",
        base_currency="USD",
        ref_price=61.0,
        volatility_lookback=0.62,
        liquidity_score=0.42,
        meta={
            "volume_20d": 780_000,
            "bid_ask_spread": 0.18,
            "event_risk": True,
        },
    ),
}


def _symbols_for_universe(universe_name: str) -> tuple[str, ...]:
    if universe_name == "default":
        return OFFLINE_DEFAULT_SYMBOLS
    return OFFLINE_EXTENDED_SYMBOLS


def _build_snapshot_id(config: ScanConfig, symbol_count: int) -> str:
    return f"offline-{config.universe_name}-{symbol_count}"


def build_offline_snapshot(
    config: ScanConfig,
    symbols: tuple[SymbolSnapshot, ...] | None = None,
) -> ScanSnapshot:
    if symbols is None:
        selected = _symbols_for_universe(config.universe_name)
        snapshot_symbols = tuple(_OFFLINE_SNAPSHOT_BY_SYMBOL[name] for name in selected)
    else:
        snapshot_symbols = symbols

    return ScanSnapshot(
        snapshot_id=_build_snapshot_id(config=config, symbol_count=len(snapshot_symbols)),
        created_at=OFFLINE_REFERENCE_DATETIME.isoformat(),
        universe_name=config.universe_name,
        symbols=snapshot_symbols,
        config_version=SCORING_CONFIG.config_version,
        meta={
            "config": config,
            "reference_datetime": OFFLINE_REFERENCE_DATETIME,
            "data_source_path": ("mem",),
        },
    )

