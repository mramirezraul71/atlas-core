from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime

from atlas_scanner.config_loader import ScanConfig, build_default_scan_config_offline
from atlas_scanner.filters.offline import (
    event_risk_filter,
    liquidity_filter,
    tradability_filter,
)
from atlas_scanner.fixtures.offline import OFFLINE_REFERENCE_DATETIME, build_offline_snapshot
from atlas_scanner.models import SymbolSnapshot
from atlas_scanner.scoring.offline import ScoredSymbol, rank_symbols
from atlas_scanner.universe.offline import select_offline_universe


@dataclass(frozen=True)
class OfflineScanResult:
    config: ScanConfig
    reference_datetime: datetime
    selected_symbols: tuple[SymbolSnapshot, ...]
    ranked_symbols: tuple[ScoredSymbol, ...]
    universe_name: str
    data_source_path: tuple[str, ...]
    meta: dict[str, object]


def run_offline_scan(
    config: ScanConfig | None = None,
    *,
    min_volume_20d: float | int | None = None,
    max_bid_ask_spread: float | int | None = None,
    min_liquidity_score: float | int | None = None,
    min_ref_price: float | int | None = None,
    max_ref_price: float | int | None = None,
    max_event_risk: float | int | None = None,
) -> OfflineScanResult:
    effective_config = config or build_default_scan_config_offline()
    snapshot = build_offline_snapshot(config=effective_config)
    symbols_universe = select_offline_universe(
        config=effective_config,
        snapshots=snapshot.symbols,
    )

    symbols_filtered: tuple[SymbolSnapshot, ...] = symbols_universe
    filters_applied: list[str] = []

    if (
        min_volume_20d is not None
        and max_bid_ask_spread is not None
        and min_liquidity_score is not None
    ):
        symbols_filtered = liquidity_filter(
            snapshots=symbols_filtered,
            min_volume_20d=min_volume_20d,
            max_bid_ask_spread=max_bid_ask_spread,
            min_liquidity_score=min_liquidity_score,
        )
        filters_applied.append("liquidity")

    if min_ref_price is not None:
        symbols_filtered = tradability_filter(
            snapshots=symbols_filtered,
            min_ref_price=min_ref_price,
            max_ref_price=max_ref_price,
        )
        filters_applied.append("tradability")

    if max_event_risk is not None:
        symbols_filtered = event_risk_filter(
            snapshots=symbols_filtered,
            max_event_risk=max_event_risk,
        )
        filters_applied.append("event_risk")

    ranked_symbols = rank_symbols(symbols_filtered)

    return OfflineScanResult(
        config=effective_config,
        reference_datetime=OFFLINE_REFERENCE_DATETIME,
        selected_symbols=symbols_filtered,
        ranked_symbols=ranked_symbols,
        universe_name=effective_config.universe_name,
        data_source_path=("mem",),
        meta={
            "total_symbols_snapshot": len(snapshot.symbols),
            "total_symbols_universe": len(symbols_universe),
            "total_symbols_final": len(symbols_filtered),
            "ranking_applied": True,
            "total_ranked_symbols": len(ranked_symbols),
            "filters_applied": tuple(filters_applied),
        },
    )

