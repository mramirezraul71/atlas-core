from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from datetime import datetime

from atlas_scanner.config_loader import ScanConfig, build_default_scan_config_offline
from atlas_scanner.features.builders import (
    GammaFeatureInput,
    VolFeatureInput,
    enrich_candidate_with_features,
)
from atlas_scanner.filters.offline import (
    event_risk_filter,
    liquidity_filter,
    tradability_filter,
)
from atlas_scanner.fixtures.offline import OFFLINE_REFERENCE_DATETIME, build_offline_snapshot
from atlas_scanner.models import ProCandidateOpportunity, SymbolSnapshot
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
    candidate_opportunities: tuple[ProCandidateOpportunity, ...] = ()


def _asset_type_from_snapshot(snapshot: SymbolSnapshot) -> str:
    value = snapshot.asset_type.strip().upper()
    if value == "EQUITY":
        return "STOCK"
    if value in {"INDEX", "ETF", "STOCK", "FUTURE"}:
        return value
    return "OTHER"


def _as_optional_float(value: object) -> float | None:
    if isinstance(value, (int, float)):
        return float(value)
    return None


def _vol_input_from_snapshot(snapshot: SymbolSnapshot) -> VolFeatureInput:
    iv_current = _as_optional_float(snapshot.meta.get("iv_current"))
    if iv_current is None:
        iv_current = float(snapshot.volatility_lookback)

    iv_history_raw = snapshot.meta.get("iv_history")
    iv_history = iv_history_raw if isinstance(iv_history_raw, (list, tuple)) else None

    rv_raw = snapshot.meta.get("rv_annualized")
    rv_annualized = rv_raw if isinstance(rv_raw, Mapping) else None

    return VolFeatureInput(
        iv_current=iv_current,
        iv_history=iv_history,
        rv_annualized=rv_annualized,
    )


def _gamma_input_from_snapshot(snapshot: SymbolSnapshot) -> GammaFeatureInput | None:
    strike_gamma_raw = snapshot.meta.get("strike_gamma")
    strike_gamma = strike_gamma_raw if isinstance(strike_gamma_raw, (list, tuple)) else None
    net_gamma = _as_optional_float(snapshot.meta.get("net_gamma"))
    if strike_gamma is None and net_gamma is None:
        return None
    return GammaFeatureInput(
        strike_gamma=strike_gamma,
        net_gamma=net_gamma,
    )


def _candidate_from_scored(
    scored: ScoredSymbol,
    as_of: datetime,
) -> ProCandidateOpportunity:
    snapshot = scored.symbol_snapshot
    candidate = ProCandidateOpportunity(
        symbol=snapshot.symbol,
        as_of=as_of,
        asset_type=_asset_type_from_snapshot(snapshot),
        total_score=scored.score,
        component_scores=dict(scored.component_scores),
        weights_effective={},
        strengths=scored.strengths,
        penalties=scored.penalties,
        explanation=scored.explanation,
    )
    return enrich_candidate_with_features(
        candidate,
        vol_input=_vol_input_from_snapshot(snapshot),
        gamma_input=_gamma_input_from_snapshot(snapshot),
    )


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

    ranked_symbols = rank_symbols(
        symbols_filtered,
        scoring_config=effective_config.scoring,
    )
    candidate_opportunities = tuple(
        _candidate_from_scored(scored=item, as_of=OFFLINE_REFERENCE_DATETIME)
        for item in ranked_symbols
    )

    return OfflineScanResult(
        config=effective_config,
        reference_datetime=OFFLINE_REFERENCE_DATETIME,
        selected_symbols=symbols_filtered,
        ranked_symbols=ranked_symbols,
        candidate_opportunities=candidate_opportunities,
        universe_name=effective_config.universe_name,
        data_source_path=("mem",),
        meta={
            "total_symbols_snapshot": len(snapshot.symbols),
            "total_symbols_universe": len(symbols_universe),
            "total_symbols_final": len(symbols_filtered),
            "ranking_applied": True,
            "explanations_applied": True,
            "total_ranked_symbols": len(ranked_symbols),
            "candidate_features_enriched": True,
            "filters_applied": tuple(filters_applied),
        },
    )

