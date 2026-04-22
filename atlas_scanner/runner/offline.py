from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import asdict, dataclass
from datetime import date, datetime

from atlas_scanner.config_loader import ScanConfig, build_default_scan_config_offline
from atlas_scanner.data.openbb_vol_macro import OpenBBVolMacroProvider
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
from atlas_scanner.ports.vol_macro_provider import MacroData, VolData, VolMacroProvider
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


def _provider_enriched_snapshot(
    snapshot: SymbolSnapshot,
    *,
    vol_data: VolData,
    macro_data: MacroData,
) -> SymbolSnapshot:
    merged_meta = dict(snapshot.meta)
    if vol_data.iv_history:
        merged_meta["iv_history"] = list(vol_data.iv_history)
    if vol_data.iv_current is not None:
        merged_meta["iv_current"] = vol_data.iv_current
    if vol_data.rv_annualized:
        merged_meta["rv_annualized"] = dict(vol_data.rv_annualized)

    if macro_data.vix is not None:
        merged_meta["vix"] = macro_data.vix
    if macro_data.macro_regime is not None:
        merged_meta["macro_regime"] = macro_data.macro_regime
    if macro_data.seasonal_factor is not None:
        merged_meta["seasonal_factor"] = macro_data.seasonal_factor

    return SymbolSnapshot(
        symbol=snapshot.symbol,
        asset_type=snapshot.asset_type,
        base_currency=snapshot.base_currency,
        ref_price=snapshot.ref_price,
        volatility_lookback=snapshot.volatility_lookback,
        liquidity_score=snapshot.liquidity_score,
        meta=merged_meta,
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
        meta={
            "component_explanations": {
                name: asdict(explanation)
                for name, explanation in scored.component_explanations.items()
            },
            "top_reasons": tuple(scored.top_reasons),
        },
    )
    return enrich_candidate_with_features(
        candidate,
        vol_input=_vol_input_from_snapshot(snapshot),
        gamma_input=_gamma_input_from_snapshot(snapshot),
    )


def run_offline_scan(
    config: ScanConfig | None = None,
    vol_macro_provider: VolMacroProvider | None = None,
    *,
    min_volume_20d: float | int | None = None,
    max_bid_ask_spread: float | int | None = None,
    min_liquidity_score: float | int | None = None,
    min_ref_price: float | int | None = None,
    max_ref_price: float | int | None = None,
    max_event_risk: float | int | None = None,
) -> OfflineScanResult:
    effective_config = config or build_default_scan_config_offline()
    provider = vol_macro_provider or OpenBBVolMacroProvider()
    snapshot = build_offline_snapshot(config=effective_config)
    as_of_date: date = OFFLINE_REFERENCE_DATETIME.date()
    try:
        macro_data = provider.get_macro_data(as_of_date)
    except Exception:
        macro_data = MacroData()
    provider_enriched_symbols: list[SymbolSnapshot] = []
    for symbol_snapshot in snapshot.symbols:
        try:
            vol_data = provider.get_vol_data(symbol_snapshot.symbol, as_of_date)
        except Exception:
            vol_data = VolData()
        provider_enriched_symbols.append(
            _provider_enriched_snapshot(
                symbol_snapshot,
                vol_data=vol_data,
                macro_data=macro_data,
            )
        )
    snapshot_symbols = tuple(provider_enriched_symbols)
    symbols_universe = select_offline_universe(
        config=effective_config,
        snapshots=snapshot_symbols,
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
            "total_symbols_snapshot": len(snapshot_symbols),
            "total_symbols_universe": len(symbols_universe),
            "total_symbols_final": len(symbols_filtered),
            "ranking_applied": True,
            "explanations_applied": True,
            "total_ranked_symbols": len(ranked_symbols),
            "candidate_features_enriched": True,
            "vol_macro_provider_applied": True,
            "filters_applied": tuple(filters_applied),
        },
    )

