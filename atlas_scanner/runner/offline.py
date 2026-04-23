from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import asdict, dataclass
from datetime import date, datetime

from atlas_scanner.config_loader import ScanConfig, build_default_scan_config_offline
from atlas_scanner.features.gamma import StrikeGamma
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
from atlas_scanner.ports.gamma_oi_provider import GammaData, GammaOIProvider, OIFlowData
from atlas_scanner.ports.vol_macro_provider import MacroData, VolData, VolMacroProvider
from atlas_scanner.runner.provider_resolution import (
    is_empty_gamma_data,
    is_empty_macro_data,
    is_empty_oi_flow_data,
    is_empty_vol_data,
    resolve_offline_providers,
)
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
    gamma_data: GammaData,
    oi_flow_data: OIFlowData,
    provider_status: dict[str, dict[str, str]],
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

    if gamma_data.strikes:
        merged_meta["gamma_strikes"] = [
            {
                "strike": row.strike,
                "call_gamma": row.call_gamma,
                "put_gamma": row.put_gamma,
            }
            for row in gamma_data.strikes
        ]
        merged_meta["strike_gamma"] = tuple(
            StrikeGamma(
                strike=row.strike,
                call_gamma=row.call_gamma,
                put_gamma=row.put_gamma,
            )
            for row in gamma_data.strikes
        )
    if gamma_data.net_gex is not None:
        merged_meta["net_gex"] = gamma_data.net_gex
        merged_meta["net_gamma"] = gamma_data.net_gex

    if oi_flow_data.oi_change_1d_pct is not None:
        merged_meta["oi_change_1d_pct"] = oi_flow_data.oi_change_1d_pct
    if oi_flow_data.call_put_volume_ratio is not None:
        merged_meta["call_put_volume_ratio"] = oi_flow_data.call_put_volume_ratio
    if oi_flow_data.volume_imbalance is not None:
        merged_meta["volume_imbalance"] = oi_flow_data.volume_imbalance
    if oi_flow_data.call_volume is not None:
        merged_meta["call_volume"] = oi_flow_data.call_volume
    if oi_flow_data.put_volume is not None:
        merged_meta["put_volume"] = oi_flow_data.put_volume
    if oi_flow_data.meta:
        merged_meta["oi_flow_meta"] = dict(oi_flow_data.meta)
    merged_meta["provider_status"] = provider_status

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
    gamma_oi_provider: GammaOIProvider | None = None,
    *,
    min_volume_20d: float | int | None = None,
    max_bid_ask_spread: float | int | None = None,
    min_liquidity_score: float | int | None = None,
    min_ref_price: float | int | None = None,
    max_ref_price: float | int | None = None,
    max_event_risk: float | int | None = None,
    as_of_date: date | None = None,
    symbol_allowlist: Sequence[str] | None = None,
) -> OfflineScanResult:
    effective_config = config or build_default_scan_config_offline()
    effective_as_of_date = as_of_date or OFFLINE_REFERENCE_DATETIME.date()
    reference_datetime = datetime.combine(
        effective_as_of_date,
        OFFLINE_REFERENCE_DATETIME.timetz(),
    )
    providers = resolve_offline_providers(
        vol_macro_provider=vol_macro_provider,
        gamma_oi_provider=gamma_oi_provider,
    )
    vol_provider = providers.vol_macro_provider
    gamma_provider = providers.gamma_oi_provider
    snapshot = build_offline_snapshot(config=effective_config)
    allowed_symbols: set[str] | None = None
    if symbol_allowlist is not None:
        allowed_symbols = {symbol.strip().upper() for symbol in symbol_allowlist if symbol.strip()}
    base_snapshot_symbols = snapshot.symbols
    if allowed_symbols is not None:
        base_snapshot_symbols = tuple(
            symbol_snapshot
            for symbol_snapshot in snapshot.symbols
            if symbol_snapshot.symbol.upper() in allowed_symbols
        )
    vol_macro_call_counts = {
        "vol_data": {"ok": 0, "empty": 0, "error": 0},
        "macro_data": {"ok": 0, "empty": 0, "error": 0},
    }
    gamma_oi_call_counts = {
        "gamma_data": {"ok": 0, "empty": 0, "error": 0},
        "oi_flow_data": {"ok": 0, "empty": 0, "error": 0},
    }
    try:
        macro_data = vol_provider.get_macro_data(effective_as_of_date)
        if is_empty_macro_data(macro_data):
            macro_status = "empty"
        else:
            macro_status = "ok"
    except Exception:
        macro_data = MacroData()
        macro_status = "error"
    vol_macro_call_counts["macro_data"][macro_status] += 1
    provider_enriched_symbols: list[SymbolSnapshot] = []
    for symbol_snapshot in base_snapshot_symbols:
        try:
            vol_data = vol_provider.get_vol_data(symbol_snapshot.symbol, effective_as_of_date)
            if is_empty_vol_data(vol_data):
                vol_status = "empty"
            else:
                vol_status = "ok"
        except Exception:
            vol_data = VolData()
            vol_status = "error"
        vol_macro_call_counts["vol_data"][vol_status] += 1
        try:
            gamma_data = gamma_provider.get_gamma_data(symbol_snapshot.symbol, effective_as_of_date)
            if is_empty_gamma_data(gamma_data):
                gamma_status = "empty"
            else:
                gamma_status = "ok"
        except Exception:
            gamma_data = GammaData()
            gamma_status = "error"
        gamma_oi_call_counts["gamma_data"][gamma_status] += 1
        try:
            oi_flow_data = gamma_provider.get_oi_flow_data(symbol_snapshot.symbol, effective_as_of_date)
            if is_empty_oi_flow_data(oi_flow_data):
                oi_flow_status = "empty"
            else:
                oi_flow_status = "ok"
        except Exception:
            oi_flow_data = OIFlowData()
            oi_flow_status = "error"
        gamma_oi_call_counts["oi_flow_data"][oi_flow_status] += 1
        provider_enriched_symbols.append(
            _provider_enriched_snapshot(
                symbol_snapshot,
                vol_data=vol_data,
                macro_data=macro_data,
                gamma_data=gamma_data,
                oi_flow_data=oi_flow_data,
                provider_status={
                    "vol_macro": {
                        "vol_data": vol_status,
                        "macro_data": macro_status,
                    },
                    "gamma_oi": {
                        "gamma_data": gamma_status,
                        "oi_flow_data": oi_flow_status,
                    },
                },
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
        _candidate_from_scored(scored=item, as_of=reference_datetime)
        for item in ranked_symbols
    )

    vol_macro_status = (
        "error"
        if vol_macro_call_counts["vol_data"]["error"] > 0 or vol_macro_call_counts["macro_data"]["error"] > 0
        else "ok"
        if vol_macro_call_counts["vol_data"]["ok"] > 0 or vol_macro_call_counts["macro_data"]["ok"] > 0
        else "empty"
    )
    gamma_oi_status = (
        "error"
        if gamma_oi_call_counts["gamma_data"]["error"] > 0 or gamma_oi_call_counts["oi_flow_data"]["error"] > 0
        else "ok"
        if gamma_oi_call_counts["gamma_data"]["ok"] > 0 or gamma_oi_call_counts["oi_flow_data"]["ok"] > 0
        else "empty"
    )

    return OfflineScanResult(
        config=effective_config,
        reference_datetime=reference_datetime,
        selected_symbols=symbols_filtered,
        ranked_symbols=ranked_symbols,
        candidate_opportunities=candidate_opportunities,
        universe_name=effective_config.universe_name,
        data_source_path=("mem",),
        meta={
            "as_of_date": effective_as_of_date.isoformat(),
            "total_symbols_snapshot": len(base_snapshot_symbols),
            "total_symbols_universe": len(symbols_universe),
            "total_symbols_final": len(symbols_filtered),
            "ranking_applied": True,
            "explanations_applied": True,
            "total_ranked_symbols": len(ranked_symbols),
            "candidate_features_enriched": True,
            "vol_macro_provider_applied": True,
            "gamma_oi_provider_applied": True,
            "providers": {
                "vol_macro": {
                    "name": type(vol_provider).__name__,
                    "status": vol_macro_status,
                    "calls": vol_macro_call_counts,
                },
                "gamma_oi": {
                    "name": type(gamma_provider).__name__,
                    "status": gamma_oi_status,
                    "calls": gamma_oi_call_counts,
                },
            },
            "filters_applied": tuple(filters_applied),
            "symbol_allowlist_applied": allowed_symbols is not None,
        },
    )

