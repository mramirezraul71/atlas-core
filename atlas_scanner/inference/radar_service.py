from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
from typing import Any, Mapping

from atlas_scanner.contracts import (
    InterpretedScenario,
    RadarSignal,
    RadarSignalBatch,
    RadarTimeframe,
)
from atlas_scanner.fusion import build_feature_vector
from atlas_scanner.inference.scoring import aggregate_conviction, build_degradation_reason, build_primary_reason
from atlas_scanner.interpretation import interpret_signal
from atlas_scanner.perception.context import build_operational_context
from atlas_scanner.perception.institutional import build_insider_trading, build_institutional_ownership
from atlas_scanner.perception.macro import build_macro_context
from atlas_scanner.perception.market import (
    MarketPerceptionInput,
    build_flow_perception,
    build_gamma_context,
    build_market_perception,
    to_dealer_positioning_snapshot,
)
from atlas_scanner.perception.political import build_political_trading_context
from atlas_scanner.perception.regulatory import build_regulatory_event_context


DEFAULT_TIMEFRAMES: tuple[RadarTimeframe, ...] = ("1m", "5m", "15m", "1h", "1d")


@dataclass(frozen=True)
class RadarRunInput:
    symbol: str
    market_by_timeframe: Mapping[RadarTimeframe, Mapping[str, Any]]
    flow_by_timeframe: Mapping[RadarTimeframe, Mapping[str, Any]]
    macro_by_timeframe: Mapping[RadarTimeframe, Mapping[str, Any]] | None = None
    institutional_payload: Mapping[str, Any] | None = None
    insider_payload: Mapping[str, Any] | None = None
    political_payload: Mapping[str, Any] | None = None
    regulatory_payload: Mapping[str, Any] | None = None
    runtime_mode: str = "paper"
    market_session: str = "regular"
    provider_status: Mapping[str, str] | None = None
    provider_latency_ms: Mapping[str, int] | None = None
    as_of: datetime | None = None
    timeframes: tuple[RadarTimeframe, ...] = DEFAULT_TIMEFRAMES


def run_radar_first_cut(payload: RadarRunInput) -> RadarSignalBatch:
    as_of = payload.as_of or datetime.now(timezone.utc)
    _ = build_operational_context(
        symbol=payload.symbol,
        runtime_mode=payload.runtime_mode,
        market_session=payload.market_session,
        provider_status=payload.provider_status,
        provider_latency_ms=payload.provider_latency_ms,
        as_of=as_of,
    )
    ownership = build_institutional_ownership(
        symbol=payload.symbol,
        provider_payload=payload.institutional_payload,
        as_of=as_of,
    )
    insider = build_insider_trading(
        symbol=payload.symbol,
        provider_payload=payload.insider_payload,
        as_of=as_of,
    )
    political = build_political_trading_context(
        scope=payload.symbol,
        provider_payload=payload.political_payload,
        as_of=as_of,
    )
    regulatory = build_regulatory_event_context(
        symbol_or_scope=payload.symbol,
        provider_payload=payload.regulatory_payload,
        as_of=as_of,
    )
    signals: list[RadarSignal] = []
    scenarios: dict[RadarTimeframe, tuple[InterpretedScenario, ...]] = {}
    for timeframe in payload.timeframes:
        market_raw = payload.market_by_timeframe.get(timeframe, {})
        flow_raw = payload.flow_by_timeframe.get(timeframe, {})
        macro_raw = (payload.macro_by_timeframe or {}).get(timeframe, {})
        perception_input = MarketPerceptionInput(
            symbol=payload.symbol,
            timeframe=timeframe,
            market_data=market_raw,
            flow_data=flow_raw,
            provider_diagnostics=payload.provider_status or {},
            as_of=as_of,
        )
        market = build_market_perception(perception_input)
        flow = build_flow_perception(perception_input)
        gamma_context = build_gamma_context(
            symbol=payload.symbol,
            as_of=as_of,
            spot_price=market.spot_price,
            options_chain=flow_raw.get("options_chain") if isinstance(flow_raw.get("options_chain"), Mapping) else None,
            flow_snapshot=flow,
        )
        dealer = to_dealer_positioning_snapshot(gamma_context)
        macro = build_macro_context(
            scope=payload.symbol,
            provider_payload=macro_raw,
            as_of=as_of,
        )
        feature_vector = build_feature_vector(
            market,
            flow,
            as_of=as_of,
            dealer=dealer,
            macro=macro,
            ownership=ownership,
            insider=insider,
            political=political,
            regulatory=regulatory,
        )
        aggregate = aggregate_conviction(feature_vector)
        quality = feature_vector.quality
        degradation_reason = quality.degradation_reasons[0] if quality.degradation_reasons else None
        signal = RadarSignal(
            symbol=payload.symbol.upper(),
            timeframe=timeframe,
            as_of=as_of,
            direction_score=feature_vector.direction,
            volume_confirmation_score=feature_vector.volume_confirmation,
            flow_conviction_score=feature_vector.flow_conviction,
            dte_pressure_score=feature_vector.dte_pressure,
            dealer_positioning_proxy_score=feature_vector.dealer_proxy,
            aggregate_conviction_score=aggregate,
            quality=quality,
            primary_conviction_reason=build_primary_reason(feature_vector),
            primary_degradation_reason=degradation_reason,
            meta={
                "feature_reasons": feature_vector.feature_reasons,
                "horizon_scores": dict(feature_vector.horizon_scores),
                "freshness": {
                    horizon: {
                        domain: {
                            "status": item.status,
                            "ttl_sec": item.ttl_sec,
                            "age_sec": item.age_sec,
                            "weight_multiplier": item.weight_multiplier,
                            "reason": item.reason,
                        }
                        for domain, item in domains.items()
                    }
                    for horizon, domains in feature_vector.freshness.items()
                },
                "effective_weights": dict(feature_vector.effective_weights),
                "degraded_domains": feature_vector.degraded_domains,
                "active_domains": feature_vector.active_domains,
                "macro_calendar": {
                    "calendar_risk_score": macro.calendar_risk_score,
                    "calendar_volatility_window": macro.calendar_volatility_window,
                    "upcoming_events_count": len(macro.upcoming_events),
                    "recent_events_count": len(macro.recent_events),
                    "recent_surprise": macro.recent_events[0].surprise if macro.recent_events else None,
                },
                "dealer_context": feature_vector.meta.get("dealer_context", {}),
                "structural_confidence_score": feature_vector.meta.get("structural_confidence_score"),
                "structural_bullish_score": feature_vector.meta.get("structural_bullish_score"),
                "structural_bearish_score": feature_vector.meta.get("structural_bearish_score"),
                "fast_pressure_score": feature_vector.meta.get("fast_pressure_score"),
                "fast_directional_bias": feature_vector.meta.get("fast_directional_bias"),
                "fast_risk_score": feature_vector.meta.get("fast_risk_score"),
                "fast_structural_alignment": feature_vector.meta.get("fast_structural_alignment"),
                "fast_structural_divergence_score": feature_vector.meta.get("fast_structural_divergence_score"),
                "horizon_conflict": feature_vector.meta.get("horizon_conflict"),
                "cross_horizon_alignment": feature_vector.meta.get("cross_horizon_alignment"),
                "institutional_context": {
                    "ownership_delta_pct": ownership.ownership_delta_pct,
                    "concentration_score": ownership.concentration_score,
                    "sponsorship_score": ownership.meta.get("sponsorship_score"),
                    "ownership_signal": ownership.meta.get("ownership_signal"),
                    "holder_change_balance": ownership.meta.get("holder_change_balance"),
                    "delay_days": ownership.meta.get("delay_days"),
                },
                "political_context": {
                    "net_political_flow": political.net_political_flow,
                    "disclosure_lag_days": political.disclosure_lag_days,
                    "signal_strength": political.meta.get("signal_strength"),
                    "transaction_count": political.meta.get("transaction_count"),
                    "buy_sell_balance": political.meta.get("buy_sell_balance"),
                    "notable_entities": political.meta.get("notable_entities"),
                    "related_tickers": political.related_tickers,
                },
                "provider_status": dict(payload.provider_status or {}),
                "provider_latency_ms": dict(payload.provider_latency_ms or {}),
            },
        )
        classification = _classify_snapshot_state(signal)
        signal = RadarSignal(
            symbol=signal.symbol,
            timeframe=signal.timeframe,
            as_of=signal.as_of,
            direction_score=signal.direction_score,
            volume_confirmation_score=signal.volume_confirmation_score,
            flow_conviction_score=signal.flow_conviction_score,
            dte_pressure_score=signal.dte_pressure_score,
            dealer_positioning_proxy_score=signal.dealer_positioning_proxy_score,
            aggregate_conviction_score=signal.aggregate_conviction_score,
            quality=signal.quality,
            primary_conviction_reason=signal.primary_conviction_reason,
            primary_degradation_reason=signal.primary_degradation_reason,
            meta={**dict(signal.meta), "snapshot_classification": classification},
        )
        if signal.primary_degradation_reason is None:
            signal = RadarSignal(
                symbol=signal.symbol,
                timeframe=signal.timeframe,
                as_of=signal.as_of,
                direction_score=signal.direction_score,
                volume_confirmation_score=signal.volume_confirmation_score,
                flow_conviction_score=signal.flow_conviction_score,
                dte_pressure_score=signal.dte_pressure_score,
                dealer_positioning_proxy_score=signal.dealer_positioning_proxy_score,
                aggregate_conviction_score=signal.aggregate_conviction_score,
                quality=signal.quality,
                primary_conviction_reason=signal.primary_conviction_reason,
                primary_degradation_reason=build_degradation_reason(signal),
                meta=signal.meta,
            )
        signals.append(signal)
        scenarios[timeframe] = interpret_signal(signal)
    ordered_signals = tuple(sorted(signals, key=lambda item: item.aggregate_conviction_score, reverse=True))
    primary_signal = ordered_signals[0] if ordered_signals else None
    return RadarSignalBatch(
        symbol=payload.symbol.upper(),
        as_of=as_of,
        signals=ordered_signals,
        primary_signal=primary_signal,
        scenarios_by_timeframe=scenarios,
        meta={"mode": "first_cut_non_invasive"},
    )


def _classify_snapshot_state(signal: RadarSignal) -> str:
    if not isinstance(signal.meta, Mapping):
        return "non_operable"
    freshness = signal.meta.get("freshness", {})
    active = signal.meta.get("active_domains", ())
    degraded = signal.meta.get("degraded_domains", ())
    if not signal.quality.is_operable:
        if "macro" in active or "ownership" in active or "insider" in active or "political" in active:
            return "structural_only"
        if "market" in active or "flow" in active or "dealer" in active:
            return "fast_only"
        return "non_operable"
    if signal.quality.is_operable and not signal.quality.is_degraded:
        if isinstance(freshness, Mapping):
            intraday = freshness.get("intraday", {})
            swing = freshness.get("swing", {})
            if isinstance(intraday, Mapping) and isinstance(swing, Mapping):
                critical_ok = all(
                    str((intraday.get(domain, {}) or {}).get("status", "degraded")) == "usable"
                    for domain in ("market", "flow", "dealer")
                )
                structural_ok = any(
                    str((swing.get(domain, {}) or {}).get("status", "degraded")) == "usable"
                    for domain in ("ownership", "insider", "political", "regulatory")
                )
                if critical_ok and structural_ok:
                    return "fully_operable"
    if degraded:
        return "operable_with_degradation"
    return "operable_with_degradation"
