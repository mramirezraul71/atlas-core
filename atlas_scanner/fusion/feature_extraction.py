from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime
from typing import Mapping

from atlas_scanner.contracts import (
    DealerPositioningSnapshot,
    FlowPerceptionSnapshot,
    InsiderTradingSnapshot,
    InstitutionalOwnershipSnapshot,
    MacroContextSnapshot,
    MarketPerceptionSnapshot,
    PoliticalTradingSnapshot,
    RadarQualityFlags,
    RegulatoryEventSnapshot,
)
from atlas_scanner.fusion.freshness import DomainFreshness, effective_horizon_weights, evaluate_domain_freshness


@dataclass(frozen=True)
class RadarFeatureVector:
    direction: float
    volume_confirmation: float
    flow_conviction: float
    dte_pressure: float
    dealer_proxy: float
    structural_support: float
    macro_alignment: float
    institutional_alignment: float
    quality: RadarQualityFlags
    freshness: Mapping[str, Mapping[str, DomainFreshness]] = field(default_factory=dict)
    effective_weights: Mapping[str, Mapping[str, float]] = field(default_factory=dict)
    horizon_scores: Mapping[str, float] = field(default_factory=dict)
    degraded_domains: tuple[str, ...] = ()
    active_domains: tuple[str, ...] = ()
    feature_reasons: tuple[str, ...] = ()
    meta: Mapping[str, object] = field(default_factory=dict)


def _bounded_score(value: float | None, *, floor: float = 0.0, ceil: float = 100.0) -> float:
    if value is None:
        return 50.0
    if value < floor:
        return floor
    if value > ceil:
        return ceil
    return value


def _build_quality_flags(
    market: MarketPerceptionSnapshot,
    flow: FlowPerceptionSnapshot,
    macro: MacroContextSnapshot | None = None,
) -> RadarQualityFlags:
    has_market_data = market.spot_price is not None and market.return_pct is not None
    has_flow_data = (flow.call_volume is not None and flow.put_volume is not None) or (
        flow.call_premium is not None and flow.put_premium is not None
    )
    has_greeks_context = flow.net_gamma is not None
    has_oi_context = flow.oi_concentration is not None
    provider_quality_ok = not any(value == "error" for value in flow.diagnostics.values())
    degradation_reasons: list[str] = []
    if not has_flow_data:
        degradation_reasons.append("missing_flow_data")
    if not has_market_data:
        degradation_reasons.append("missing_market_data")
    if not has_greeks_context:
        degradation_reasons.append("missing_greeks_context")
    if not has_oi_context:
        degradation_reasons.append("missing_oi_context")
    if not provider_quality_ok:
        degradation_reasons.append("provider_quality_doubtful")
    if macro is not None and macro.calendar_volatility_window:
        degradation_reasons.append("macro_volatility_window")
    is_operable = has_market_data and provider_quality_ok
    is_degraded = len(degradation_reasons) > 0
    return RadarQualityFlags(
        has_market_data=has_market_data,
        has_flow_data=has_flow_data,
        has_greeks_context=has_greeks_context,
        has_oi_context=has_oi_context,
        provider_quality_ok=provider_quality_ok,
        is_degraded=is_degraded,
        is_operable=is_operable,
        degradation_reasons=tuple(degradation_reasons),
    )


def build_feature_vector(
    market: MarketPerceptionSnapshot,
    flow: FlowPerceptionSnapshot,
    *,
    as_of: datetime | None = None,
    dealer: DealerPositioningSnapshot | None = None,
    macro: MacroContextSnapshot | None = None,
    ownership: InstitutionalOwnershipSnapshot | None = None,
    insider: InsiderTradingSnapshot | None = None,
    political: PoliticalTradingSnapshot | None = None,
    regulatory: RegulatoryEventSnapshot | None = None,
) -> RadarFeatureVector:
    current_as_of = as_of or market.as_of
    flow_direction_raw = None
    if flow.call_premium is not None and flow.put_premium is not None and (flow.call_premium + flow.put_premium) > 0:
        flow_direction_raw = ((flow.call_premium - flow.put_premium) / (flow.call_premium + flow.put_premium)) * 50.0 + 50.0
    direction = _bounded_score((market.momentum_score or 50.0) * 0.6 + (flow_direction_raw or 50.0) * 0.4)

    rv = market.relative_volume if market.relative_volume is not None else 1.0
    volume_confirmation = _bounded_score(50.0 + ((rv - 1.0) * 25.0) + (market.volume_acceleration or 0.0) * 10.0)

    flow_conviction = _bounded_score(50.0 + (flow.equity_flow_bias or 0.0) * 25.0)
    dte_pressure = _bounded_score(50.0 + _dte_imbalance(flow) * 50.0)
    dealer_proxy_base = _bounded_score(50.0 + (flow.net_gamma or 0.0) * 15.0 + (flow.oi_concentration or 0.0) * 20.0)
    dealer_proxy = _dealer_score(dealer, dealer_proxy_base)
    macro_alignment = _macro_score(macro)
    direction = _apply_macro_direction_adjustment(direction=direction, macro=macro)
    institutional_alignment = _institutional_score(ownership, insider, political, regulatory)
    structural_support = _bounded_score((macro_alignment * 0.35) + (institutional_alignment * 0.65))
    structural_composites = _structural_composites(ownership=ownership, insider=insider, political=political, regulatory=regulatory)
    fast_composites = _fast_composites(
        direction=direction,
        flow_conviction=flow_conviction,
        dealer_proxy=dealer_proxy,
        macro_alignment=macro_alignment,
        volume_confirmation=volume_confirmation,
    )
    quality = _build_quality_flags(market, flow, macro)
    degraded_domains = _degraded_domains(dealer, macro, ownership, insider, political, regulatory)
    active_domains = _active_domains(dealer, macro, ownership, insider, political, regulatory)
    freshness_by_horizon = _evaluate_freshness(
        as_of=current_as_of,
        market=market,
        flow=flow,
        dealer=dealer,
        macro=macro,
        ownership=ownership,
        insider=insider,
        political=political,
        regulatory=regulatory,
    )
    intraday_weights = effective_horizon_weights(
        horizon="intraday",
        base_weights={"market": 0.2, "flow": 0.2, "dealer": 0.4, "macro": 0.1, "institutional": 0.1},
        domain_freshness=freshness_by_horizon["intraday"],
    )
    swing_weights = effective_horizon_weights(
        horizon="swing",
        base_weights={"market": 0.15, "flow": 0.15, "dealer": 0.15, "macro": 0.2, "institutional": 0.35},
        domain_freshness=freshness_by_horizon["swing"],
    )
    positional_weights = effective_horizon_weights(
        horizon="positional",
        base_weights={"macro": 0.28, "institutional": 0.52, "regulatory": 0.15, "political": 0.05},
        domain_freshness=freshness_by_horizon["positional"],
    )

    intraday_score = _bounded_score(
        (direction * intraday_weights.get("market", 0.0))
        + (flow_conviction * intraday_weights.get("flow", 0.0))
        + (dealer_proxy * intraday_weights.get("dealer", 0.0))
        + (macro_alignment * intraday_weights.get("macro", 0.0))
        + (institutional_alignment * intraday_weights.get("institutional", 0.0))
        + (volume_confirmation * 0.15)
    )
    swing_score = _bounded_score(
        (direction * swing_weights.get("market", 0.0))
        + (flow_conviction * swing_weights.get("flow", 0.0))
        + (dealer_proxy * swing_weights.get("dealer", 0.0))
        + (macro_alignment * swing_weights.get("macro", 0.0))
        + (institutional_alignment * swing_weights.get("institutional", 0.0))
    )
    positional_score = _bounded_score(
        (macro_alignment * positional_weights.get("macro", 0.0))
        + (institutional_alignment * positional_weights.get("institutional", 0.0))
        + ((100.0 - (regulatory.overhang_score or 0.0) * 100.0) * positional_weights.get("regulatory", 0.0) if regulatory else 0.0)
        + ((50.0 + (political.net_political_flow or 0.0) * 40.0) * positional_weights.get("political", 0.0) if political else 0.0)
        + (structural_support * 0.15)
    )
    divergence = _divergence_layer(
        intraday=intraday_score,
        swing=swing_score,
        positional=positional_score,
        fast_pressure=fast_composites["fast_pressure_score"],
        structural_confidence=structural_composites["structural_confidence_score"],
    )
    reasons = (
        f"momentum={market.momentum_score}",
        f"relative_volume={market.relative_volume}",
        f"flow_bias={flow.equity_flow_bias}",
        f"macro_alignment={macro_alignment}",
        f"structural_support={structural_support}",
    )
    return RadarFeatureVector(
        direction=direction,
        volume_confirmation=volume_confirmation,
        flow_conviction=flow_conviction,
        dte_pressure=dte_pressure,
        dealer_proxy=dealer_proxy,
        structural_support=structural_support,
        macro_alignment=macro_alignment,
        institutional_alignment=institutional_alignment,
        quality=quality,
        freshness={
            horizon: dict(values)
            for horizon, values in freshness_by_horizon.items()
        },
        effective_weights={
            "intraday": intraday_weights,
            "swing": swing_weights,
            "positional": positional_weights,
        },
        horizon_scores={
            "intraday": intraday_score,
            "swing": swing_score,
            "positional": positional_score,
        },
        degraded_domains=degraded_domains,
        active_domains=active_domains,
        feature_reasons=reasons,
        meta={
            "post_flow_price_reaction_pct": flow.post_flow_price_reaction_pct or 0.0,
            "macro_calendar_risk_score": macro.calendar_risk_score if macro is not None else 0.0,
            "macro_volatility_window": bool(macro.calendar_volatility_window) if macro is not None else False,
            "dealer_context": _dealer_context(dealer),
            **structural_composites,
            **fast_composites,
            **divergence,
        },
    )


def _dte_imbalance(flow: FlowPerceptionSnapshot) -> float:
    call_total = sum(flow.dte_call_premium.values())
    put_total = sum(flow.dte_put_premium.values())
    total = call_total + put_total
    if total <= 0.0:
        return 0.0
    return (call_total - put_total) / total


def _dealer_score(dealer: DealerPositioningSnapshot | None, fallback: float) -> float:
    if dealer is None:
        return fallback
    pin_penalty = 8.0 if dealer.pinning_zone is not None else 0.0
    accel_boost = 10.0 if dealer.acceleration_zone is not None else 0.0
    pressure = 50.0
    if isinstance(dealer.meta, Mapping):
        pressure_raw = dealer.meta.get("dealer_pressure_score")
        if isinstance(pressure_raw, (int, float)):
            pressure = float(pressure_raw)
    confidence_weight = max(0.2, min(1.0, dealer.confidence))
    raw = (fallback * 0.55) + (pressure * 0.45) + accel_boost - pin_penalty
    return _bounded_score((raw * confidence_weight) + (fallback * (1.0 - confidence_weight)))


def _dealer_context(dealer: DealerPositioningSnapshot | None) -> Mapping[str, object]:
    if dealer is None:
        return {}
    meta = dealer.meta if isinstance(dealer.meta, Mapping) else {}
    return {
        "gamma_flip_level": dealer.gamma_flip_level,
        "call_wall": dealer.call_wall,
        "put_wall": dealer.put_wall,
        "pinning_zone": dealer.pinning_zone,
        "acceleration_zone": dealer.acceleration_zone,
        "acceleration_direction": meta.get("acceleration_direction", "neutral"),
        "acceleration_strength": meta.get("acceleration_strength"),
        "pinning_strength": meta.get("pinning_strength"),
        "gamma_flip_confidence": meta.get("gamma_flip_confidence"),
        "dealer_pressure_score": meta.get("dealer_pressure_score"),
    }


def _structural_composites(
    *,
    ownership: InstitutionalOwnershipSnapshot | None,
    insider: InsiderTradingSnapshot | None,
    political: PoliticalTradingSnapshot | None,
    regulatory: RegulatoryEventSnapshot | None,
) -> dict[str, float]:
    bullish = 0.0
    bearish = 0.0
    confidence = 0.0
    contributors = 0
    if ownership is not None and ownership.quality_flags.get("provider_ready", False):
        signal = ownership.meta.get("ownership_signal") if isinstance(ownership.meta, Mapping) else None
        contributors += 1
        confidence += ownership.confidence * 0.35
        if signal == "bullish":
            bullish += 0.35
        elif signal == "bearish":
            bearish += 0.35
    if insider is not None and insider.buy_sell_ratio is not None:
        contributors += 1
        confidence += insider.confidence * 0.25
        if insider.buy_sell_ratio >= 1.05:
            bullish += 0.25
        if insider.buy_sell_ratio <= 0.95:
            bearish += 0.25
    if political is not None and political.net_political_flow is not None:
        contributors += 1
        confidence += political.confidence * 0.20
        if political.net_political_flow >= 0.1:
            bullish += 0.2
        if political.net_political_flow <= -0.1:
            bearish += 0.2
    if regulatory is not None:
        contributors += 1
        confidence += regulatory.confidence * 0.20
        overhang = regulatory.overhang_score or 0.0
        bearish += overhang * 0.20
        bullish += max(0.0, (1.0 - overhang) * 0.05)
    if contributors <= 0:
        return {
            "structural_confidence_score": 20.0,
            "structural_bullish_score": 50.0,
            "structural_bearish_score": 50.0,
        }
    return {
        "structural_confidence_score": _bounded_score((confidence / contributors) * 100.0),
        "structural_bullish_score": _bounded_score(bullish * 100.0),
        "structural_bearish_score": _bounded_score(bearish * 100.0),
    }


def _fast_composites(
    *,
    direction: float,
    flow_conviction: float,
    dealer_proxy: float,
    macro_alignment: float,
    volume_confirmation: float,
) -> dict[str, float | str]:
    fast_pressure = _bounded_score((direction * 0.3) + (flow_conviction * 0.3) + (dealer_proxy * 0.3) + (macro_alignment * 0.1))
    fast_risk = _bounded_score(100.0 - ((volume_confirmation * 0.45) + (macro_alignment * 0.25) + (dealer_proxy * 0.30)))
    bias = "neutral"
    if fast_pressure >= 57.0:
        bias = "bullish"
    elif fast_pressure <= 43.0:
        bias = "bearish"
    return {
        "fast_pressure_score": fast_pressure,
        "fast_directional_bias": bias,
        "fast_risk_score": fast_risk,
    }


def _divergence_layer(
    *,
    intraday: float,
    swing: float,
    positional: float,
    fast_pressure: float,
    structural_confidence: float,
) -> dict[str, float | str | bool]:
    diff = float(fast_pressure) - float(structural_confidence)
    fast_structural_alignment = "aligned"
    if diff >= 10.0:
        fast_structural_alignment = "fast_dominant"
    elif diff <= -10.0:
        fast_structural_alignment = "structural_dominant"
    intraday_vs_swing = abs(intraday - swing)
    swing_vs_positional = abs(swing - positional)
    horizon_conflict = intraday_vs_swing >= 12.0 or swing_vs_positional >= 12.0
    cross_horizon_alignment = not horizon_conflict and abs(intraday - positional) <= 10.0
    return {
        "fast_structural_alignment": fast_structural_alignment,
        "fast_structural_divergence_score": _bounded_score(abs(diff), floor=0.0, ceil=100.0),
        "horizon_conflict": horizon_conflict,
        "cross_horizon_alignment": cross_horizon_alignment,
    }


def _macro_score(macro: MacroContextSnapshot | None) -> float:
    if macro is None:
        return 45.0
    base = 55.0 if macro.high_impact_flag else 50.0
    if macro.upcoming_event:
        base -= 5.0
    base -= min(15.0, macro.calendar_risk_score * 20.0)
    if macro.calendar_volatility_window:
        base -= 8.0
    if _recent_surprise(macro) is not None:
        base += max(-6.0, min(6.0, _recent_surprise(macro) * 3.0))
    sensitivity = macro.sector_sensitivity or 0.0
    return _bounded_score(base + (sensitivity * 10.0))


def _apply_macro_direction_adjustment(*, direction: float, macro: MacroContextSnapshot | None) -> float:
    if macro is None:
        return direction
    surprise = _recent_surprise(macro)
    if surprise is None:
        return direction
    return _bounded_score(direction + max(-10.0, min(10.0, surprise * 4.0)))


def _recent_surprise(macro: MacroContextSnapshot) -> float | None:
    if not macro.recent_events:
        return None
    latest = macro.recent_events[0]
    return latest.surprise


def _institutional_score(
    ownership: InstitutionalOwnershipSnapshot | None,
    insider: InsiderTradingSnapshot | None,
    political: PoliticalTradingSnapshot | None,
    regulatory: RegulatoryEventSnapshot | None,
) -> float:
    score = 50.0
    if ownership is not None and ownership.ownership_delta_pct is not None:
        score += ownership.ownership_delta_pct * 25.0
    if ownership is not None and ownership.concentration_score is not None:
        score += ownership.concentration_score * 12.0
    if ownership is not None:
        sponsorship = ownership.meta.get("sponsorship_score") if isinstance(ownership.meta, Mapping) else None
        if isinstance(sponsorship, (int, float)):
            score += float(sponsorship) * 10.0
        signal = ownership.meta.get("ownership_signal") if isinstance(ownership.meta, Mapping) else None
        if signal == "bullish":
            score += 4.0
        if signal == "bearish":
            score -= 4.0
    if insider is not None and insider.buy_sell_ratio is not None:
        score += (insider.buy_sell_ratio - 1.0) * 15.0
    if political is not None and political.net_political_flow is not None:
        lag_days = political.disclosure_lag_days or 30
        lag_factor = 0.35 if lag_days > 45 else 0.5 if lag_days > 20 else 0.65
        strength = 0.4
        if isinstance(political.meta, Mapping):
            signal_strength = str(political.meta.get("signal_strength", "weak"))
            if signal_strength == "medium":
                strength = 0.65
            if signal_strength == "strong":
                strength = 0.9
        score += political.net_political_flow * 10.0 * lag_factor * strength
    if regulatory is not None:
        overhang = regulatory.overhang_score or 0.0
        score -= overhang * 20.0
    return _bounded_score(score)


def _degraded_domains(
    dealer: DealerPositioningSnapshot | None,
    macro: MacroContextSnapshot | None,
    ownership: InstitutionalOwnershipSnapshot | None,
    insider: InsiderTradingSnapshot | None,
    political: PoliticalTradingSnapshot | None,
    regulatory: RegulatoryEventSnapshot | None,
) -> tuple[str, ...]:
    degraded: list[str] = []
    for name, snapshot in (
        ("dealer", dealer),
        ("macro", macro),
        ("ownership", ownership),
        ("insider", insider),
        ("political", political),
        ("regulatory", regulatory),
    ):
        if snapshot is None:
            degraded.append(name)
            continue
        quality_flags = getattr(snapshot, "quality_flags", {})
        if isinstance(quality_flags, Mapping) and bool(quality_flags.get("is_stub", False)):
            degraded.append(name)
    return tuple(degraded)


def _active_domains(
    dealer: DealerPositioningSnapshot | None,
    macro: MacroContextSnapshot | None,
    ownership: InstitutionalOwnershipSnapshot | None,
    insider: InsiderTradingSnapshot | None,
    political: PoliticalTradingSnapshot | None,
    regulatory: RegulatoryEventSnapshot | None,
) -> tuple[str, ...]:
    active: list[str] = ["market", "flow"]
    if dealer is not None:
        active.append("dealer")
    if macro is not None:
        active.append("macro")
    if ownership is not None:
        active.append("ownership")
    if insider is not None:
        active.append("insider")
    if political is not None:
        active.append("political")
    if regulatory is not None:
        active.append("regulatory")
    return tuple(active)


def _evaluate_freshness(
    *,
    as_of: datetime,
    market: MarketPerceptionSnapshot,
    flow: FlowPerceptionSnapshot,
    dealer: DealerPositioningSnapshot | None,
    macro: MacroContextSnapshot | None,
    ownership: InstitutionalOwnershipSnapshot | None,
    insider: InsiderTradingSnapshot | None,
    political: PoliticalTradingSnapshot | None,
    regulatory: RegulatoryEventSnapshot | None,
) -> dict[str, dict[str, DomainFreshness]]:
    horizons = ("intraday", "swing", "positional")
    per_horizon: dict[str, dict[str, DomainFreshness]] = {}
    for horizon in horizons:
        per_horizon[horizon] = {
            "market": evaluate_domain_freshness(
                domain="market",
                horizon=horizon,
                as_of=as_of,
                snapshot_as_of=market.as_of,
                delay_sec=None,
                confidence=1.0,
                quality_flags={"provider_ready": True, "is_stub": False},
            ),
            "flow": evaluate_domain_freshness(
                domain="flow",
                horizon=horizon,
                as_of=as_of,
                snapshot_as_of=flow.as_of,
                delay_sec=None,
                confidence=1.0,
                quality_flags=flow.quality_flags,
            ),
            "dealer": _domain_freshness_from_snapshot(domain="dealer", horizon=horizon, as_of=as_of, snapshot=dealer),
            "macro": _domain_freshness_from_snapshot(domain="macro", horizon=horizon, as_of=as_of, snapshot=macro),
            "ownership": _domain_freshness_from_snapshot(domain="ownership", horizon=horizon, as_of=as_of, snapshot=ownership),
            "insider": _domain_freshness_from_snapshot(domain="insider", horizon=horizon, as_of=as_of, snapshot=insider),
            "political": _domain_freshness_from_snapshot(domain="political", horizon=horizon, as_of=as_of, snapshot=political),
            "regulatory": _domain_freshness_from_snapshot(domain="regulatory", horizon=horizon, as_of=as_of, snapshot=regulatory),
            "institutional": _domain_freshness_from_snapshot(domain="ownership", horizon=horizon, as_of=as_of, snapshot=ownership),
        }
    return per_horizon


def _domain_freshness_from_snapshot(
    *,
    domain: str,
    horizon: str,
    as_of: datetime,
    snapshot: object | None,
) -> DomainFreshness:
    if snapshot is None:
        return evaluate_domain_freshness(
            domain=domain,
            horizon=horizon,
            as_of=as_of,
            snapshot_as_of=None,
            delay_sec=None,
            confidence=0.0,
            quality_flags={"provider_ready": False},
        )
    snapshot_as_of = getattr(snapshot, "as_of", None)
    delay_sec = getattr(snapshot, "delay_sec", None)
    confidence = getattr(snapshot, "confidence", 0.0)
    quality_flags = getattr(snapshot, "quality_flags", {})
    return evaluate_domain_freshness(
        domain=domain,
        horizon=horizon,
        as_of=as_of,
        snapshot_as_of=snapshot_as_of if isinstance(snapshot_as_of, datetime) else None,
        delay_sec=delay_sec if isinstance(delay_sec, int) else None,
        confidence=confidence if isinstance(confidence, (int, float)) else 0.0,
        quality_flags=quality_flags if isinstance(quality_flags, Mapping) else {},
    )
