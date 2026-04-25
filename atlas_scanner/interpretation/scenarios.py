from __future__ import annotations

from atlas_scanner.contracts import InterpretedScenario, RadarSignal

_SCENARIO_CATALOG: tuple[str, ...] = (
    "speculative_upside_momentum",
    "speculative_downside_momentum",
    "protective_downside_hedge",
    "upside_call_chasing",
    "bearish_put_chasing",
    "covered_call_supply",
    "short_put_risk_acceptance",
    "bullish_flow_absorbed",
    "bearish_flow_absorbed",
    "dealer_gamma_pin_risk",
    "gamma_acceleration_zone",
    "dark_pool_accumulation",
    "dark_pool_distribution",
    "mixed_horizon_divergence",
    "0dte_noise_high_risk",
    "high_flow_low_confirmation",
    "high_confirmation_breakout_setup",
    "hedge_unwind_reversal_candidate",
    "macro_event_risk",
    "macro_tailwind",
    "institutional_accumulation",
    "institutional_distribution",
    "insider_bullish_alignment",
    "insider_bearish_alignment",
    "political_interest_context",
    "regulatory_overhang",
    "dealer_gamma_acceleration",
    "dealer_gamma_absorption",
    "cross_domain_bullish_alignment",
    "cross_domain_bearish_alignment",
    "fast_signal_but_weak_structural_support",
    "weak_intraday_but_strong_structural_tailwind",
    "macro_event_imminent",
    "macro_volatility_window_active",
    "macro_surprise_bullish",
    "macro_surprise_bearish",
    "macro_calendar_clear",
    "institutional_alignment",
    "institutional_sponsorship_strong",
    "institutional_sponsorship_weak",
    "dealer_gamma_flip_approaching",
    "dealer_call_wall_active",
    "dealer_put_wall_active",
    "dealer_pinning_zone_active",
    "dealer_acceleration_zone_bullish",
    "dealer_acceleration_zone_bearish",
    "dealer_pressure_neutral",
    "political_bullish_alignment",
    "political_bearish_alignment",
    "political_signal_weak",
    "political_signal_stale",
)


def interpret_signal(signal: RadarSignal) -> tuple[InterpretedScenario, ...]:
    scenarios: list[InterpretedScenario] = []
    bullish = signal.direction_score >= 55.0
    bearish = signal.direction_score <= 45.0
    high_flow_low_confirmation = signal.flow_conviction_score >= 65.0 and signal.volume_confirmation_score < 50.0
    high_confirmation_breakout = signal.flow_conviction_score >= 60.0 and signal.volume_confirmation_score >= 60.0
    horizon_scores = signal.meta.get("horizon_scores") if isinstance(signal.meta, dict) else {}
    intraday = float(horizon_scores.get("intraday", signal.aggregate_conviction_score)) if isinstance(horizon_scores, dict) else signal.aggregate_conviction_score
    positional = float(horizon_scores.get("positional", signal.aggregate_conviction_score)) if isinstance(horizon_scores, dict) else signal.aggregate_conviction_score
    macro_calendar = signal.meta.get("macro_calendar") if isinstance(signal.meta, dict) else {}
    macro_risk = float(macro_calendar.get("calendar_risk_score", 0.0)) if isinstance(macro_calendar, dict) else 0.0
    macro_window = bool(macro_calendar.get("calendar_volatility_window", False)) if isinstance(macro_calendar, dict) else False
    recent_surprise = macro_calendar.get("recent_surprise") if isinstance(macro_calendar, dict) else None
    institutional = signal.meta.get("institutional_context") if isinstance(signal.meta, dict) else {}
    dealer = signal.meta.get("dealer_context") if isinstance(signal.meta, dict) else {}
    political = signal.meta.get("political_context") if isinstance(signal.meta, dict) else {}
    freshness = signal.meta.get("freshness") if isinstance(signal.meta, dict) else {}
    ownership_signal = institutional.get("ownership_signal") if isinstance(institutional, dict) else None
    sponsorship_score = institutional.get("sponsorship_score") if isinstance(institutional, dict) else None
    dealer_pressure = float(dealer.get("dealer_pressure_score", 50.0)) if isinstance(dealer, dict) else 50.0
    acceleration_direction = str(dealer.get("acceleration_direction", "neutral")) if isinstance(dealer, dict) else "neutral"
    pinning_strength = dealer.get("pinning_strength") if isinstance(dealer, dict) else None
    gamma_flip_confidence = dealer.get("gamma_flip_confidence") if isinstance(dealer, dict) else None
    fast_structural_alignment = signal.meta.get("fast_structural_alignment") if isinstance(signal.meta, dict) else None
    horizon_conflict = bool(signal.meta.get("horizon_conflict", False)) if isinstance(signal.meta, dict) else False
    political_flow = float(political.get("net_political_flow", 0.0)) if isinstance(political, dict) and isinstance(political.get("net_political_flow"), (int, float)) else 0.0
    political_strength = str(political.get("signal_strength", "weak")) if isinstance(political, dict) else "weak"
    political_domain = freshness.get("swing", {}).get("political", {}) if isinstance(freshness, dict) else {}
    political_status = political_domain.get("status") if isinstance(political_domain, dict) else None

    if bullish:
        scenarios.append(_scenario(signal, "speculative_upside_momentum", "Flow y dirección empujan al alza."))
    if bearish:
        scenarios.append(_scenario(signal, "speculative_downside_momentum", "Flow y dirección empujan a la baja."))
    if signal.flow_conviction_score >= 60.0 and signal.dte_pressure_score >= 55.0:
        scenarios.append(_scenario(signal, "upside_call_chasing", "Prima call dominante por buckets DTE."))
    if signal.flow_conviction_score <= 40.0 and signal.dte_pressure_score <= 45.0:
        scenarios.append(_scenario(signal, "bearish_put_chasing", "Prima put dominante por buckets DTE."))
    if signal.dealer_positioning_proxy_score <= 40.0:
        scenarios.append(_scenario(signal, "dealer_gamma_pin_risk", "Dealer proxy sugiere pinning o absorción."))
        scenarios.append(_scenario(signal, "dealer_gamma_absorption", "Estructura dealer absorbiendo dirección intradía."))
    if signal.dealer_positioning_proxy_score >= 65.0:
        scenarios.append(_scenario(signal, "gamma_acceleration_zone", "Dealer proxy favorece aceleración."))
        scenarios.append(_scenario(signal, "dealer_gamma_acceleration", "Posicionamiento dealer habilita aceleración."))
    if isinstance(gamma_flip_confidence, (int, float)) and gamma_flip_confidence >= 0.6:
        scenarios.append(_scenario(signal, "dealer_gamma_flip_approaching", "Gamma flip cercano con confianza operativa."))
    if isinstance(dealer, dict) and dealer.get("call_wall") is not None:
        scenarios.append(_scenario(signal, "dealer_call_wall_active", "Call wall activo limitando/activando ruptura superior."))
    if isinstance(dealer, dict) and dealer.get("put_wall") is not None:
        scenarios.append(_scenario(signal, "dealer_put_wall_active", "Put wall activo como soporte/disparo de downside."))
    if isinstance(pinning_strength, (int, float)) and pinning_strength >= 0.55:
        scenarios.append(_scenario(signal, "dealer_pinning_zone_active", "Zona de pinning activa en vecindad de vencimiento."))
    if acceleration_direction == "bullish" and dealer_pressure >= 55.0:
        scenarios.append(_scenario(signal, "dealer_acceleration_zone_bullish", "Hedging dealer favorece aceleración al alza."))
    if acceleration_direction == "bearish" and dealer_pressure <= 45.0:
        scenarios.append(_scenario(signal, "dealer_acceleration_zone_bearish", "Hedging dealer favorece aceleración a la baja."))
    if 47.0 <= dealer_pressure <= 53.0:
        scenarios.append(_scenario(signal, "dealer_pressure_neutral", "Presión dealer neta neutral sin sesgo direccional fuerte."))
    if high_flow_low_confirmation:
        scenarios.append(_scenario(signal, "high_flow_low_confirmation", "Flow alto sin confirmación de volumen/precio."))
    if high_confirmation_breakout:
        scenarios.append(
            _scenario(signal, "high_confirmation_breakout_setup", "Flow y volumen confirman potencial breakout.")
        )
    if signal.timeframe == "1m" and signal.quality.is_degraded:
        scenarios.append(_scenario(signal, "0dte_noise_high_risk", "Temporalidad ultra corta con calidad degradada."))
    if "macro" not in _active_domains(signal):
        scenarios.append(_scenario(signal, "macro_event_risk", "Contexto macro degradado o no disponible."))
    else:
        scenarios.append(_scenario(signal, "macro_tailwind", "Contexto macro disponible y utilizable."))
    if macro_window:
        scenarios.append(_scenario(signal, "macro_volatility_window_active", "Ventana de volatilidad macro activa."))
    if macro_risk >= 0.65:
        scenarios.append(_scenario(signal, "macro_event_imminent", "Evento macro high-impact inminente o concentrado."))
    if isinstance(recent_surprise, (int, float)) and recent_surprise >= 0.2:
        scenarios.append(_scenario(signal, "macro_surprise_bullish", "Sorpresa macro reciente con sesgo positivo."))
    if isinstance(recent_surprise, (int, float)) and recent_surprise <= -0.2:
        scenarios.append(_scenario(signal, "macro_surprise_bearish", "Sorpresa macro reciente con sesgo negativo."))
    if not macro_window and macro_risk <= 0.25:
        scenarios.append(_scenario(signal, "macro_calendar_clear", "Calendario macro limpio, baja incertidumbre de eventos."))
    if "ownership" in _active_domains(signal):
        scenarios.append(_scenario(signal, "institutional_accumulation", "Ownership institucional respalda continuidad."))
        if ownership_signal == "bearish":
            scenarios.append(_scenario(signal, "institutional_distribution", "Flujo de holdings institucional sugiere distribución."))
        if ownership_signal in {"bullish", "bearish"}:
            scenarios.append(_scenario(signal, "institutional_alignment", "Señal institucional alineada con contexto estructural."))
        if isinstance(sponsorship_score, (int, float)) and sponsorship_score >= 0.7:
            scenarios.append(
                _scenario(signal, "institutional_sponsorship_strong", "Patrocinio institucional fuerte para horizonte posicional.")
            )
        if isinstance(sponsorship_score, (int, float)) and sponsorship_score <= 0.35:
            scenarios.append(
                _scenario(signal, "institutional_sponsorship_weak", "Patrocinio institucional débil, soporte estructural limitado.")
            )
    if "insider" in _active_domains(signal):
        if bullish:
            scenarios.append(_scenario(signal, "insider_bullish_alignment", "Insiders y flujo alineados al alza."))
        if bearish:
            scenarios.append(_scenario(signal, "insider_bearish_alignment", "Insiders y flujo alineados a la baja."))
    if "political" in _active_domains(signal):
        scenarios.append(_scenario(signal, "political_interest_context", "Disclosure político aporta contexto posicional."))
        if political_flow >= 0.18:
            scenarios.append(_scenario(signal, "political_bullish_alignment", "Disclosure político netamente comprador alineado al sesgo bullish."))
        if political_flow <= -0.18:
            scenarios.append(_scenario(signal, "political_bearish_alignment", "Disclosure político netamente vendedor alineado al sesgo bearish."))
        if political_strength == "weak" or abs(political_flow) < 0.08:
            scenarios.append(_scenario(signal, "political_signal_weak", "Señal política débil o balanceada; usar como contexto estructural."))
        if political_status in {"stale", "degraded"}:
            scenarios.append(_scenario(signal, "political_signal_stale", "Señal política con frescura insuficiente para convicción adicional."))
    if "regulatory" in _active_domains(signal):
        scenarios.append(_scenario(signal, "regulatory_overhang", "Riesgo regulatorio potencial en pricing."))
    if bullish and positional >= 55.0:
        scenarios.append(_scenario(signal, "cross_domain_bullish_alignment", "Alineación bullish entre dominios rápidos y estructurales."))
    if bearish and positional <= 45.0:
        scenarios.append(_scenario(signal, "cross_domain_bearish_alignment", "Alineación bearish entre dominios rápidos y estructurales."))
    if intraday >= 60.0 and positional < 50.0:
        scenarios.append(_scenario(signal, "fast_signal_but_weak_structural_support", "Impulso rápido sin soporte estructural robusto."))
    if intraday < 50.0 and positional >= 60.0:
        scenarios.append(_scenario(signal, "weak_intraday_but_strong_structural_tailwind", "Debilidad intradía con cola estructural favorable."))
    if horizon_conflict:
        scenarios.append(_scenario(signal, "mixed_horizon_divergence", "Conflicto explícito entre horizontes fast/swing/posicional."))
    if fast_structural_alignment == "aligned" and bullish:
        scenarios.append(_scenario(signal, "cross_domain_bullish_alignment", "Alineación explícita fast vs structural con sesgo bullish."))
    if fast_structural_alignment == "aligned" and bearish:
        scenarios.append(_scenario(signal, "cross_domain_bearish_alignment", "Alineación explícita fast vs structural con sesgo bearish."))

    if not scenarios:
        scenarios.append(_scenario(signal, "mixed_horizon_divergence", "Señales mixtas sin dominancia estructural."))
    return tuple(scenarios)


def _active_domains(signal: RadarSignal) -> tuple[str, ...]:
    if not isinstance(signal.meta, dict):
        return ()
    raw = signal.meta.get("active_domains")
    if isinstance(raw, (list, tuple)):
        return tuple(str(item) for item in raw)
    return ()


def _scenario(signal: RadarSignal, name: str, explanation: str) -> InterpretedScenario:
    if name not in _SCENARIO_CATALOG:
        raise ValueError(f"unknown scenario '{name}'")
    conviction = signal.aggregate_conviction_score / 100.0
    ambiguity = ("quality_degraded",) if signal.quality.is_degraded else ()
    invalidation = (
        "price_reversal_vs_direction",
        "flow_reversal_next_window",
    )
    risks = (
        "flow_spoofing_or_rollover",
        "macro_event_noise",
    )
    return InterpretedScenario(
        name=name,
        conviction=conviction,
        probability=conviction,
        explanation=explanation,
        misinterpretation_risks=risks,
        invalidation_conditions=invalidation,
        ambiguity_flags=ambiguity,
        meta={"timeframe": signal.timeframe},
    )
