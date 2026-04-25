from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime
from typing import Any, Mapping

from atlas_scanner.perception.common.circuit_breaker import resolve_provider_circuit_breaker
from atlas_scanner.contracts import DealerPositioningSnapshot
from atlas_scanner.contracts import FlowPerceptionSnapshot


@dataclass(frozen=True)
class GammaContextSnapshot:
    symbol: str
    as_of: datetime
    gamma_by_strike: Mapping[float, float] = field(default_factory=dict)
    gamma_flip_level: float | None = None
    call_wall: float | None = None
    put_wall: float | None = None
    pinning_zone: tuple[float, float] | None = None
    confidence: float = 0.0
    degraded: bool = True
    degradation_reasons: tuple[str, ...] = ()
    diagnostics: Mapping[str, str] = field(default_factory=dict)
    meta: Mapping[str, Any] = field(default_factory=dict)


def build_gamma_context(
    *,
    symbol: str,
    as_of: datetime,
    spot_price: float | None,
    options_chain: Mapping[str, Any] | None = None,
    flow_snapshot: FlowPerceptionSnapshot | None = None,
) -> GammaContextSnapshot:
    chain = options_chain or {}
    strike_gamma_raw = chain.get("gamma_by_strike")
    oi_by_strike_raw = chain.get("oi_by_strike")
    call_oi_raw = chain.get("call_oi_by_strike")
    put_oi_raw = chain.get("put_oi_by_strike")
    dte_weights_raw = chain.get("dte_weights")
    gamma_by_strike = (
        {float(k): float(v) for k, v in strike_gamma_raw.items()}
        if isinstance(strike_gamma_raw, Mapping)
        else {}
    )
    oi_by_strike = (
        {float(k): float(v) for k, v in oi_by_strike_raw.items()}
        if isinstance(oi_by_strike_raw, Mapping)
        else {}
    )
    call_oi_by_strike = (
        {float(k): float(v) for k, v in call_oi_raw.items()}
        if isinstance(call_oi_raw, Mapping)
        else {}
    )
    put_oi_by_strike = (
        {float(k): float(v) for k, v in put_oi_raw.items()}
        if isinstance(put_oi_raw, Mapping)
        else {}
    )
    dte_weights = (
        {str(k): float(v) for k, v in dte_weights_raw.items()}
        if isinstance(dte_weights_raw, Mapping)
        else {}
    )
    if gamma_by_strike and spot_price is not None:
        weighted_exposure = _weighted_gamma_exposure(
            gamma_by_strike=gamma_by_strike,
            oi_by_strike=oi_by_strike,
            call_oi_by_strike=call_oi_by_strike,
            put_oi_by_strike=put_oi_by_strike,
            dte_weights=dte_weights,
            spot_price=spot_price,
        )
        flip = _gamma_flip_level(weighted_exposure)
        call_wall, put_wall, wall_strength = _walls_from_oi(
            oi_by_strike=oi_by_strike,
            call_oi_by_strike=call_oi_by_strike,
            put_oi_by_strike=put_oi_by_strike,
            spot_price=spot_price,
        )
        pinning = _pinning_zone(spot_price=spot_price, call_wall=call_wall, put_wall=put_wall)
        pin_strength = _pinning_strength(
            spot_price=spot_price,
            pinning_zone=pinning,
            wall_strength=wall_strength,
        )
        acceleration, accel_direction, accel_strength = _acceleration_zone(
            spot_price=spot_price,
            gamma_flip_level=flip,
            call_wall=call_wall,
            put_wall=put_wall,
            wall_strength=wall_strength,
        )
        pressure_score = _dealer_pressure_score(
            spot_price=spot_price,
            gamma_flip=flip,
            wall_strength=wall_strength,
            pinning_strength=pin_strength,
            acceleration_strength=accel_strength,
            acceleration_direction=accel_direction,
        )
        flip_confidence = _flip_confidence(
            weighted_exposure=weighted_exposure,
            oi_by_strike=oi_by_strike,
            spot_price=spot_price,
            gamma_flip=flip,
        )
        confidence = max(0.2, min(1.0, (flip_confidence * 0.45) + (wall_strength * 0.25) + (pin_strength * 0.15) + (accel_strength * 0.15)))
        return GammaContextSnapshot(
            symbol=symbol.upper(),
            as_of=as_of,
            gamma_by_strike=weighted_exposure,
            gamma_flip_level=flip,
            call_wall=call_wall,
            put_wall=put_wall,
            pinning_zone=pinning,
            confidence=confidence,
            degraded=False,
            diagnostics={"source": "options_chain", "gamma_flip_confidence": f"{flip_confidence:.3f}"},
            meta={
                "provider_payload_complete": bool(oi_by_strike),
                "call_wall_strength": wall_strength,
                "put_wall_strength": wall_strength,
                "pinning_strength": pin_strength,
                "acceleration_strength": accel_strength,
                "acceleration_direction": accel_direction,
                "acceleration_zone": acceleration,
                "dealer_pressure_score": pressure_score,
                "gamma_flip_confidence": flip_confidence,
            },
        )

    # Degraded proxy path: leverage flow + IV context if available
    breaker = resolve_provider_circuit_breaker("dealer:gamma_proxy")
    proxy_flip = None
    confidence = 0.25
    reasons = ["missing_full_options_chain"]
    if flow_snapshot is not None:
        bias = flow_snapshot.net_bias or 0.0
        if spot_price is not None:
            proxy_flip = spot_price * (1.0 - (bias * 0.002))
        confidence = 0.45 if flow_snapshot.net_gamma is not None else 0.30
        reasons.append("proxy_from_flow_iv")
        breaker.record_success()
    else:
        breaker.record_failure("missing_flow_proxy")
    return GammaContextSnapshot(
        symbol=symbol.upper(),
        as_of=as_of,
        gamma_flip_level=proxy_flip,
        confidence=confidence,
        degraded=True,
        degradation_reasons=tuple(reasons),
        diagnostics={"source": "proxy", "circuit_state": breaker.snapshot().state},
        meta={
            "provider_payload_complete": False,
            "dealer_pressure_score": 50.0,
            "acceleration_direction": "neutral",
            "gamma_flip_confidence": confidence,
        },
    )


def to_dealer_positioning_snapshot(context: GammaContextSnapshot) -> DealerPositioningSnapshot:
    acceleration_zone = context.meta.get("acceleration_zone")
    if not isinstance(acceleration_zone, tuple):
        acceleration_zone = None
        if context.gamma_flip_level is not None and context.call_wall is not None:
            lower = min(context.gamma_flip_level, context.call_wall)
            upper = max(context.gamma_flip_level, context.call_wall)
            acceleration_zone = (lower, upper)
    quality_flags = {
        "degraded": context.degraded,
        "degradation_reasons": list(context.degradation_reasons),
        "gamma_flip_confidence": context.meta.get("gamma_flip_confidence"),
        "pinning_strength": context.meta.get("pinning_strength"),
        "acceleration_strength": context.meta.get("acceleration_strength"),
        "dealer_pressure_score": context.meta.get("dealer_pressure_score"),
    }
    return DealerPositioningSnapshot(
        symbol=context.symbol,
        as_of=context.as_of,
        source=str(context.diagnostics.get("source", "gamma_context")),
        gamma_flip_level=context.gamma_flip_level,
        call_wall=context.call_wall,
        put_wall=context.put_wall,
        pinning_zone=context.pinning_zone,
        acceleration_zone=acceleration_zone,
        freshness_sec=45 if not context.degraded else 180,
        delay_sec=0 if not context.degraded else 90,
        confidence=context.confidence,
        quality_flags=quality_flags,
        meta=dict(context.meta),
    )


def _gamma_flip_level(gamma_by_strike: Mapping[float, float]) -> float | None:
    ordered = sorted(gamma_by_strike.items(), key=lambda item: item[0])
    for idx in range(len(ordered) - 1):
        strike_a, gamma_a = ordered[idx]
        strike_b, gamma_b = ordered[idx + 1]
        if gamma_a == 0.0:
            return strike_a
        if gamma_b == 0.0:
            return strike_b
        if gamma_a * gamma_b < 0:
            slope = (gamma_b - gamma_a) / (strike_b - strike_a)
            if slope == 0:
                return strike_b
            return strike_a - (gamma_a / slope)
    return None


def _walls_from_oi(
    *,
    oi_by_strike: Mapping[float, float],
    call_oi_by_strike: Mapping[float, float],
    put_oi_by_strike: Mapping[float, float],
    spot_price: float,
) -> tuple[float | None, float | None, float]:
    if not oi_by_strike and not call_oi_by_strike and not put_oi_by_strike:
        return None, None, 0.0
    call_map = call_oi_by_strike if call_oi_by_strike else oi_by_strike
    put_map = put_oi_by_strike if put_oi_by_strike else oi_by_strike
    call_candidates = sorted(call_map.items(), key=lambda item: item[1] * _distance_weight(strike=item[0], spot=spot_price), reverse=True)
    put_candidates = sorted(put_map.items(), key=lambda item: item[1] * _distance_weight(strike=item[0], spot=spot_price), reverse=True)
    call_wall = call_candidates[0][0] if call_candidates else None
    put_wall = put_candidates[0][0] if put_candidates else None
    call_strength = _normalized_strength(call_candidates)
    put_strength = _normalized_strength(put_candidates)
    wall_strength = max(0.0, min(1.0, (call_strength + put_strength) / 2.0))
    return call_wall, put_wall, wall_strength


def _pinning_zone(*, spot_price: float, call_wall: float | None, put_wall: float | None) -> tuple[float, float] | None:
    if call_wall is None or put_wall is None:
        return None
    lower = min(put_wall, call_wall)
    upper = max(put_wall, call_wall)
    if not (lower <= spot_price <= upper):
        return (lower, upper)
    padding = max(0.1, (upper - lower) * 0.1)
    return (lower - padding, upper + padding)


def _pinning_strength(*, spot_price: float, pinning_zone: tuple[float, float] | None, wall_strength: float) -> float:
    if pinning_zone is None:
        return 0.0
    lower, upper = pinning_zone
    width = max(0.1, upper - lower)
    center = (lower + upper) / 2.0
    proximity = max(0.0, 1.0 - (abs(spot_price - center) / width))
    return max(0.0, min(1.0, (proximity * 0.6) + (wall_strength * 0.4)))


def _acceleration_zone(
    *,
    spot_price: float,
    gamma_flip_level: float | None,
    call_wall: float | None,
    put_wall: float | None,
    wall_strength: float,
) -> tuple[tuple[float, float] | None, str, float]:
    if gamma_flip_level is None:
        return None, "neutral", 0.0
    upper_ref = call_wall if call_wall is not None else gamma_flip_level * 1.01
    lower_ref = put_wall if put_wall is not None else gamma_flip_level * 0.99
    lower = min(gamma_flip_level, lower_ref)
    upper = max(gamma_flip_level, upper_ref)
    direction = "bullish" if spot_price >= gamma_flip_level else "bearish"
    proximity = max(0.0, 1.0 - (abs(spot_price - gamma_flip_level) / max(0.1, abs(gamma_flip_level) * 0.02)))
    strength = max(0.0, min(1.0, (proximity * 0.7) + (wall_strength * 0.3)))
    return (lower, upper), direction, strength


def _dealer_pressure_score(
    *,
    spot_price: float,
    gamma_flip: float | None,
    wall_strength: float,
    pinning_strength: float,
    acceleration_strength: float,
    acceleration_direction: str,
) -> float:
    proximity = 0.0
    if gamma_flip is not None:
        proximity = max(0.0, 1.0 - (abs(spot_price - gamma_flip) / max(0.1, abs(gamma_flip) * 0.03)))
    direction_bias = 0.55 if acceleration_direction == "bullish" else 0.45 if acceleration_direction == "bearish" else 0.5
    score = 50.0 + ((proximity * 20.0) + (wall_strength * 15.0) + (pinning_strength * 10.0) + (acceleration_strength * 15.0)) * (direction_bias - 0.5)
    return max(0.0, min(100.0, score))


def _flip_confidence(
    *,
    weighted_exposure: Mapping[float, float],
    oi_by_strike: Mapping[float, float],
    spot_price: float,
    gamma_flip: float | None,
) -> float:
    if gamma_flip is None or not weighted_exposure:
        return 0.2
    sign_changes = 0
    ordered = sorted(weighted_exposure.items(), key=lambda item: item[0])
    for idx in range(len(ordered) - 1):
        if ordered[idx][1] * ordered[idx + 1][1] < 0:
            sign_changes += 1
    stability = 1.0 / max(1.0, float(sign_changes))
    oi_density = min(1.0, sum(oi_by_strike.values()) / max(1.0, 1_000_000.0))
    proximity = max(0.0, 1.0 - (abs(spot_price - gamma_flip) / max(0.1, abs(spot_price) * 0.05)))
    return max(0.2, min(1.0, (stability * 0.35) + (oi_density * 0.35) + (proximity * 0.30)))


def _weighted_gamma_exposure(
    *,
    gamma_by_strike: Mapping[float, float],
    oi_by_strike: Mapping[float, float],
    call_oi_by_strike: Mapping[float, float],
    put_oi_by_strike: Mapping[float, float],
    dte_weights: Mapping[str, float],
    spot_price: float,
) -> dict[float, float]:
    out: dict[float, float] = {}
    near_term_weight = max(0.6, min(1.4, float(dte_weights.get("0-7", 1.0)))) if dte_weights else 1.0
    for strike, gamma in gamma_by_strike.items():
        oi_total = oi_by_strike.get(strike, 0.0)
        call_oi = call_oi_by_strike.get(strike, 0.0)
        put_oi = put_oi_by_strike.get(strike, 0.0)
        side_bias = 1.0 + ((call_oi - put_oi) / max(1.0, call_oi + put_oi)) * 0.25
        distance_bias = _distance_weight(strike=strike, spot=spot_price)
        exposure = gamma * max(1.0, oi_total) * near_term_weight * side_bias * distance_bias
        out[strike] = exposure
    return out


def _distance_weight(*, strike: float, spot: float) -> float:
    distance_pct = abs(strike - spot) / max(0.1, abs(spot))
    if distance_pct <= 0.02:
        return 1.0
    if distance_pct <= 0.05:
        return 0.75
    if distance_pct <= 0.1:
        return 0.45
    return 0.25


def _normalized_strength(candidates: list[tuple[float, float]]) -> float:
    if not candidates:
        return 0.0
    top = max(0.0, candidates[0][1])
    tail = sum(max(0.0, item[1]) for item in candidates[:5])
    if tail <= 0:
        return 0.0
    return max(0.0, min(1.0, top / tail))
