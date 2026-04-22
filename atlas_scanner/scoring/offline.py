from __future__ import annotations

from dataclasses import dataclass

from atlas_scanner.config_loader import (
    ComponentWeights,
    DEFAULT_OFFLINE_SCORING_CONFIG,
    GammaScoringConfig,
    OfflineScoringConfig,
    VolScoringConfig,
)
from atlas_scanner.features.builders import build_gamma_features, build_vol_features
from atlas_scanner.models import SymbolSnapshot
from atlas_scanner.models.domain_models import GammaFeatures, VolFeatures


@dataclass(frozen=True)
class ScoredSymbol:
    symbol_snapshot: SymbolSnapshot
    score: float
    component_scores: dict[str, float]
    explanation: str
    strengths: tuple[str, ...]
    penalties: tuple[str, ...]


_COMPONENT_ORDER: tuple[str, ...] = (
    "liquidity",
    "price",
    "event_risk",
    "spread",
    "vol",
    "gamma",
)
_STRENGTH_LABELS: dict[str, str] = {
    "liquidity": "high liquidity",
    "price": "strong price range",
    "event_risk": "contained event risk",
    "spread": "tight bid/ask spread",
    "vol": "favorable volatility setup",
    "gamma": "supportive gamma structure",
}
_PENALTY_LABELS: dict[str, str] = {
    "liquidity": "weak liquidity",
    "price": "unfavorable price range",
    "event_risk": "elevated event risk",
    "spread": "wide bid/ask spread",
    "vol": "weak volatility setup",
    "gamma": "fragile gamma structure",
}


def build_score_explanation(
    component_scores: dict[str, float],
) -> tuple[str, tuple[str, ...], tuple[str, ...]]:
    strengths: list[str] = []
    penalties: list[str] = []

    for component in _COMPONENT_ORDER:
        value = component_scores[component]
        if value >= 0.70:
            strengths.append(_STRENGTH_LABELS[component])
        if value <= 0.30:
            penalties.append(_PENALTY_LABELS[component])

    strengths_tuple = tuple(strengths)
    penalties_tuple = tuple(penalties)

    if strengths_tuple and penalties_tuple:
        explanation = (
            f"Pros: {', '.join(strengths_tuple)}. "
            f"Risks: {', '.join(penalties_tuple)}."
        )
    elif strengths_tuple:
        explanation = f"Pros: {', '.join(strengths_tuple)}."
    elif penalties_tuple:
        explanation = f"Risks: {', '.join(penalties_tuple)}."
    else:
        explanation = "Balanced profile with mixed component signals."

    return explanation, strengths_tuple, penalties_tuple


def normalize_value(value: float, lower: float, upper: float) -> float:
    if upper <= lower:
        return 0.0
    normalized = (value - lower) / (upper - lower)
    if normalized < 0.0:
        return 0.0
    if normalized > 1.0:
        return 1.0
    return normalized


def _as_optional_float(value: object) -> float | None:
    if isinstance(value, (int, float)):
        return float(value)
    return None


def _clamp_0_100(value: float) -> float:
    if value < 0.0:
        return 0.0
    if value > 100.0:
        return 100.0
    return value


def _select_first(values: tuple[float | None, ...]) -> float | None:
    for value in values:
        if value is not None:
            return float(value)
    return None


def _has_vol_signal(features: VolFeatures) -> bool:
    return any(
        value is not None
        for value in (
            features.iv_rank_20d,
            features.iv_rank_50d,
            features.iv_rank_100d,
            features.vrp_20d,
            features.vrp_10d,
            features.vrp_60d,
            features.vrp_5d,
        )
    )


def _has_gamma_signal(features: GammaFeatures) -> bool:
    return any(
        value is not None
        for value in (
            features.gamma_regime,
            features.gamma_flip_agg,
            features.call_wall_nearest,
            features.put_wall_nearest,
            features.net_gex,
        )
    )


def _extract_vol_features(snapshot: SymbolSnapshot) -> VolFeatures:
    iv_current = _as_optional_float(snapshot.meta.get("iv_current"))
    if iv_current is None:
        iv_current = float(snapshot.volatility_lookback)

    iv_history_raw = snapshot.meta.get("iv_history")
    iv_history = iv_history_raw if isinstance(iv_history_raw, (list, tuple)) else None

    rv_annualized_raw = snapshot.meta.get("rv_annualized")
    rv_annualized = rv_annualized_raw if isinstance(rv_annualized_raw, dict) else None

    return build_vol_features(
        iv_current=iv_current,
        iv_history=iv_history,
        rv_annualized=rv_annualized,
    )


def _extract_gamma_features(snapshot: SymbolSnapshot) -> GammaFeatures:
    strike_gamma_raw = snapshot.meta.get("strike_gamma")
    strike_gamma = strike_gamma_raw if isinstance(strike_gamma_raw, (list, tuple)) else None
    net_gamma = _as_optional_float(snapshot.meta.get("net_gamma"))
    neutral_threshold = _as_optional_float(snapshot.meta.get("gamma_neutral_threshold")) or 0.0

    return build_gamma_features(
        strike_gamma=strike_gamma,
        net_gamma=net_gamma,
        neutral_threshold=neutral_threshold if neutral_threshold >= 0 else 0.0,
    )


def compute_vol_score(
    vol_features: VolFeatures | None,
    config: VolScoringConfig,
) -> float:
    """
    Return a normalized 0-100 volatility score.
    """
    if vol_features is None:
        return 0.0

    iv_rank_value = _select_first(
        (vol_features.iv_rank_20d, vol_features.iv_rank_50d, vol_features.iv_rank_100d)
    )
    vrp_value = _select_first((vol_features.vrp_20d, vol_features.vrp_10d, vol_features.vrp_60d, vol_features.vrp_5d))
    if iv_rank_value is None and vrp_value is None:
        return 0.0

    score_parts: list[tuple[float, float]] = []

    if iv_rank_value is not None:
        iv_rank_component = normalize_value(iv_rank_value, config.iv_rank_min, config.iv_rank_max) * 100.0
        score_parts.append((iv_rank_component, 0.60))

    if vrp_value is not None:
        vrp_upper = config.vrp_min + max(1.0, (config.iv_rank_max - config.iv_rank_min) / 2.0)
        vrp_component = normalize_value(vrp_value, config.vrp_min, vrp_upper) * 100.0
        score_parts.append((vrp_component, 0.40))

    total_weight = sum(weight for _, weight in score_parts)
    if total_weight <= 0:
        return 0.0
    weighted = sum(value * weight for value, weight in score_parts) / total_weight
    return _clamp_0_100(weighted)


def compute_gamma_score(
    gamma_features: GammaFeatures | None,
    config: GammaScoringConfig,
) -> float:
    """
    Return a normalized 0-100 gamma score.
    """
    if gamma_features is None:
        return 0.0

    if gamma_features.gamma_regime == "positive":
        base = 80.0
    elif gamma_features.gamma_regime == "neutral":
        base = 50.0
    elif gamma_features.gamma_regime == "negative":
        base = 20.0
    else:
        base = 0.0

    if gamma_features.gamma_flip_agg is not None:
        base += 10.0
    if gamma_features.call_wall_nearest is not None:
        base += 5.0
    if gamma_features.put_wall_nearest is not None:
        base += 5.0

    if config.net_gex_negative_only and gamma_features.net_gex is not None and gamma_features.net_gex > 0:
        base -= 15.0

    return _clamp_0_100(base)


def compute_component_weighted_score(
    *,
    vol_score: float | None,
    gamma_score: float | None,
    component_weights: ComponentWeights,
) -> float:
    """
    Weighted 0-100 combination for available component scores.
    """
    weighted_sum = 0.0
    used_weight = 0.0

    if vol_score is not None:
        weight = max(0.0, component_weights.vol)
        weighted_sum += _clamp_0_100(vol_score) * weight
        used_weight += weight

    if gamma_score is not None:
        weight = max(0.0, component_weights.gamma)
        weighted_sum += _clamp_0_100(gamma_score) * weight
        used_weight += weight

    if used_weight <= 0:
        return 0.0
    return _clamp_0_100(weighted_sum / used_weight)


def score_symbol(
    snapshot: SymbolSnapshot,
    scoring_config: OfflineScoringConfig | None = None,
) -> ScoredSymbol:
    effective_config = scoring_config or DEFAULT_OFFLINE_SCORING_CONFIG
    thresholds = effective_config.thresholds
    weights = effective_config.weights
    component_weights = effective_config.component_weights

    liquidity_component = normalize_value(
        float(snapshot.liquidity_score),
        thresholds.liquidity_lower,
        thresholds.liquidity_upper,
    )
    price_component = normalize_value(
        float(snapshot.ref_price),
        thresholds.price_lower,
        thresholds.price_upper,
    )

    event_risk_raw = float(snapshot.meta.get("event_risk", 1.0))
    event_risk_component = 1.0 - normalize_value(
        event_risk_raw,
        thresholds.event_risk_lower,
        thresholds.event_risk_upper,
    )

    spread_raw = float(snapshot.meta.get("bid_ask_spread", 1.0))
    spread_component = 1.0 - normalize_value(
        spread_raw,
        thresholds.spread_lower,
        thresholds.spread_upper,
    )

    score = (
        weights.liquidity * liquidity_component
        + weights.price * price_component
        + weights.event_risk * event_risk_component
        + weights.spread * spread_component
    )
    base_score = max(0.0, min(1.0, score))

    vol_features = _extract_vol_features(snapshot)
    gamma_features = _extract_gamma_features(snapshot)

    vol_available = _has_vol_signal(vol_features)
    gamma_available = _has_gamma_signal(gamma_features)

    vol_score = compute_vol_score(vol_features, config=effective_config.vol)
    gamma_score = compute_gamma_score(gamma_features, config=effective_config.gamma)

    component_total_0_100 = compute_component_weighted_score(
        vol_score=vol_score if vol_available else None,
        gamma_score=gamma_score if gamma_available else None,
        component_weights=component_weights,
    )
    score = component_total_0_100 / 100.0 if (vol_available or gamma_available) else base_score
    score = max(0.0, min(1.0, score))

    component_scores = {
        "liquidity": liquidity_component,
        "price": price_component,
        "event_risk": event_risk_component,
        "spread": spread_component,
        "vol": (vol_score / 100.0) if vol_available else 0.5,
        "gamma": (gamma_score / 100.0) if gamma_available else 0.5,
    }
    explanation, strengths, penalties = build_score_explanation(component_scores)

    return ScoredSymbol(
        symbol_snapshot=snapshot,
        score=score,
        component_scores=component_scores,
        explanation=explanation,
        strengths=strengths,
        penalties=penalties,
    )


def rank_symbols(
    snapshots: tuple[SymbolSnapshot, ...],
    scoring_config: OfflineScoringConfig | None = None,
) -> tuple[ScoredSymbol, ...]:
    scored = tuple(score_symbol(snapshot, scoring_config=scoring_config) for snapshot in snapshots)
    return tuple(sorted(scored, key=lambda item: item.score, reverse=True))

