from __future__ import annotations

from dataclasses import dataclass

from atlas_scanner.config_loader import (
    ComponentWeights,
    DEFAULT_OFFLINE_SCORING_CONFIG,
    GammaScoringConfig,
    MacroScoringConfig,
    OIFlowScoringConfig,
    OfflineScoringConfig,
    PriceScoringConfig,
    VolScoringConfig,
)
from atlas_scanner.features.builders import build_gamma_features, build_vol_features
from atlas_scanner.features.flow import (
    normalize_call_put_volume_ratio,
    normalize_oi_change,
    normalize_volume_imbalance,
    resolve_volume_imbalance,
)
from atlas_scanner.features.macro import (
    normalize_event_risk,
    normalize_seasonal_factor,
    score_macro_regime,
    score_vix_bucket,
)
from atlas_scanner.features.price import (
    interpret_trend_state,
    normalize_adx,
    normalize_distance_to_vwap,
)
from atlas_scanner.models import SymbolSnapshot
from atlas_scanner.models.domain_models import GammaFeatures, VolFeatures


@dataclass(frozen=True)
class ComponentExplanation:
    name: str
    score: float | None
    status: str
    reasons: tuple[str, ...]


@dataclass(frozen=True)
class ScoredSymbol:
    symbol_snapshot: SymbolSnapshot
    score: float
    component_scores: dict[str, float]
    component_explanations: dict[str, ComponentExplanation]
    top_reasons: tuple[str, ...]
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
    "oi_flow",
    "macro",
)
_STRENGTH_LABELS: dict[str, str] = {
    "liquidity": "high liquidity",
    "price": "strong price range",
    "event_risk": "contained event risk",
    "spread": "tight bid/ask spread",
    "vol": "favorable volatility setup",
    "gamma": "supportive gamma structure",
    "oi_flow": "supportive options flow",
    "macro": "favorable macro backdrop",
}
_PENALTY_LABELS: dict[str, str] = {
    "liquidity": "weak liquidity",
    "price": "unfavorable price range",
    "event_risk": "elevated event risk",
    "spread": "wide bid/ask spread",
    "vol": "weak volatility setup",
    "gamma": "fragile gamma structure",
    "oi_flow": "adverse options flow",
    "macro": "adverse macro backdrop",
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


def _status_from_score(score: float | None, *, available: bool) -> str:
    if not available or score is None:
        return "unavailable"
    if score >= 60.0:
        return "positive"
    if score < 40.0:
        return "negative"
    return "neutral"


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


def compute_price_score(
    snapshot: SymbolSnapshot,
    config: PriceScoringConfig,
) -> float:
    """
    Return a normalized 0-100 price-action score from available snapshot fields.
    """
    score_parts: list[tuple[float, float]] = []

    adx_score = normalize_adx(
        adx=_as_optional_float(snapshot.meta.get("adx_14")),
        ranging_max=config.adx_ranging_max,
        trending_min=config.adx_trending_min,
    )
    if adx_score is not None:
        score_parts.append((_clamp_0_100(adx_score), 0.50))

    trend_score = interpret_trend_state(
        trend_state=snapshot.meta.get("trend_state")
        if isinstance(snapshot.meta.get("trend_state"), str)
        else None
    )
    if trend_score is not None:
        score_parts.append((_clamp_0_100(trend_score), 0.30))

    vwap_score = normalize_distance_to_vwap(
        distance=_as_optional_float(snapshot.meta.get("distance_to_vwap")),
        max_distance=0.05,
    )
    if vwap_score is not None:
        score_parts.append((_clamp_0_100(vwap_score), 0.20))

    if not score_parts:
        return 0.0
    total_weight = sum(weight for _, weight in score_parts)
    weighted = sum(value * weight for value, weight in score_parts) / total_weight
    return _clamp_0_100(weighted)


def compute_macro_score(
    snapshot: SymbolSnapshot,
    config: MacroScoringConfig,
) -> float:
    """
    Return a normalized 0-100 macro/context score from available snapshot fields.
    """
    score_parts: list[tuple[float, float]] = []

    regime_score = score_macro_regime(
        regime=snapshot.meta.get("macro_regime")
        if isinstance(snapshot.meta.get("macro_regime"), str)
        else None
    )
    if regime_score is not None:
        score_parts.append((regime_score, 0.40))

    event_risk = _as_optional_float(snapshot.meta.get("event_risk"))
    event_score = normalize_event_risk(event_risk=event_risk)
    if event_score is not None:
        score_parts.append((event_score, 0.40))
        if config.block_on_event_risk_high and event_risk is not None and event_risk >= 0.9:
            return 0.0

    vix_score = score_vix_bucket(vix=_as_optional_float(snapshot.meta.get("vix")))
    if vix_score is not None:
        score_parts.append((vix_score, 0.20))

    seasonal_score = normalize_seasonal_factor(
        seasonal_factor=_as_optional_float(snapshot.meta.get("seasonal_factor")),
        seasonal_factor_min=config.seasonal_factor_min,
        seasonal_factor_max=config.seasonal_factor_max,
    )
    if seasonal_score is not None:
        score_parts.append((seasonal_score, 0.20))

    if not score_parts:
        return 0.0
    total_weight = sum(weight for _, weight in score_parts)
    weighted = sum(value * weight for value, weight in score_parts) / total_weight
    return _clamp_0_100(weighted)


def compute_oi_flow_score(
    snapshot: SymbolSnapshot,
    config: OIFlowScoringConfig,
) -> float:
    """
    Return a normalized 0-100 score from options OI/flow signals.

    Semantics:
    - Higher scores indicate call-supportive or balanced upside flow.
    - Lower scores indicate put-heavy / defensive flow.
    """
    score_parts: list[tuple[float, float]] = []

    oi_change_score = normalize_oi_change(
        oi_change_1d_pct=_as_optional_float(snapshot.meta.get("oi_change_1d_pct")),
        oi_change_min_pct=config.oi_change_min_pct,
    )
    if oi_change_score is not None:
        score_parts.append((_clamp_0_100(oi_change_score), 0.40))

    ratio_score = normalize_call_put_volume_ratio(
        ratio=_as_optional_float(snapshot.meta.get("call_put_volume_ratio")),
        ratio_min=config.call_put_volume_ratio_min,
        ratio_max=config.call_put_volume_ratio_max,
    )
    if ratio_score is not None:
        score_parts.append((_clamp_0_100(ratio_score), 0.40))

    imbalance_value = resolve_volume_imbalance(
        volume_imbalance=_as_optional_float(snapshot.meta.get("volume_imbalance")),
        call_volume=_as_optional_float(snapshot.meta.get("call_volume")),
        put_volume=_as_optional_float(snapshot.meta.get("put_volume")),
    )
    imbalance_score = normalize_volume_imbalance(imbalance=imbalance_value)
    if imbalance_score is not None:
        score_parts.append((_clamp_0_100(imbalance_score), 0.20))

    if not score_parts:
        return 0.0
    total_weight = sum(weight for _, weight in score_parts)
    weighted = sum(value * weight for value, weight in score_parts) / total_weight
    return _clamp_0_100(weighted)


def explain_vol_component(
    *,
    vol_features: VolFeatures,
    vol_score: float | None,
    available: bool,
) -> ComponentExplanation:
    if not available or vol_score is None:
        return ComponentExplanation(
            name="vol",
            score=None,
            status="unavailable",
            reasons=("volatility inputs unavailable",),
        )

    reasons: list[str] = []
    iv_rank = _select_first((vol_features.iv_rank_20d, vol_features.iv_rank_50d, vol_features.iv_rank_100d))
    vrp = _select_first((vol_features.vrp_20d, vol_features.vrp_10d, vol_features.vrp_60d, vol_features.vrp_5d))
    if iv_rank is not None:
        if iv_rank >= 70.0:
            reasons.append("IV Rank elevated")
        elif iv_rank < 30.0:
            reasons.append("IV Rank muted")
        else:
            reasons.append("IV Rank mid-range")
    if vrp is not None:
        if vrp > 0.0:
            reasons.append("positive volatility risk premium")
        else:
            reasons.append("flat/negative volatility risk premium")
    if not reasons:
        reasons.append("volatility setup mixed")
    return ComponentExplanation(
        name="vol",
        score=vol_score,
        status=_status_from_score(vol_score, available=True),
        reasons=tuple(reasons),
    )


def explain_gamma_component(
    *,
    gamma_features: GammaFeatures,
    gamma_score: float | None,
    available: bool,
) -> ComponentExplanation:
    if not available or gamma_score is None:
        return ComponentExplanation(
            name="gamma",
            score=None,
            status="unavailable",
            reasons=("gamma inputs unavailable",),
        )

    reasons: list[str] = []
    if gamma_features.gamma_regime == "positive":
        reasons.append("positive gamma regime")
    elif gamma_features.gamma_regime == "negative":
        reasons.append("negative gamma regime")
    elif gamma_features.gamma_regime == "neutral":
        reasons.append("neutral gamma regime")
    if gamma_features.gamma_flip_agg is not None:
        reasons.append("gamma flip level identified")
    if gamma_features.call_wall_nearest is not None and gamma_features.put_wall_nearest is not None:
        reasons.append("well-defined gamma wall structure")
    if not reasons:
        reasons.append("gamma structure mixed")
    return ComponentExplanation(
        name="gamma",
        score=gamma_score,
        status=_status_from_score(gamma_score, available=True),
        reasons=tuple(reasons),
    )


def explain_price_component(
    *,
    snapshot: SymbolSnapshot,
    price_score: float | None,
    available: bool,
) -> ComponentExplanation:
    if not available or price_score is None:
        return ComponentExplanation(
            name="price",
            score=None,
            status="unavailable",
            reasons=("price-action inputs unavailable",),
        )

    reasons: list[str] = []
    trend_state = snapshot.meta.get("trend_state")
    if isinstance(trend_state, str):
        normalized = trend_state.strip().upper()
        if normalized == "TREND_UP":
            reasons.append("uptrend price action")
        elif normalized == "TREND_DOWN":
            reasons.append("downtrend price action")
        elif normalized == "RANGING":
            reasons.append("range-bound price action")
    adx_14 = _as_optional_float(snapshot.meta.get("adx_14"))
    if adx_14 is not None:
        if adx_14 >= 30.0:
            reasons.append("strong trend strength (ADX)")
        elif adx_14 <= 25.0:
            reasons.append("contained trend strength (ADX)")
    distance_to_vwap = _as_optional_float(snapshot.meta.get("distance_to_vwap"))
    if distance_to_vwap is not None:
        if abs(distance_to_vwap) <= 0.01:
            reasons.append("price anchored near VWAP")
        else:
            reasons.append("price stretched from VWAP")
    if not reasons:
        reasons.append("price-action setup mixed")
    return ComponentExplanation(
        name="price",
        score=price_score,
        status=_status_from_score(price_score, available=True),
        reasons=tuple(reasons),
    )


def explain_macro_component(
    *,
    snapshot: SymbolSnapshot,
    macro_score: float | None,
    available: bool,
    config: MacroScoringConfig,
) -> ComponentExplanation:
    if not available or macro_score is None:
        return ComponentExplanation(
            name="macro",
            score=None,
            status="unavailable",
            reasons=("macro inputs unavailable",),
        )

    reasons: list[str] = []
    regime = snapshot.meta.get("macro_regime")
    if isinstance(regime, str):
        normalized = regime.strip().lower()
        if normalized in {"risk_on", "bullish", "favorable"}:
            reasons.append("macro regime supportive")
        elif normalized in {"risk_off", "bearish", "adverse"}:
            reasons.append("macro regime adverse")
        else:
            reasons.append("macro regime neutral")

    event_risk = _as_optional_float(snapshot.meta.get("event_risk"))
    if event_risk is not None:
        if config.block_on_event_risk_high and event_risk >= 0.9:
            reasons.append("macro blocked by elevated event risk")
        elif event_risk >= 0.6:
            reasons.append("elevated event risk")
        else:
            reasons.append("event risk contained")

    vix_value = _as_optional_float(snapshot.meta.get("vix"))
    if vix_value is not None:
        if vix_value > 28.0:
            reasons.append("high volatility macro backdrop")
        elif vix_value <= 20.0:
            reasons.append("calm volatility macro backdrop")

    seasonal_factor = _as_optional_float(snapshot.meta.get("seasonal_factor"))
    if seasonal_factor is not None:
        if seasonal_factor >= 1.2:
            reasons.append("seasonal factor supportive")
        elif seasonal_factor < 0.8:
            reasons.append("seasonal factor weak")

    if not reasons:
        reasons.append("macro backdrop mixed")
    return ComponentExplanation(
        name="macro",
        score=macro_score,
        status=_status_from_score(macro_score, available=True),
        reasons=tuple(reasons),
    )


def explain_oi_flow_component(
    *,
    snapshot: SymbolSnapshot,
    oi_flow_score: float | None,
    available: bool,
) -> ComponentExplanation:
    if not available or oi_flow_score is None:
        return ComponentExplanation(
            name="oi_flow",
            score=None,
            status="unavailable",
            reasons=("oi/flow inputs unavailable",),
        )

    reasons: list[str] = []
    oi_change_1d_pct = _as_optional_float(snapshot.meta.get("oi_change_1d_pct"))
    if oi_change_1d_pct is not None:
        if oi_change_1d_pct > 0:
            reasons.append("open interest expanding")
        else:
            reasons.append("open interest contracting")

    ratio = _as_optional_float(snapshot.meta.get("call_put_volume_ratio"))
    if ratio is not None:
        if ratio >= 1.2:
            reasons.append("call-heavy options flow")
        elif ratio <= 0.8:
            reasons.append("put-heavy options flow")
        else:
            reasons.append("balanced call/put flow")

    imbalance = _as_optional_float(snapshot.meta.get("volume_imbalance"))
    if imbalance is not None:
        if abs(imbalance) >= 0.5:
            reasons.append("strong directional flow imbalance")
        else:
            reasons.append("mild options flow imbalance")

    if not reasons:
        reasons.append("options flow mixed")
    return ComponentExplanation(
        name="oi_flow",
        score=oi_flow_score,
        status=_status_from_score(oi_flow_score, available=True),
        reasons=tuple(reasons),
    )


def summarize_score_explanation(
    component_explanations: dict[str, ComponentExplanation],
    total_score: float,
) -> tuple[str, ...]:
    available = [exp for exp in component_explanations.values() if exp.status != "unavailable"]
    if not available:
        return ("multifactor inputs unavailable; legacy scoring fallback applied",)

    positives = sorted(
        [exp for exp in available if exp.status == "positive" and exp.reasons],
        key=lambda item: item.score or 0.0,
        reverse=True,
    )
    negatives = sorted(
        [exp for exp in available if exp.status == "negative" and exp.reasons],
        key=lambda item: item.score or 100.0,
    )
    neutrals = [exp for exp in available if exp.status == "neutral" and exp.reasons]

    chosen: list[str] = []
    if total_score >= 0.60:
        chosen.extend(exp.reasons[0] for exp in positives[:3])
        if len(chosen) < 2:
            chosen.extend(exp.reasons[0] for exp in neutrals[:2])
    elif total_score < 0.40:
        chosen.extend(exp.reasons[0] for exp in negatives[:3])
        if len(chosen) < 2:
            chosen.extend(exp.reasons[0] for exp in neutrals[:2])
    else:
        if positives:
            chosen.append(positives[0].reasons[0])
        if negatives:
            chosen.append(negatives[0].reasons[0])
        chosen.extend(exp.reasons[0] for exp in neutrals[:2])

    if len(chosen) < 2:
        fallback = [exp.reasons[0] for exp in available if exp.reasons]
        for reason in fallback:
            if reason not in chosen:
                chosen.append(reason)
            if len(chosen) >= 2:
                break

    return tuple(chosen[:4])


def compute_component_weighted_score(
    *,
    vol_score: float | None,
    gamma_score: float | None,
    oi_flow_score: float | None,
    price_score: float | None,
    macro_score: float | None,
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

    if oi_flow_score is not None:
        weight = max(0.0, component_weights.oi_flow)
        weighted_sum += _clamp_0_100(oi_flow_score) * weight
        used_weight += weight

    if price_score is not None:
        weight = max(0.0, component_weights.price)
        weighted_sum += _clamp_0_100(price_score) * weight
        used_weight += weight

    if macro_score is not None:
        weight = max(0.0, component_weights.macro)
        weighted_sum += _clamp_0_100(macro_score) * weight
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
    oi_flow_available = any(
        snapshot.meta.get(key) is not None
        for key in (
            "oi_change_1d_pct",
            "call_put_volume_ratio",
            "volume_imbalance",
            "call_volume",
            "put_volume",
        )
    )
    price_available = any(
        snapshot.meta.get(key) is not None
        for key in ("adx_14", "trend_state", "distance_to_vwap")
    )
    macro_available = any(
        snapshot.meta.get(key) is not None
        for key in ("macro_regime", "vix", "seasonal_factor")
    )

    vol_score = compute_vol_score(vol_features, config=effective_config.vol)
    gamma_score = compute_gamma_score(gamma_features, config=effective_config.gamma)
    oi_flow_signal_score = compute_oi_flow_score(snapshot, config=effective_config.oi_flow)
    price_signal_score = compute_price_score(snapshot, config=effective_config.price)
    macro_signal_score = compute_macro_score(snapshot, config=effective_config.macro)

    component_total_0_100 = compute_component_weighted_score(
        vol_score=vol_score if vol_available else None,
        gamma_score=gamma_score if gamma_available else None,
        oi_flow_score=oi_flow_signal_score if oi_flow_available else None,
        price_score=price_signal_score if price_available else None,
        macro_score=macro_signal_score if macro_available else None,
        component_weights=component_weights,
    )
    has_new_component = (
        vol_available or gamma_available or oi_flow_available or price_available or macro_available
    )
    score = component_total_0_100 / 100.0 if has_new_component else base_score
    score = max(0.0, min(1.0, score))

    component_scores = {
        "liquidity": liquidity_component,
        "price": (price_signal_score / 100.0) if price_available else price_component,
        "event_risk": event_risk_component,
        "spread": spread_component,
        "vol": (vol_score / 100.0) if vol_available else 0.5,
        "gamma": (gamma_score / 100.0) if gamma_available else 0.5,
        "oi_flow": (oi_flow_signal_score / 100.0) if oi_flow_available else 0.5,
        "macro": (macro_signal_score / 100.0) if macro_available else 0.5,
    }
    component_explanations = {
        "vol": explain_vol_component(
            vol_features=vol_features,
            vol_score=vol_score if vol_available else None,
            available=vol_available,
        ),
        "gamma": explain_gamma_component(
            gamma_features=gamma_features,
            gamma_score=gamma_score if gamma_available else None,
            available=gamma_available,
        ),
        "oi_flow": explain_oi_flow_component(
            snapshot=snapshot,
            oi_flow_score=oi_flow_signal_score if oi_flow_available else None,
            available=oi_flow_available,
        ),
        "price": explain_price_component(
            snapshot=snapshot,
            price_score=price_signal_score if price_available else None,
            available=price_available,
        ),
        "macro": explain_macro_component(
            snapshot=snapshot,
            macro_score=macro_signal_score if macro_available else None,
            available=macro_available,
            config=effective_config.macro,
        ),
    }
    top_reasons = summarize_score_explanation(component_explanations, total_score=score)
    explanation, strengths, penalties = build_score_explanation(component_scores)

    return ScoredSymbol(
        symbol_snapshot=snapshot,
        score=score,
        component_scores=component_scores,
        component_explanations=component_explanations,
        top_reasons=top_reasons,
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

