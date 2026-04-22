from __future__ import annotations

from dataclasses import dataclass

from atlas_scanner.config_loader import DEFAULT_OFFLINE_SCORING_CONFIG, OfflineScoringConfig
from atlas_scanner.models import SymbolSnapshot


@dataclass(frozen=True)
class ScoredSymbol:
    symbol_snapshot: SymbolSnapshot
    score: float
    component_scores: dict[str, float]


def normalize_value(value: float, lower: float, upper: float) -> float:
    if upper <= lower:
        return 0.0
    normalized = (value - lower) / (upper - lower)
    if normalized < 0.0:
        return 0.0
    if normalized > 1.0:
        return 1.0
    return normalized


def score_symbol(
    snapshot: SymbolSnapshot,
    scoring_config: OfflineScoringConfig | None = None,
) -> ScoredSymbol:
    effective_config = scoring_config or DEFAULT_OFFLINE_SCORING_CONFIG
    thresholds = effective_config.thresholds
    weights = effective_config.weights

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
    score = max(0.0, min(1.0, score))

    component_scores = {
        "liquidity": liquidity_component,
        "price": price_component,
        "event_risk": event_risk_component,
        "spread": spread_component,
    }
    return ScoredSymbol(
        symbol_snapshot=snapshot,
        score=score,
        component_scores=component_scores,
    )


def rank_symbols(
    snapshots: tuple[SymbolSnapshot, ...],
    scoring_config: OfflineScoringConfig | None = None,
) -> tuple[ScoredSymbol, ...]:
    scored = tuple(score_symbol(snapshot, scoring_config=scoring_config) for snapshot in snapshots)
    return tuple(sorted(scored, key=lambda item: item.score, reverse=True))

