from __future__ import annotations

from dataclasses import dataclass

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


def score_symbol(snapshot: SymbolSnapshot) -> ScoredSymbol:
    liquidity_component = normalize_value(float(snapshot.liquidity_score), 0.0, 1.0)
    price_component = normalize_value(float(snapshot.ref_price), 5.0, 500.0)

    event_risk_raw = float(snapshot.meta.get("event_risk", 1.0))
    event_risk_component = 1.0 - normalize_value(event_risk_raw, 0.0, 1.0)

    spread_raw = float(snapshot.meta.get("bid_ask_spread", 1.0))
    spread_component = 1.0 - normalize_value(spread_raw, 0.0, 0.10)

    score = (
        0.40 * liquidity_component
        + 0.20 * price_component
        + 0.25 * event_risk_component
        + 0.15 * spread_component
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


def rank_symbols(snapshots: tuple[SymbolSnapshot, ...]) -> tuple[ScoredSymbol, ...]:
    scored = tuple(score_symbol(snapshot) for snapshot in snapshots)
    return tuple(sorted(scored, key=lambda item: item.score, reverse=True))

