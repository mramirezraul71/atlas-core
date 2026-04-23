from __future__ import annotations

from atlas_scanner.models import SymbolSnapshot


def compute_macro(symbol: SymbolSnapshot) -> float:
    _ = symbol
    return 0.0


def classify_macro_regime_from_vix(vix: float | None) -> str | None:
    if vix is None:
        return None
    if vix <= 20.0:
        return "favorable"
    if vix <= 28.0:
        return "neutral"
    return "adverse"


def score_macro_regime(regime: str | None) -> float | None:
    if regime is None:
        return None
    normalized = regime.strip().lower()
    if normalized in {"risk_on", "bullish", "favorable"}:
        return 80.0
    if normalized in {"neutral", "mixed"}:
        return 50.0
    if normalized in {"risk_off", "bearish", "adverse"}:
        return 20.0
    return None


def normalize_event_risk(event_risk: float | None) -> float | None:
    if event_risk is None:
        return None
    bounded = max(0.0, min(1.0, event_risk))
    return (1.0 - bounded) * 100.0


def score_vix_bucket(vix: float | None) -> float | None:
    if vix is None:
        return None
    if vix <= 20.0:
        return 80.0
    if vix <= 28.0:
        return 55.0
    return 30.0


def normalize_seasonal_factor(
    seasonal_factor: float | None,
    seasonal_factor_min: float,
    seasonal_factor_max: float,
) -> float | None:
    if seasonal_factor is None or seasonal_factor_max <= seasonal_factor_min:
        return None
    normalized = (seasonal_factor - seasonal_factor_min) / (seasonal_factor_max - seasonal_factor_min)
    if normalized < 0.0:
        normalized = 0.0
    if normalized > 1.0:
        normalized = 1.0
    return normalized * 100.0

