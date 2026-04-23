from __future__ import annotations

from atlas_scanner.models import SymbolSnapshot


def compute_price(symbol: SymbolSnapshot) -> float:
    _ = symbol
    return 0.0


def normalize_adx(
    adx: float | None,
    ranging_max: float,
    trending_min: float,
) -> float | None:
    if adx is None:
        return None
    if trending_min <= ranging_max:
        return None
    normalized = (adx - ranging_max) / (trending_min - ranging_max)
    if normalized < 0.0:
        normalized = 0.0
    if normalized > 1.0:
        normalized = 1.0
    return (1.0 - normalized) * 100.0


def interpret_trend_state(trend_state: str | None) -> float | None:
    if trend_state is None:
        return None
    normalized = trend_state.strip().upper()
    if normalized == "RANGING":
        return 80.0
    if normalized in {"TREND_UP", "TREND_DOWN"}:
        return 60.0
    if normalized == "UNKNOWN":
        return 50.0
    return None


def normalize_distance_to_vwap(
    distance: float | None,
    *,
    max_distance: float = 0.05,
) -> float | None:
    if distance is None or max_distance <= 0:
        return None
    normalized = 1.0 - min(abs(distance) / max_distance, 1.0)
    return normalized * 100.0

