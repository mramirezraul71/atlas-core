from __future__ import annotations

from atlas_scanner.models import SymbolSnapshot


def compute_flow(symbol: SymbolSnapshot) -> float:
    _ = symbol
    return 0.0


def normalize_oi_change(
    oi_change_1d_pct: float | None,
    oi_change_min_pct: float,
    *,
    oi_change_window: float = 10.0,
) -> float | None:
    if oi_change_1d_pct is None:
        return None
    upper = max(oi_change_min_pct + oi_change_window, oi_change_min_pct + 1.0)
    normalized = (oi_change_1d_pct - oi_change_min_pct) / (upper - oi_change_min_pct)
    if normalized < 0.0:
        normalized = 0.0
    if normalized > 1.0:
        normalized = 1.0
    return normalized * 100.0


def normalize_call_put_volume_ratio(
    ratio: float | None,
    ratio_min: float,
    ratio_max: float,
) -> float | None:
    if ratio is None:
        return None
    upper = min(2.0, ratio_max)
    upper = max(ratio_min + 0.1, upper)
    normalized = (ratio - ratio_min) / (upper - ratio_min)
    if normalized < 0.0:
        normalized = 0.0
    if normalized > 1.0:
        normalized = 1.0
    return normalized * 100.0


def resolve_volume_imbalance(
    volume_imbalance: float | None,
    call_volume: float | None,
    put_volume: float | None,
) -> float | None:
    if volume_imbalance is not None:
        return volume_imbalance
    if call_volume is None or put_volume is None:
        return None
    total = call_volume + put_volume
    if total <= 0:
        return None
    return (call_volume - put_volume) / total


def normalize_volume_imbalance(imbalance: float | None) -> float | None:
    if imbalance is None:
        return None
    bounded = max(-1.0, min(1.0, imbalance))
    return ((bounded + 1.0) / 2.0) * 100.0

