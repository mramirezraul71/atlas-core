from __future__ import annotations

from atlas_scanner.models import SymbolSnapshot


def _meta_as_float(snapshot: SymbolSnapshot, key: str) -> float | None:
    value = snapshot.meta.get(key)
    if isinstance(value, (int, float, bool)):
        return float(value)
    return None


def liquidity_filter(
    snapshots: tuple[SymbolSnapshot, ...],
    min_volume_20d: float | int,
    max_bid_ask_spread: float | int,
    min_liquidity_score: float | int,
) -> tuple[SymbolSnapshot, ...]:
    kept: list[SymbolSnapshot] = []
    for snapshot in snapshots:
        volume_20d = _meta_as_float(snapshot, "volume_20d")
        bid_ask_spread = _meta_as_float(snapshot, "bid_ask_spread")
        if volume_20d is None or bid_ask_spread is None:
            continue
        if volume_20d < float(min_volume_20d):
            continue
        if bid_ask_spread > float(max_bid_ask_spread):
            continue
        if snapshot.liquidity_score < float(min_liquidity_score):
            continue
        kept.append(snapshot)
    return tuple(kept)


def tradability_filter(
    snapshots: tuple[SymbolSnapshot, ...],
    min_ref_price: float | int,
    max_ref_price: float | int | None = None,
) -> tuple[SymbolSnapshot, ...]:
    kept: list[SymbolSnapshot] = []
    for snapshot in snapshots:
        if snapshot.ref_price < float(min_ref_price):
            continue
        if max_ref_price is not None and snapshot.ref_price > float(max_ref_price):
            continue
        kept.append(snapshot)
    return tuple(kept)


def event_risk_filter(
    snapshots: tuple[SymbolSnapshot, ...],
    max_event_risk: float | int,
) -> tuple[SymbolSnapshot, ...]:
    kept: list[SymbolSnapshot] = []
    for snapshot in snapshots:
        event_risk = _meta_as_float(snapshot, "event_risk")
        if event_risk is None:
            continue
        if event_risk <= float(max_event_risk):
            kept.append(snapshot)
    return tuple(kept)

