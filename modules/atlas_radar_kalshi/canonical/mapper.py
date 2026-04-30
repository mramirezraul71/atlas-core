"""Mapeo canónico enriquecido (el módulo `normalization.py` del paquete raíz)."""
from __future__ import annotations

from .types import CanonicalMarket
from ..normalization import canonical_market_key


def market_from_payload(title: str, close_time: str | None, **venues: str) -> CanonicalMarket:
    key = canonical_market_key(title, close_time)
    day = key.split("|")[-1] if "|" in key else "na"
    return CanonicalMarket(
        key=key,
        title_norm=(title or "")[:500],
        close_day=day,
        source_tickers={k: v for k, v in venues.items() if v},
    )
