"""Utilidades de ingestión: etiquetado de venue (bus lógico sobre MarketEvent)."""
from __future__ import annotations

from ..scanner import MarketEvent


def venue_of_event(ev: MarketEvent) -> str:
    pl = (ev.payload or {}) if isinstance(ev.payload, dict) else {}
    src = str(pl.get("source", "") or "").lower()
    if src in ("polymarket", "poly"):
        return "polymarket"
    t = (ev.market_ticker or "").upper()
    if t.startswith("POLY:"):
        return "polymarket"
    return "kalshi"
