"""Imports de la reorganización por dominios (regresión de empaquetado)."""
from __future__ import annotations

import pytest

import modules.atlas_radar_kalshi as m
from modules.atlas_radar_kalshi.canonical import market_from_payload
from modules.atlas_radar_kalshi.execution import reconcile_kalshi_from_router
from modules.atlas_radar_kalshi.ingestion import venue_of_event
from modules.atlas_radar_kalshi.scanner import MarketEvent


def test_public_exports() -> None:
    assert m.compute_calibration_kpis is not None
    assert m.venue_of_event is not None
    assert market_from_payload("Test Title?", "2026-01-01T00:00:00Z") is not None


def test_venue_of_event_poly() -> None:
    ev = MarketEvent(
        kind="ticker",
        market_ticker="POLY:1",
        payload={"source": "polymarket"},
    )
    assert venue_of_event(ev) == "polymarket"


@pytest.mark.asyncio
async def test_reconcile_delegates() -> None:
    class K:
        async def reconcile(self) -> dict:
            return {"paper": True}

    class R:
        kalshi = K()

    out = await reconcile_kalshi_from_router(R())
    assert out.get("paper") is True
