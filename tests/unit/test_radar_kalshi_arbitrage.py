from __future__ import annotations

from modules.atlas_radar_kalshi.arbitrage_engine import ArbitrageEngine


def test_detects_cross_venue_arb() -> None:
    eng = ArbitrageEngine(min_profit=0.01)
    eng.upsert_quote(
        ticker="KX-CPI-1",
        source="kalshi",
        title="Will CPI be above 3.0% in May?",
        close_time="2026-05-31T23:59:59Z",
        yes_mid=0.62,
    )
    eng.upsert_quote(
        ticker="POLY:abc",
        source="polymarket",
        title="Will CPI be above 3.0% in May?",
        close_time="2026-05-31T23:00:00Z",
        yes_mid=0.33,
    )
    rows = eng.detect()
    assert rows
    assert rows[0]["profit_estimate"] >= 0.01
    assert rows[0]["strategy"] in {"YES_KALSHI_NO_POLY", "NO_KALSHI_YES_POLY"}


def test_build_execution_intents() -> None:
    row = {
        "strategy": "YES_KALSHI_NO_POLY",
        "kalshi_ticker": "KX-1",
        "polymarket_ticker": "POLY:1",
        "kalshi_yes": 0.61,
        "polymarket_yes": 0.44,
    }
    intents = ArbitrageEngine.build_execution_intents(row, contracts=3)
    assert len(intents) == 2
    assert intents[0]["ticker"] == "KX-1"
    assert intents[0]["side"] == "YES"
    assert intents[0]["contracts"] == 3
    assert intents[1]["ticker"] == "POLY:1"
    assert intents[1]["side"] == "NO"

