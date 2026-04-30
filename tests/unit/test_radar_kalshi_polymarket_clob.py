from __future__ import annotations

from pathlib import Path

import pytest

from modules.atlas_radar_kalshi.config import RadarSettings
from modules.atlas_radar_kalshi.executor_v2 import OrderRequestV2
from modules.atlas_radar_kalshi.polymarket_executor import (
    PolymarketExecutor,
    PolymarketExecutorConfig,
    _clob_yes_no_tokens,
    _parse_poly_ticker,
)


def test_parse_poly_ticker() -> None:
    assert _parse_poly_ticker("POLY:540816") == "540816"


def test_clob_yes_no_tokens_from_str_json() -> None:
    m = {
        "clobTokenIds": '["111", "222"]',
    }
    assert _clob_yes_no_tokens(m) == ("111", "222")


@pytest.mark.asyncio
async def test_polymarket_live_submit_patched_clob() -> None:
    settings = RadarSettings(
        kalshi_api_key_id="k",
        kalshi_private_key_path=Path("C:/ATLAS/config/kalshi_private.pem"),
        kalshi_environment="demo",
        execution_mode="live",
        polymarket_enabled=True,
        polymarket_private_key="0x" + "1" * 64,
        polymarket_funder=None,
    )
    ex = PolymarketExecutor(
        settings=settings,
        cfg=PolymarketExecutorConfig(enable_live=True),
    )

    async def _fake_fetch(_mid: str) -> dict:
        return {
            "clobTokenIds": '["yes-token", "no-token"]',
            "orderPriceMinTickSize": 0.01,
            "orderMinSize": 1.0,
        }

    def _fake_place(*, token_id: str, price: float, size: float) -> dict:
        assert token_id == "yes-token"
        assert abs(price - 0.55) < 1e-9
        assert size == 5.0
        return {"orderID": "ord-1", "success": True}

    ex._fetch_gamma_market = _fake_fetch  # type: ignore[method-assign]
    ex._place_limit_sync = _fake_place  # type: ignore[method-assign]

    req = OrderRequestV2(
        market_ticker="POLY:540816",
        side="YES",
        contracts=5,
        price_cents=55,
        client_order_id="c1",
    )
    rep = await ex.submit(req)
    assert rep.ok is True
    assert rep.order_id == "ord-1"
    assert rep.status == "submitted"


@pytest.mark.asyncio
async def test_polymarket_live_rejects_below_min_size() -> None:
    settings = RadarSettings(
        kalshi_api_key_id="k",
        kalshi_private_key_path=Path("C:/ATLAS/config/kalshi_private.pem"),
        kalshi_environment="demo",
        execution_mode="live",
        polymarket_enabled=True,
        polymarket_private_key="0x" + "1" * 64,
    )
    ex = PolymarketExecutor(
        settings=settings,
        cfg=PolymarketExecutorConfig(enable_live=True),
    )

    async def _fake_fetch(_mid: str) -> dict:
        return {
            "clobTokenIds": '["a", "b"]',
            "orderMinSize": 10.0,
            "orderPriceMinTickSize": 0.01,
        }

    ex._fetch_gamma_market = _fake_fetch  # type: ignore[method-assign]

    req = OrderRequestV2(
        market_ticker="POLY:1",
        side="NO",
        contracts=2,
        price_cents=50,
        client_order_id="c2",
    )
    rep = await ex.submit(req)
    assert rep.ok is False
    assert rep.error == "below_order_min_size"
