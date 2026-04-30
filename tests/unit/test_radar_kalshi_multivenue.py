from __future__ import annotations

import pytest

from modules.atlas_radar_kalshi.config import RadarSettings
from modules.atlas_radar_kalshi.execution_router import ExecutionModeRouter
from modules.atlas_radar_kalshi.executor_v2 import OrderRequestV2


def _settings() -> RadarSettings:
    return RadarSettings(
        kalshi_api_key_id="demo-key",
        kalshi_private_key_path="C:/ATLAS/config/kalshi_private.pem",
        kalshi_environment="demo",
        execution_mode="paper",
        polymarket_enabled=True,
    )


def test_execution_router_routes_polymarket_ticker() -> None:
    router = ExecutionModeRouter(settings=_settings())
    assert router.is_polymarket("POLY:123")
    assert not router.is_polymarket("KXBTC-24APR")


@pytest.mark.asyncio
async def test_execution_router_paper_submit_polymarket() -> None:
    router = ExecutionModeRouter(settings=_settings())
    req = OrderRequestV2(
        market_ticker="POLY:123",
        side="YES",
        contracts=10,
        price_cents=54,
        client_order_id="test-poly",
    )
    report = await router.submit(req)
    assert report.ok is True
    assert report.filled_contracts == 10
    assert report.status == "filled"

