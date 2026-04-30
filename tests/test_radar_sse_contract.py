"""Contrato SSE del radar (Bloque 4): formato de líneas y envelope mínimo."""
from __future__ import annotations

import json

import pytest
from fastapi import FastAPI
from fastapi.testclient import TestClient

from atlas_adapter.routes.radar_public import _sse_event_block, build_radar_stub_api_router


@pytest.fixture()
def radar_client() -> TestClient:
    app = FastAPI()
    app.include_router(build_radar_stub_api_router())
    return TestClient(app)


def test_sse_event_block_format() -> None:
    env = {
        "type": "heartbeat",
        "timestamp": "2026-01-01T00:00:00+00:00",
        "symbol": "SPY",
        "source": "stub",
        "sequence": 7,
        "data": {"quant_reachable": False},
    }
    block = _sse_event_block(7, "heartbeat", env)
    assert "id: 7\n" in block or block.startswith("id: 7")
    assert "event: heartbeat\n" in block
    assert "data:" in block
    line = [ln for ln in block.splitlines() if ln.startswith("data:")][0]
    parsed = json.loads(line[len("data:") :].strip())
    assert parsed["sequence"] == 7
    assert parsed["symbol"] == "SPY"
    assert parsed["data"]["quant_reachable"] is False


def test_stream_route_registered_in_openapi(radar_client: TestClient) -> None:
    r = radar_client.get("/openapi.json")
    assert r.status_code == 200
    paths = r.json().get("paths") or {}
    assert any("stream" in p and "radar" in p for p in paths)


def test_symbols_search_route_registered(radar_client: TestClient) -> None:
    r = radar_client.get("/api/radar/symbols/search?q=AA&limit=3")
    assert r.status_code == 200
