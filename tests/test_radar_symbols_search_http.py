"""Cabeceras HTTP del proxy de búsqueda de símbolos del radar (Bloque 2)."""

from __future__ import annotations

import pytest
from fastapi import FastAPI
from fastapi.testclient import TestClient

from atlas_adapter.routes.radar_public import build_radar_stub_api_router


@pytest.fixture()
def radar_only_client() -> TestClient:
    app = FastAPI()
    app.include_router(build_radar_stub_api_router())
    return TestClient(app)


def test_symbols_search_cache_control_private_short_ttl(radar_only_client: TestClient) -> None:
    r = radar_only_client.get("/api/radar/symbols/search?q=AA&limit=3")
    assert r.status_code == 200
    cc = (r.headers.get("cache-control") or "").lower()
    assert "private" in cc
    assert "max-age=12" in cc.replace(" ", "")
    assert "must-revalidate" in cc
    assert "stale-while-revalidate=24" in cc.replace(" ", "")
    body = r.json()
    assert "ok" in body
    assert "matches" in body


def test_symbols_search_pragma_no_cache(radar_only_client: TestClient) -> None:
    r = radar_only_client.get("/api/radar/symbols/search?q=X")
    assert r.status_code == 200
    assert (r.headers.get("pragma") or "").lower() == "no-cache"
