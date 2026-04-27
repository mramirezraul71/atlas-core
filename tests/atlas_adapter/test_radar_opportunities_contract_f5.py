"""F5 — Contrato HTTP de los endpoints multi-símbolo del Radar.

Sin sqlalchemy ni app completa: monta sólo el router del radar para aislar
y validar shape de respuesta y filtros.
"""
from __future__ import annotations

import pytest
from fastapi import FastAPI
from fastapi.testclient import TestClient

from atlas_adapter.routes.radar_public import build_radar_stub_api_router


@pytest.fixture()
def client() -> TestClient:
    app = FastAPI()
    app.include_router(build_radar_stub_api_router())
    return TestClient(app)


def test_opportunities_route_present_in_openapi(client: TestClient) -> None:
    r = client.get("/openapi.json")
    assert r.status_code == 200
    paths = r.json().get("paths") or {}
    assert "/api/radar/opportunities" in paths
    assert "/api/radar/opportunities/{symbol}" in paths
    assert "/api/radar/stream/opportunities" in paths


def test_opportunities_returns_envelope_shape(client: TestClient) -> None:
    r = client.get("/api/radar/opportunities?limit=5")
    assert r.status_code == 200
    body = r.json()
    for k in (
        "ok",
        "source",
        "timestamp",
        "universe_size",
        "evaluated",
        "succeeded",
        "failed",
        "returned",
        "limit",
        "min_score",
        "filters",
        "items",
        "degradations_active",
        "trace_id",
    ):
        assert k in body, f"falta campo {k!r} en envelope"
    assert isinstance(body["items"], list)
    assert body["limit"] == 5
    assert body["returned"] <= 5
    assert body["source"] in ("quant", "stub")


def test_opportunities_items_have_canonical_fields(client: TestClient) -> None:
    r = client.get("/api/radar/opportunities?limit=3&min_score=0")
    assert r.status_code == 200
    items = r.json()["items"]
    assert items, "esperado al menos un item con min_score=0"
    for it in items:
        for k in (
            "symbol",
            "asset_class",
            "score",
            "classification",
            "direction",
            "timestamp",
            "snapshot",
            "degradations_active",
            "source",
            "trace_id",
        ):
            assert k in it, f"item sin campo {k!r}: {it}"
        assert it["classification"] in ("high_conviction", "watchlist", "reject")
        assert it["direction"] in ("long", "short", "neutral")
        assert it["source"] in ("quant", "stub")
        assert 0.0 <= float(it["score"]) <= 100.0


def test_opportunities_filter_min_score_excludes_low(client: TestClient) -> None:
    # Sin Quant en test → todos los scores son 0.0 (stub). min_score=1 ⇒ 0 items.
    r = client.get("/api/radar/opportunities?limit=50&min_score=1")
    assert r.status_code == 200
    body = r.json()
    assert body["returned"] == 0
    assert body["items"] == []


def test_opportunities_filter_asset_class(client: TestClient) -> None:
    r = client.get("/api/radar/opportunities?limit=50&asset_class=etf&min_score=0")
    assert r.status_code == 200
    body = r.json()
    assert body["filters"]["asset_class"] == "etf"
    for it in body["items"]:
        assert it["asset_class"] == "etf"


def test_opportunities_filter_optionable_true_excludes_indices(client: TestClient) -> None:
    r = client.get("/api/radar/opportunities?limit=100&optionable=true&min_score=0")
    assert r.status_code == 200
    syms = [it["symbol"] for it in r.json()["items"]]
    # En el universo curado, los índices (SPX/NDX/RUT/VIX) son no-optionable.
    for forbidden in ("SPX", "NDX", "RUT", "VIX"):
        assert forbidden not in syms


def test_opportunity_by_symbol_returns_single(client: TestClient) -> None:
    r = client.get("/api/radar/opportunities/SPY")
    assert r.status_code == 200
    body = r.json()
    assert body["symbol"] == "SPY"
    assert "snapshot" in body
    assert body["source"] in ("quant", "stub")
    assert body["trace_id"]


def test_opportunity_by_symbol_404_for_unknown(client: TestClient) -> None:
    r = client.get("/api/radar/opportunities/ZZZ_NOT_REAL")
    assert r.status_code == 404
    assert r.json().get("detail") == "symbol_not_in_universe"


def test_opportunities_response_headers_indicate_stub_or_live(client: TestClient) -> None:
    r = client.get("/api/radar/opportunities?limit=2")
    assert r.status_code == 200
    # Sin Quant configurado en test → header de stub activo.
    has_stub = r.headers.get("X-Atlas-Radar-Stub") == "1"
    has_live = r.headers.get("X-Atlas-Radar-Source") == "quant"
    assert has_stub or has_live
    # Cache-Control restrictivo en JSON del radar.
    assert "no-store" in (r.headers.get("Cache-Control") or "")


def test_existing_endpoints_unchanged(client: TestClient) -> None:
    """Garantía: F5 no modifica rutas previas."""
    for path in (
        "/api/radar/dashboard/summary",
        "/api/radar/summary",
        "/api/radar/dealer/SPY",
        "/api/radar/diagnostics/providers",
        "/api/radar/sensors/camera/health",
        "/api/radar/camera/status",
    ):
        r = client.get(path)
        assert r.status_code == 200, f"{path} respondió {r.status_code}"
