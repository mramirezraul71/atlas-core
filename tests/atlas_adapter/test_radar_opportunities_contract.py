"""Contrato HTTP F2: oportunidades multi-símbolo y compatibilidad legacy."""
from __future__ import annotations

import json

import pytest
from fastapi import FastAPI
from fastapi.testclient import TestClient

from atlas_adapter.routes.radar_opportunities import build_radar_opportunities_router
from atlas_adapter.routes.radar_public import build_radar_stub_api_router


@pytest.fixture()
def radar_full_client(monkeypatch: pytest.MonkeyPatch) -> TestClient:
    monkeypatch.setenv("ATLAS_RADAR_MULTI_SYMBOL_ENABLED", "1")
    monkeypatch.setenv("ATLAS_RADAR_MIN_SCORE", "0")
    monkeypatch.setenv("ATLAS_RADAR_MAX_SYMBOLS_PER_BATCH", "15")
    app = FastAPI()
    app.include_router(build_radar_stub_api_router())
    app.include_router(build_radar_opportunities_router())
    return TestClient(app)


@pytest.fixture()
def radar_flag_off(monkeypatch: pytest.MonkeyPatch) -> TestClient:
    monkeypatch.delenv("ATLAS_RADAR_MULTI_SYMBOL_ENABLED", raising=False)
    app = FastAPI()
    app.include_router(build_radar_stub_api_router())
    app.include_router(build_radar_opportunities_router())
    return TestClient(app)


def test_feature_flag_off_503(radar_flag_off: TestClient) -> None:
    r = radar_flag_off.get("/api/radar/opportunities?limit=3&min_score=0")
    assert r.status_code == 503
    body = r.json()
    detail = body.get("detail")
    if isinstance(detail, dict):
        assert detail.get("code") == "RADAR_MULTI_SYMBOL_DISABLED"
    else:
        assert "RADAR_MULTI_SYMBOL_DISABLED" in str(detail)
    assert "no-store" in (r.headers.get("cache-control") or "").lower()


def test_opportunities_shape_and_ranking(radar_full_client: TestClient) -> None:
    r = radar_full_client.get("/api/radar/opportunities?limit=20&min_score=0")
    assert r.status_code == 200
    assert "no-store" in (r.headers.get("cache-control") or "").lower()
    data = r.json()
    assert data.get("ok") is True
    opps = data.get("opportunities") or []
    assert len(opps) >= 1
    for o in opps:
        assert set(o.keys()) >= {
            "symbol",
            "asset_class",
            "score",
            "classification",
            "timestamp",
            "horizon_min",
            "direction",
            "snapshot",
            "degradations_active",
            "source",
            "trace_id",
        }
    scores = [float(x["score"]) for x in opps]
    assert scores == sorted(scores, reverse=True)


def test_legacy_summary_still_works(radar_full_client: TestClient) -> None:
    r = radar_full_client.get("/api/radar/summary?symbol=SPY")
    assert r.status_code == 200
    body = r.json()
    assert body.get("symbol") == "SPY"
    assert "radar" in body


def test_opportunities_detail_spy(radar_full_client: TestClient) -> None:
    r = radar_full_client.get("/api/radar/opportunities/SPY?min_score=0")
    assert r.status_code == 200
    body = r.json()
    assert body.get("ok") is True
    assert body.get("opportunity", {}).get("symbol") == "SPY"


def test_opportunities_sse_envelope_canonical_fields() -> None:
    """El stream F2 reutiliza ``_sse_event_block``; validar contrato de 6 campos."""
    from atlas_adapter.routes.radar_public import _sse_event_block, _utc_iso
    from atlas_adapter.routes.radar_schemas import RadarSseEnvelope

    for etype, sym in (
        ("heartbeat", "*"),
        ("universe_snapshot", "*"),
        ("opportunity_added", "SPY"),
    ):
        env = {
            "type": etype,
            "timestamp": _utc_iso(),
            "symbol": sym,
            "source": "stub",
            "sequence": 3,
            "data": {"probe": True},
        }
        RadarSseEnvelope.model_validate(env)
        block = _sse_event_block(3, etype, env)
        line = [ln for ln in block.splitlines() if ln.startswith("data:")][0]
        parsed = json.loads(line[5:].strip())
        assert set(parsed.keys()) >= {"type", "timestamp", "symbol", "source", "sequence", "data"}
        assert parsed["source"] in {"quant", "stub"}
