"""Contract test F7 — endpoint GET /vision/confirm."""
from __future__ import annotations

import pytest
from fastapi import FastAPI
from fastapi.testclient import TestClient

from atlas_adapter.routes.vision_confirm import build_vision_confirm_router


@pytest.fixture()
def client_disabled(monkeypatch: pytest.MonkeyPatch) -> TestClient:
    monkeypatch.delenv("ATLAS_VISION_GATE_ENABLED", raising=False)
    app = FastAPI()
    app.include_router(build_vision_confirm_router())
    return TestClient(app)


@pytest.fixture()
def client_enabled(monkeypatch: pytest.MonkeyPatch) -> TestClient:
    monkeypatch.setenv("ATLAS_VISION_GATE_ENABLED", "true")
    app = FastAPI()
    app.include_router(build_vision_confirm_router())
    return TestClient(app)


def _assert_schema(payload: dict) -> None:
    for key in (
        "decision",
        "confidence",
        "imbalance_side",
        "pattern",
        "ts",
        "degraded",
        "camera_state",
    ):
        assert key in payload, f"missing key {key!r}"
    assert payload["decision"] in {"allow", "delay", "block", "force_exit"}
    assert payload["camera_state"] in {"CAMERA_OK", "CAMERA_UNAVAILABLE", "CAMERA_DEGRADED"}
    assert isinstance(payload["confidence"], (int, float))


def test_disabled_returns_paper_default_allow(client_disabled: TestClient) -> None:
    r = client_disabled.get("/vision/confirm?symbol=AAPL&intent=entry&strategy=vertical_spread")
    assert r.status_code == 200
    body = r.json()
    _assert_schema(body)
    assert body["decision"] == "allow"
    assert body["degraded"] is True
    assert body["symbol"] == "AAPL"
    assert body["reason"] == "vision_gate_disabled_paper_default"


def test_enabled_camera_unavailable_no_visual_required_returns_allow(
    client_enabled: TestClient,
) -> None:
    r = client_enabled.get("/vision/confirm?symbol=msft&intent=entry&strategy=iron_condor")
    assert r.status_code == 200
    body = r.json()
    _assert_schema(body)
    assert body["symbol"] == "MSFT"
    # Cámara no disponible y no se exige visual → allow degraded
    assert body["decision"] == "allow"
    assert body["degraded"] is True


def test_enabled_visual_required_blocks(client_enabled: TestClient) -> None:
    r = client_enabled.get(
        "/vision/confirm?symbol=AAPL&intent=entry&strategy=x"
        "&requires_visual_confirmation=true"
    )
    assert r.status_code == 200
    body = r.json()
    _assert_schema(body)
    assert body["decision"] == "block"
    assert body["camera_state"] == "CAMERA_UNAVAILABLE"


def test_invalid_intent_rejected(client_enabled: TestClient) -> None:
    r = client_enabled.get("/vision/confirm?symbol=AAPL&intent=foo")
    assert r.status_code == 422
