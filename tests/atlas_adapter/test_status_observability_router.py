from pathlib import Path

from atlas_adapter.routes.status_observability import build_router
from fastapi import FastAPI
from fastapi.testclient import TestClient


def test_status_router_exposes_expected_paths():
    router = build_router(
        Path("C:/repo"),
        lambda: "ok-status",
        lambda: {"connected": True},
        enable_background=False,
    )
    paths = {route.path for route in router.routes}

    assert "/status" in paths
    assert "/health" in paths
    assert "/health/deep" in paths
    assert "/health/debug" in paths
    assert "/metrics" in paths
    assert "/metrics/prometheus" in paths
    assert "/api/observability/metrics" in paths


def test_status_endpoint_uses_providers():
    app = FastAPI()
    app.include_router(
        build_router(
            Path("C:/repo"),
            lambda: "atlas-ready",
            lambda: {"connected": True},
            enable_background=False,
        )
    )

    client = TestClient(app)
    response = client.get("/status")

    assert response.status_code == 200
    payload = response.json()
    assert payload["ok"] is True
    assert payload["atlas"] == "atlas-ready"
    assert payload["nexus_connected"] is True
    assert payload["robot_connected"] is True


def test_health_endpoint_returns_basic_shape():
    app = FastAPI()
    app.include_router(
        build_router(
            Path("C:/repo"),
            lambda: "atlas-ready",
            lambda: {"connected": False},
            enable_background=False,
        )
    )

    client = TestClient(app)
    response = client.get("/health")

    assert response.status_code == 200
    payload = response.json()
    assert payload["service"] == "atlas_push"
    assert "ts" in payload
    assert "pid" in payload
