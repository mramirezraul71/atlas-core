"""Smoke test for the /lotto-quant redirect endpoint.

Verifies the route is registered and emits a 302 to the configured
ATLAS_LOTTO_QUANT_URL (or the documented default).
"""

from __future__ import annotations

import os


def _get_route(app, path: str):
    for route in app.routes:
        if getattr(route, "path", None) == path:
            return route
    return None


def test_lotto_quant_route_registered():
    from atlas_adapter.atlas_http_api import app
    route = _get_route(app, "/lotto-quant")
    assert route is not None, "GET /lotto-quant must be registered"
    methods = getattr(route, "methods", set()) or set()
    assert "GET" in methods


def test_lotto_quant_redirect_default(monkeypatch):
    from fastapi.testclient import TestClient
    monkeypatch.delenv("ATLAS_LOTTO_QUANT_URL", raising=False)
    from atlas_adapter.atlas_http_api import app

    with TestClient(app, follow_redirects=False) as client:
        r = client.get("/lotto-quant")
    assert r.status_code in (302, 307)
    assert r.headers["location"] == "http://127.0.0.1:8501"


def test_lotto_quant_redirect_custom_env(monkeypatch):
    from fastapi.testclient import TestClient
    monkeypatch.setenv("ATLAS_LOTTO_QUANT_URL", "https://lotto.example.com:9000/")
    from atlas_adapter.atlas_http_api import app

    with TestClient(app, follow_redirects=False) as client:
        r = client.get("/lotto-quant")
    assert r.status_code in (302, 307)
    # Trailing slash stripped
    assert r.headers["location"] == "https://lotto.example.com:9000"
