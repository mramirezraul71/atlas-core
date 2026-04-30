"""Conectividad visual / arranque: PUSH :8791 saludable cuando el stack está levantado.

En CI o sin servicios locales, el test se omite explícitamente (no indica regresión).
"""
from __future__ import annotations

import os
import urllib.error
import urllib.request

import pytest


def _http_get_status(url: str, *, timeout: float) -> int | None:
    try:
        with urllib.request.urlopen(url, timeout=timeout) as r:  # noqa: S310
            return int(getattr(r, "status", 200) or 200)
    except (urllib.error.URLError, OSError, TimeoutError, ValueError):
        return None


@pytest.mark.skipif(
    os.getenv("ATLAS_SKIP_LIVE_SERVICE_TESTS", "").lower() in ("1", "true", "yes"),
    reason="ATLAS_SKIP_LIVE_SERVICE_TESTS: no comprobar servicios en vivo",
)
def test_push_health_reachable_when_stack_running() -> None:
    """GET /health en 127.0.0.1:8791 debe ser 200 si ATLAS (PUSH) está en marcha."""
    url = "http://127.0.0.1:8791/health"
    st = _http_get_status(url, timeout=2.0)
    if st is None:
        pytest.skip("PUSH :8791 no alcanzable; arrancar el stack o definir ATLAS_SKIP_LIVE_SERVICE_TESTS=1 en CI")
    assert st == 200
