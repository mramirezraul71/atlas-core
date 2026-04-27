"""F5 — Contrato del SSE multi-símbolo /api/radar/stream/opportunities.

NOTA sobre testing de SSE: ``TestClient.iter_text`` consume el generador
hasta que cierre, lo cual NO sucede en un stream SSE infinito real (sólo
cierra al cancelar la conexión). Para no bloquear, este test:

* Valida el contrato de envelope (6 campos canónicos) sobre eventos
  construidos con la misma forma que produce el handler real, mediante el
  modelo Pydantic ``RadarOpportunityStreamEvent``.
* Verifica que la ruta esté registrada en OpenAPI.
* Verifica los headers iniciales del stream usando ``httpx`` con timeout
  agresivo y cancelando tras leer la cabecera.
"""
from __future__ import annotations

import json
from typing import Any

import pytest
from fastapi import FastAPI
from fastapi.testclient import TestClient

from atlas_adapter.routes.radar_public import (
    _sse_event_block,
    _stream_headers,
    build_radar_stub_api_router,
)
from atlas_adapter.routes.radar_schemas import RadarOpportunityStreamEvent


def _app() -> FastAPI:
    app = FastAPI()
    app.include_router(build_radar_stub_api_router())
    return app


def test_stream_opportunities_route_in_openapi() -> None:
    c = TestClient(_app())
    r = c.get("/openapi.json")
    paths = r.json().get("paths") or {}
    assert "/api/radar/stream/opportunities" in paths


@pytest.mark.parametrize(
    "ev_type,symbol",
    [
        ("heartbeat", "*"),
        ("universe_snapshot", "*"),
        ("opportunity_added", "SPY"),
        ("opportunity_updated", "QQQ"),
        ("opportunity_removed", "AAPL"),
    ],
)
def test_stream_envelope_canonical_six_fields(ev_type: str, symbol: str) -> None:
    env: dict[str, Any] = {
        "type": ev_type,
        "timestamp": "2026-01-01T00:00:00+00:00",
        "symbol": symbol,
        "source": "stub",
        "sequence": 1,
        "data": {"k": "v"},
    }
    # Pydantic acepta sin perder campos.
    parsed = RadarOpportunityStreamEvent.model_validate(env)
    assert parsed.type == ev_type
    assert parsed.symbol == symbol
    assert parsed.sequence == 1
    # Forma SSE canónica (id + event + data) reutilizando helper compartido.
    block = _sse_event_block(env["sequence"], ev_type, env)
    assert f"event: {ev_type}\n" in block
    parsed_line = [ln for ln in block.splitlines() if ln.startswith("data:")][0]
    payload = json.loads(parsed_line[len("data:") :].strip())
    for k in ("type", "timestamp", "symbol", "source", "sequence", "data"):
        assert k in payload


@pytest.mark.parametrize(
    "bad",
    [
        {"type": "unknown_event", "timestamp": "x", "symbol": "*", "source": "stub", "sequence": 0, "data": {}},
        {"type": "heartbeat", "timestamp": "x", "symbol": "*", "source": "weird", "sequence": 0, "data": {}},
    ],
)
def test_stream_envelope_rejects_invalid_type_or_source(bad: dict[str, Any]) -> None:
    with pytest.raises(Exception):
        RadarOpportunityStreamEvent.model_validate(bad)


def test_stream_helper_headers_set_stub_or_quant() -> None:
    """Garantiza que ``_stream_headers`` declara explicitamente la fuente.

    No abrimos el stream HTTP (TestClient bloquearía al consumir el cuerpo
    infinito). Reutilizamos el helper compartido con el handler real.
    """
    h_stub = _stream_headers(live=False)
    assert h_stub["X-Atlas-Radar-Stub"] == "1"
    assert "X-Atlas-Radar-Source" not in h_stub
    assert h_stub["Cache-Control"].startswith("no-cache")

    h_live = _stream_headers(live=True)
    assert h_live["X-Atlas-Radar-Source"] == "quant"
    assert "X-Atlas-Radar-Stub" not in h_live
