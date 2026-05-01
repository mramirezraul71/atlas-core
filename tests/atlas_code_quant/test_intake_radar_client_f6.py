"""Tests F6 — Atlas Code Quant Radar intake (shadow only).

Cubre:
    * Parseo de un payload Radar válido → batch interno.
    * Mapeo desde modelos Pydantic v2 **locales** (misma forma que el Radar;
      sin importar ``atlas_adapter`` para evitar path/env cruzados).
    * Item malformado dentro del batch no rompe el resto.
    * Filtros: ``filter_by_min_score``, ``filter_by_asset_class``.
    * Cliente:
        - Timeout simulado → ``RADAR_TIMEOUT`` en intake_degradations.
        - Error de conexión → ``RADAR_UNREACHABLE``.
        - HTTP 500 → ``RADAR_HTTP_ERROR``.
        - JSON malformado → ``RADAR_INVALID_PAYLOAD``.
        - 404 en ``fetch_opportunity`` → ``None`` (no error).
    * Logger ``atlas.code_quant.intake.radar`` no rompe nada.
    * Función shadow ``fetch_opportunities_snapshot`` retorna batch interno.

NO se levanta la app FastAPI completa: usamos ``httpx.MockTransport`` para
simular el Radar.
"""

from __future__ import annotations

import json
import logging
from typing import Any, Literal

import httpx
import pytest
from pydantic import BaseModel, ConfigDict, Field

from atlas_code_quant.intake.opportunity import (
    RadarIntakeDegradation,
    RadarOpportunityBatchInternal,
    RadarOpportunityInternal,
    batch_from_radar_payload,
    from_radar_payload,
)
from atlas_code_quant.intake.radar_client import (
    DEFAULT_BASE_URL,
    DEFAULT_TIMEOUT_MS,
    INTAKE_CODE_HTTP_ERROR,
    INTAKE_CODE_INVALID_PAYLOAD,
    INTAKE_CODE_TIMEOUT,
    INTAKE_CODE_UNREACHABLE,
    RadarClient,
    RadarClientConfig,
    fetch_opportunities_snapshot,
)


# --- Modelos Pydantic locales (forma compatible con Radar / intake) ----------
# No importar atlas_adapter: en máquinas con varios worktrees en PYTHONPATH
# el paquete resuelto puede no coincidir con el árbol del repo bajo prueba.


class RadarOpportunitySnapshot(BaseModel):
    """Snapshot anidado (subset estable para tests)."""

    model_config = ConfigDict(extra="allow")

    timestamp: str = ""
    snapshot_classification: str = ""
    fast_pressure_score: float = 0.0


class RadarOpportunity(BaseModel):
    model_config = ConfigDict(extra="allow")

    symbol: str
    asset_class: str
    sector: str | None = None
    optionable: bool = True
    score: float = 0.0
    classification: str = ""
    direction: str = "neutral"
    timestamp: str = ""
    horizon_min: int | None = None
    snapshot: dict[str, Any] | RadarOpportunitySnapshot = Field(default_factory=dict)
    degradations_active: list[dict[str, Any]] = Field(default_factory=list)
    source: Literal["quant", "stub"] = "quant"
    trace_id: str = ""


class RadarOpportunitiesResponse(BaseModel):
    """Envelope batch: ``extra=allow`` conserva ``items`` y metadatos del fixture."""

    model_config = ConfigDict(extra="allow")


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------


@pytest.fixture()
def valid_payload() -> dict[str, Any]:
    return {
        "ok": True,
        "source": "quant",
        "timestamp": "2026-04-27T12:00:00+00:00",
        "universe_size": 100,
        "evaluated": 50,
        "succeeded": 30,
        "failed": 20,
        "returned": 2,
        "limit": 10,
        "min_score": 50.0,
        "filters": {"asset_class": "equity"},
        "items": [
            {
                "symbol": "AAPL",
                "asset_class": "equity",
                "sector": "tech",
                "optionable": True,
                "score": 78.5,
                "classification": "high_conviction",
                "direction": "long",
                "timestamp": "2026-04-27T12:00:00+00:00",
                "horizon_min": 60,
                "snapshot": {
                    "timestamp": "2026-04-27T12:00:00+00:00",
                    "snapshot_classification": "bullish",
                    "fast_pressure_score": 1.2,
                },
                "degradations_active": [
                    {
                        "code": "STALE_QUOTE",
                        "label": "stale quote",
                        "severity": "info",
                        "source": "tradier",
                    }
                ],
                "source": "quant",
                "trace_id": "trace-aapl-001",
            },
            {
                "symbol": "spy",
                "asset_class": "ETF",
                "sector": None,
                "optionable": True,
                "score": 55.0,
                "classification": "watchlist",
                "direction": "neutral",
                "timestamp": "2026-04-27T12:00:00+00:00",
                "horizon_min": None,
                "snapshot": {
                    "timestamp": "2026-04-27T12:00:00+00:00",
                    "snapshot_classification": "neutral",
                },
                "degradations_active": [],
                "source": "quant",
                "trace_id": "trace-spy-002",
            },
        ],
        "degradations_active": [],
        "trace_id": "trace-batch-xyz",
    }


def _make_client(handler) -> RadarClient:
    transport = httpx.MockTransport(handler)
    cfg = RadarClientConfig(base_url="http://radar.test", timeout_ms=2000)
    return RadarClient(cfg, transport=transport)


# ---------------------------------------------------------------------------
# Modelos internos: parseo directo
# ---------------------------------------------------------------------------


def test_batch_from_dict_payload_preserves_core_fields(valid_payload):
    batch = batch_from_radar_payload(valid_payload)

    assert isinstance(batch, RadarOpportunityBatchInternal)
    assert batch.ok is True
    assert batch.source == "quant"
    assert batch.trace_id == "trace-batch-xyz"
    assert batch.universe_size == 100
    assert batch.evaluated == 50
    assert batch.returned == 2
    assert len(batch.items) == 2
    assert batch.intake_degradations == ()  # Path feliz: sin fallo de intake.

    aapl = batch.items[0]
    assert isinstance(aapl, RadarOpportunityInternal)
    assert aapl.symbol == "AAPL"
    assert aapl.asset_class == "equity"
    assert aapl.score == pytest.approx(78.5)
    assert aapl.classification == "high_conviction"
    assert aapl.direction == "long"
    assert aapl.horizon_min == 60
    assert aapl.trace_id == "trace-aapl-001"
    assert aapl.is_high_conviction is True
    assert aapl.is_equity is True
    assert aapl.is_live is True
    assert aapl.snapshot.get("fast_pressure_score") == pytest.approx(1.2)
    assert len(aapl.degradations_active) == 1
    assert aapl.degradations_active[0].code == "STALE_QUOTE"
    assert aapl.degradations_active[0].severity == "info"

    spy = batch.items[1]
    assert spy.symbol == "SPY"  # normalizado a uppercase
    assert spy.asset_class == "etf"  # normalizado a lowercase
    assert spy.is_etf is True
    assert spy.classification == "watchlist"


def test_mapping_from_pydantic_model_preserves_trace_and_degradations(valid_payload):
    """``batch_from_radar_payload`` debe aceptar instancias Pydantic v2 reales."""
    pydantic_model = RadarOpportunitiesResponse.model_validate(valid_payload)
    batch = batch_from_radar_payload(pydantic_model)

    assert batch.trace_id == "trace-batch-xyz"
    assert batch.source == "quant"
    assert batch.returned == 2
    assert batch.items[0].trace_id == "trace-aapl-001"
    assert batch.items[0].score == pytest.approx(78.5)


def test_from_payload_accepts_pydantic_opportunity():
    snap = RadarOpportunitySnapshot(
        timestamp="2026-04-27T12:00:00+00:00",
        snapshot_classification="bullish",
        fast_pressure_score=2.0,
    )
    opp = RadarOpportunity(
        symbol="MSFT",
        asset_class="equity",
        sector="tech",
        optionable=True,
        score=82.0,
        classification="high_conviction",
        direction="long",
        timestamp="2026-04-27T12:00:00+00:00",
        horizon_min=120,
        snapshot=snap,
        degradations_active=[],
        source="quant",
        trace_id="trace-msft-001",
    )
    internal = from_radar_payload(opp)
    assert internal.symbol == "MSFT"
    assert internal.score == pytest.approx(82.0)
    assert internal.is_high_conviction is True
    assert internal.trace_id == "trace-msft-001"


def test_malformed_item_inside_batch_does_not_break_others(valid_payload):
    payload = dict(valid_payload)
    items = list(payload["items"])
    # Inserta un ítem inválido (no es dict): no debe abortar el batch.
    items.insert(1, "not-an-object")
    payload["items"] = items
    batch = batch_from_radar_payload(payload)
    # El string se convierte en dict vacío vía _to_dict ⇒ se mapea a un
    # opportunity *vacío* pero no rompe; comprobamos que los 2 originales
    # sobreviven más cualquier extra defensivo.
    symbols = {opp.symbol for opp in batch.items}
    assert "AAPL" in symbols
    assert "SPY" in symbols


def test_filter_helpers_work(valid_payload):
    batch = batch_from_radar_payload(valid_payload)
    high = batch.filter_by_min_score(70.0)
    assert len(high) == 1
    assert high[0].symbol == "AAPL"
    etfs = batch.filter_by_asset_class("etf")
    assert len(etfs) == 1
    assert etfs[0].symbol == "SPY"


def test_empty_with_degradation_factory():
    batch = RadarOpportunityBatchInternal.empty_with_degradation(
        code="X", label="y", severity="critical", source="src", trace_id="tid"
    )
    assert batch.ok is False
    assert batch.items == ()
    assert batch.has_intake_failure is True
    assert batch.intake_degradations[0].code == "X"
    assert batch.intake_degradations[0].severity == "critical"
    assert batch.trace_id == "tid"


# ---------------------------------------------------------------------------
# Cliente: configuración
# ---------------------------------------------------------------------------


def test_config_defaults_when_env_missing(monkeypatch):
    monkeypatch.delenv("ATLAS_RADAR_BASE_URL", raising=False)
    monkeypatch.delenv("ATLAS_RADAR_TIMEOUT_MS", raising=False)
    cfg = RadarClientConfig()
    assert cfg.base_url == DEFAULT_BASE_URL
    assert cfg.timeout_seconds == pytest.approx(DEFAULT_TIMEOUT_MS / 1000.0)


def test_config_reads_env(monkeypatch):
    monkeypatch.setenv("ATLAS_RADAR_BASE_URL", "http://radar.example/")
    monkeypatch.setenv("ATLAS_RADAR_TIMEOUT_MS", "1500")
    cfg = RadarClientConfig()
    assert cfg.base_url == "http://radar.example"  # trailing slash recortado
    assert cfg.timeout_seconds == pytest.approx(1.5)


def test_config_invalid_env_falls_back_to_defaults(monkeypatch):
    monkeypatch.setenv("ATLAS_RADAR_TIMEOUT_MS", "not-a-number")
    cfg = RadarClientConfig()
    assert cfg.timeout_seconds == pytest.approx(DEFAULT_TIMEOUT_MS / 1000.0)
    monkeypatch.setenv("ATLAS_RADAR_TIMEOUT_MS", "-100")
    cfg2 = RadarClientConfig()
    assert cfg2.timeout_seconds == pytest.approx(DEFAULT_TIMEOUT_MS / 1000.0)


# ---------------------------------------------------------------------------
# Cliente: path feliz
# ---------------------------------------------------------------------------


def test_fetch_opportunities_happy_path(valid_payload):
    captured: dict[str, Any] = {}

    def handler(request: httpx.Request) -> httpx.Response:
        captured["url"] = str(request.url)
        captured["params"] = dict(request.url.params)
        return httpx.Response(200, json=valid_payload)

    client = _make_client(handler)
    batch = client.fetch_opportunities({"min_score": 50, "asset_class": "equity"})

    assert "/api/radar/opportunities" in captured["url"]
    assert captured["params"]["min_score"] == "50"
    assert captured["params"]["asset_class"] == "equity"

    assert batch.intake_degradations == ()
    assert batch.returned == 2
    assert {opp.symbol for opp in batch.items} == {"AAPL", "SPY"}


def test_fetch_opportunity_happy_path(valid_payload):
    aapl_payload = valid_payload["items"][0]

    def handler(request: httpx.Request) -> httpx.Response:
        assert request.url.path == "/api/radar/opportunities/AAPL"
        return httpx.Response(200, json=aapl_payload)

    client = _make_client(handler)
    opp = client.fetch_opportunity("aapl")
    assert opp is not None
    assert opp.symbol == "AAPL"
    assert opp.is_high_conviction is True


# ---------------------------------------------------------------------------
# Cliente: degradaciones (timeout, conexión, 5xx, JSON inválido)
# ---------------------------------------------------------------------------


def test_fetch_opportunities_timeout_yields_intake_degradation():
    def handler(request: httpx.Request) -> httpx.Response:
        raise httpx.ReadTimeout("simulated timeout", request=request)

    client = _make_client(handler)
    batch = client.fetch_opportunities()

    assert batch.has_intake_failure is True
    codes = [d.code for d in batch.intake_degradations]
    assert INTAKE_CODE_TIMEOUT in codes
    assert batch.items == ()


def test_fetch_opportunities_connect_error_yields_unreachable():
    def handler(request: httpx.Request) -> httpx.Response:
        raise httpx.ConnectError("simulated unreachable", request=request)

    client = _make_client(handler)
    batch = client.fetch_opportunities()

    codes = [d.code for d in batch.intake_degradations]
    assert INTAKE_CODE_UNREACHABLE in codes


def test_fetch_opportunities_http_5xx_yields_http_error():
    def handler(request: httpx.Request) -> httpx.Response:
        return httpx.Response(503, json={"detail": "service unavailable"})

    client = _make_client(handler)
    batch = client.fetch_opportunities()

    codes = [d.code for d in batch.intake_degradations]
    assert INTAKE_CODE_HTTP_ERROR in codes
    assert batch.intake_degradations[0].severity == "critical"


def test_fetch_opportunities_invalid_json_yields_invalid_payload():
    def handler(request: httpx.Request) -> httpx.Response:
        return httpx.Response(
            200,
            content=b"not-json{{",
            headers={"content-type": "application/json"},
        )

    client = _make_client(handler)
    batch = client.fetch_opportunities()

    codes = [d.code for d in batch.intake_degradations]
    assert INTAKE_CODE_INVALID_PAYLOAD in codes


def test_fetch_opportunities_non_dict_payload_yields_invalid_payload():
    def handler(request: httpx.Request) -> httpx.Response:
        return httpx.Response(200, json=[1, 2, 3])  # JSON pero no objeto

    client = _make_client(handler)
    batch = client.fetch_opportunities()

    codes = [d.code for d in batch.intake_degradations]
    assert INTAKE_CODE_INVALID_PAYLOAD in codes


def test_fetch_opportunity_404_returns_none():
    def handler(request: httpx.Request) -> httpx.Response:
        return httpx.Response(404, json={"detail": "not found"})

    client = _make_client(handler)
    assert client.fetch_opportunity("ZZZZ") is None


def test_fetch_opportunity_5xx_returns_none():
    def handler(request: httpx.Request) -> httpx.Response:
        return httpx.Response(502, json={"detail": "bad gateway"})

    client = _make_client(handler)
    assert client.fetch_opportunity("AAPL") is None


def test_fetch_opportunity_empty_symbol_short_circuits():
    calls = {"n": 0}

    def handler(request: httpx.Request) -> httpx.Response:  # pragma: no cover
        calls["n"] += 1
        return httpx.Response(200, json={})

    client = _make_client(handler)
    assert client.fetch_opportunity("") is None
    assert client.fetch_opportunity("   ") is None
    assert calls["n"] == 0  # no se ejecuta ninguna request


# ---------------------------------------------------------------------------
# Shadow API
# ---------------------------------------------------------------------------


def test_fetch_opportunities_snapshot_uses_provided_client(valid_payload):
    def handler(request: httpx.Request) -> httpx.Response:
        return httpx.Response(200, json=valid_payload)

    client = _make_client(handler)
    batch = fetch_opportunities_snapshot({"min_score": 50}, client=client)
    assert batch.returned == 2
    assert batch.intake_degradations == ()


def test_fetch_opportunities_snapshot_returns_batch_on_failure():
    def handler(request: httpx.Request) -> httpx.Response:
        raise httpx.ReadTimeout("boom", request=request)

    client = _make_client(handler)
    batch = fetch_opportunities_snapshot(None, client=client)
    assert isinstance(batch, RadarOpportunityBatchInternal)
    assert batch.has_intake_failure is True


# ---------------------------------------------------------------------------
# Logger
# ---------------------------------------------------------------------------


def test_logger_does_not_break_on_failure(caplog):
    def handler(request: httpx.Request) -> httpx.Response:
        return httpx.Response(503)

    client = _make_client(handler)
    with caplog.at_level(logging.WARNING, logger="atlas.code_quant.intake.radar"):
        batch = client.fetch_opportunities()

    assert batch.has_intake_failure is True
    # Al menos un mensaje WARNING en el logger esperado.
    matching = [
        r
        for r in caplog.records
        if r.name == "atlas.code_quant.intake.radar" and r.levelno >= logging.WARNING
    ]
    assert matching, "esperaba al menos un WARNING en logger intake.radar"
