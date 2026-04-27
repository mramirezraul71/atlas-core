"""Tests F7 — filtro shadow scanner→Radar.

Cubre:
    * Path feliz: 3 símbolos, mezcla high_conviction / watchlist / reject
      por score bajo.
    * Radar sin datos para algunos símbolos → ``no_data``.
    * Degradaciones a nivel de batch (TIMEOUT, UNREACHABLE, HTTP_ERROR,
      INVALID_PAYLOAD): todos los candidatos van a ``rejected_by_radar``
      con classification ``no_data`` y ``radar_degradations`` poblado.
    * Acepta entradas dict y dataclass-like.
    * Símbolos vacíos / duplicados se descartan.
    * NUNCA lanza excepción.
    * El cliente Radar recibe el filtro ``symbols=[...]`` derivado del
      universo del scanner.
    * Default ``min_score=70`` se respeta (item con score 60 + classification
      watchlist termina en reject).

NO se levanta FastAPI completa: usamos ``httpx.MockTransport`` a través
del ``RadarClient`` de F6.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import httpx
import pytest

from atlas_code_quant.intake.opportunity import (
    RadarOpportunityBatchInternal,
)
from atlas_code_quant.intake.radar_client import (
    INTAKE_CODE_HTTP_ERROR,
    INTAKE_CODE_INVALID_PAYLOAD,
    INTAKE_CODE_TIMEOUT,
    INTAKE_CODE_UNREACHABLE,
    RadarClient,
    RadarClientConfig,
)
from atlas_code_quant.intake.scanner_radar_filter import (
    DEFAULT_MIN_SCORE,
    INTAKE_CODE_NO_DATA,
    RadarFilteredCandidate,
    ScannerCandidateLike,
    ScannerRadarFilterResult,
    filter_scanner_candidates_with_radar,
)


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------


def _opp_payload(
    symbol: str,
    *,
    score: float,
    classification: str,
    asset_class: str = "equity",
    direction: str = "long",
    trace_id: str | None = None,
) -> dict[str, Any]:
    return {
        "symbol": symbol,
        "asset_class": asset_class,
        "sector": "tech",
        "optionable": True,
        "score": score,
        "classification": classification,
        "direction": direction,
        "timestamp": "2026-04-27T12:00:00+00:00",
        "horizon_min": 60,
        "snapshot": {
            "timestamp": "2026-04-27T12:00:00+00:00",
            "snapshot_classification": "bullish",
        },
        "degradations_active": [],
        "source": "quant",
        "trace_id": trace_id or f"trace-{symbol.lower()}",
    }


def _batch_payload(items: list[dict[str, Any]]) -> dict[str, Any]:
    return {
        "ok": True,
        "source": "quant",
        "timestamp": "2026-04-27T12:00:00+00:00",
        "universe_size": 100,
        "evaluated": 50,
        "succeeded": 30,
        "failed": 20,
        "returned": len(items),
        "limit": 50,
        "min_score": 0.0,
        "filters": {},
        "items": items,
        "degradations_active": [],
        "trace_id": "trace-batch-f7",
    }


def _make_client(handler) -> RadarClient:
    transport = httpx.MockTransport(handler)
    cfg = RadarClientConfig(base_url="http://radar.test", timeout_ms=2000)
    return RadarClient(cfg, transport=transport)


# ---------------------------------------------------------------------------
# Estructuras
# ---------------------------------------------------------------------------


def test_scanner_candidate_from_dict_and_attrs():
    d = {"symbol": "aapl", "asset_class": "Equity", "score": 55}
    a = ScannerCandidateLike.from_any(d)
    assert a.symbol == "AAPL"
    assert a.asset_class == "equity"
    assert a.score == pytest.approx(55.0)

    @dataclass
    class _Cand:
        symbol: str
        asset_class: str
        score: float

    c = ScannerCandidateLike.from_any(_Cand("MSFT", "equity", 80.0))
    assert c.symbol == "MSFT"
    assert c.score == pytest.approx(80.0)


def test_scanner_candidate_handles_missing_fields():
    a = ScannerCandidateLike.from_any({})
    assert a.symbol == ""
    assert a.asset_class == "unknown"
    assert a.score == 0.0


# ---------------------------------------------------------------------------
# Filtro: path feliz
# ---------------------------------------------------------------------------


def test_filter_happy_path_classifies_three_symbols():
    """AAPL high_conviction (≥70), SPY watchlist (≥70), MSFT score bajo (<70) → reject."""
    captured: dict[str, Any] = {}

    def handler(request: httpx.Request) -> httpx.Response:
        captured["params"] = dict(request.url.params)
        captured["path"] = request.url.path
        items = [
            _opp_payload("AAPL", score=85.0, classification="high_conviction"),
            _opp_payload("SPY", score=72.0, classification="watchlist", asset_class="etf"),
            _opp_payload("MSFT", score=40.0, classification="watchlist"),
        ]
        return httpx.Response(200, json=_batch_payload(items))

    client = _make_client(handler)
    candidates = [
        {"symbol": "AAPL", "asset_class": "equity", "score": 0.5},
        {"symbol": "SPY", "asset_class": "etf", "score": 0.4},
        {"symbol": "MSFT", "asset_class": "equity", "score": 0.6},
    ]

    result = filter_scanner_candidates_with_radar(candidates, client)

    assert isinstance(result, ScannerRadarFilterResult)
    assert captured["path"] == "/api/radar/opportunities"
    # El filtro debe pasar la lista de símbolos al Radar.
    assert "symbols" in captured["params"]

    approved_symbols = {c.symbol for c in result.approved_by_radar}
    rejected_symbols = {c.symbol for c in result.rejected_by_radar}
    assert approved_symbols == {"AAPL", "SPY"}
    assert rejected_symbols == {"MSFT"}

    # Classification correcta.
    by_sym = {c.symbol: c for c in result.all_candidates()}
    assert by_sym["AAPL"].classification == "high_conviction"
    assert by_sym["SPY"].classification == "watchlist"
    assert by_sym["MSFT"].classification == "reject"

    # Score preservado.
    assert by_sym["AAPL"].score == pytest.approx(85.0)
    assert by_sym["MSFT"].score == pytest.approx(40.0)

    # Sin degradaciones de batch.
    assert result.radar_degradations == ()
    assert result.has_batch_degradation is False
    assert result.metadata["batch_failed"] is False
    assert result.metadata["trace_id"] == "trace-batch-f7"
    assert result.metadata["scanner_input"] == 3
    assert result.metadata["approved_count"] == 2
    assert result.metadata["rejected_count"] == 1


def test_filter_respects_explicit_min_score():
    """Con min_score=80, sólo AAPL queda approved aunque SPY tiene watchlist."""

    def handler(request: httpx.Request) -> httpx.Response:
        items = [
            _opp_payload("AAPL", score=85.0, classification="high_conviction"),
            _opp_payload("SPY", score=72.0, classification="watchlist"),
        ]
        return httpx.Response(200, json=_batch_payload(items))

    client = _make_client(handler)
    candidates = [{"symbol": "AAPL"}, {"symbol": "SPY"}]
    result = filter_scanner_candidates_with_radar(
        candidates, client, min_score=80.0
    )
    approved = {c.symbol for c in result.approved_by_radar}
    rejected = {c.symbol for c in result.rejected_by_radar}
    assert approved == {"AAPL"}
    assert rejected == {"SPY"}
    assert {c.classification for c in result.rejected_by_radar} == {"reject"}


# ---------------------------------------------------------------------------
# Sin datos para algún símbolo
# ---------------------------------------------------------------------------


def test_filter_marks_no_data_when_radar_omits_symbol():
    """Scanner pide AAPL y NVDA; Radar sólo trae AAPL → NVDA = no_data."""

    def handler(request: httpx.Request) -> httpx.Response:
        items = [_opp_payload("AAPL", score=80.0, classification="high_conviction")]
        return httpx.Response(200, json=_batch_payload(items))

    client = _make_client(handler)
    result = filter_scanner_candidates_with_radar(
        [{"symbol": "AAPL"}, {"symbol": "NVDA"}], client
    )
    by_sym = {c.symbol: c for c in result.all_candidates()}
    assert by_sym["AAPL"].is_approved is True
    assert by_sym["NVDA"].is_rejected is True
    assert by_sym["NVDA"].classification == "no_data"
    assert by_sym["NVDA"].radar_opportunity is None
    # Tiene la degradación per-symbol RADAR_NO_DATA.
    assert any(
        d.code == INTAKE_CODE_NO_DATA for d in by_sym["NVDA"].degradations
    )
    # Pero el batch global no está degradado.
    assert result.radar_degradations == ()


# ---------------------------------------------------------------------------
# Degradaciones de batch (timeout, conexión, 5xx, JSON inválido)
# ---------------------------------------------------------------------------


@pytest.mark.parametrize(
    "exc_factory,expected_code",
    [
        (lambda req: httpx.ReadTimeout("timeout", request=req), INTAKE_CODE_TIMEOUT),
        (
            lambda req: httpx.ConnectError("unreachable", request=req),
            INTAKE_CODE_UNREACHABLE,
        ),
    ],
)
def test_filter_handles_transport_failures(exc_factory, expected_code):
    def handler(request: httpx.Request) -> httpx.Response:
        raise exc_factory(request)

    client = _make_client(handler)
    candidates = [{"symbol": "AAPL"}, {"symbol": "MSFT"}]
    result = filter_scanner_candidates_with_radar(candidates, client)

    codes = [d.code for d in result.radar_degradations]
    assert expected_code in codes
    # Todos los símbolos terminan en rejected con classification=no_data.
    assert result.approved_by_radar == ()
    rejected_syms = {c.symbol for c in result.rejected_by_radar}
    assert rejected_syms == {"AAPL", "MSFT"}
    assert all(
        c.classification == "no_data" for c in result.rejected_by_radar
    )
    assert result.metadata["batch_failed"] is True


def test_filter_handles_http_5xx():
    def handler(request: httpx.Request) -> httpx.Response:
        return httpx.Response(503)

    client = _make_client(handler)
    result = filter_scanner_candidates_with_radar([{"symbol": "AAPL"}], client)

    assert any(
        d.code == INTAKE_CODE_HTTP_ERROR for d in result.radar_degradations
    )
    assert result.approved_by_radar == ()
    assert len(result.rejected_by_radar) == 1


def test_filter_handles_invalid_payload():
    def handler(request: httpx.Request) -> httpx.Response:
        return httpx.Response(
            200, content=b"not-json", headers={"content-type": "application/json"}
        )

    client = _make_client(handler)
    result = filter_scanner_candidates_with_radar([{"symbol": "AAPL"}], client)

    assert any(
        d.code == INTAKE_CODE_INVALID_PAYLOAD for d in result.radar_degradations
    )
    assert result.approved_by_radar == ()


# ---------------------------------------------------------------------------
# Robustez: entradas raras
# ---------------------------------------------------------------------------


def test_filter_with_empty_input_returns_empty_result():
    # Sin candidatos no debe abrir socket.
    def handler(request: httpx.Request) -> httpx.Response:  # pragma: no cover
        raise AssertionError("no debería llamarse")

    client = _make_client(handler)
    result = filter_scanner_candidates_with_radar([], client)
    assert result.approved_by_radar == ()
    assert result.rejected_by_radar == ()
    assert result.radar_degradations == ()
    assert result.metadata["scanner_input"] == 0


def test_filter_dedupes_and_skips_invalid_symbols():
    captured: dict[str, Any] = {}

    def handler(request: httpx.Request) -> httpx.Response:
        captured["params"] = dict(request.url.params)
        items = [_opp_payload("AAPL", score=85.0, classification="high_conviction")]
        return httpx.Response(200, json=_batch_payload(items))

    client = _make_client(handler)
    result = filter_scanner_candidates_with_radar(
        [
            {"symbol": "AAPL"},
            {"symbol": "aapl"},  # duplicado tras normalización.
            {"symbol": ""},  # vacío.
            {"symbol": None},  # inválido.
            {},  # sin clave.
        ],
        client,
    )
    # Sólo AAPL llega al Radar.
    assert result.metadata["scanner_input"] == 1
    assert len(result.approved_by_radar) == 1
    assert result.approved_by_radar[0].symbol == "AAPL"


def test_filter_never_raises_on_unexpected_client_failure(monkeypatch):
    """Si el RadarClient lanza una excepción no controlada, el filtro
    igual devuelve un resultado degradado."""

    class _BoomClient:
        def fetch_opportunities(self, filters):  # noqa: D401, ARG002
            raise RuntimeError("boom inesperado")

    result = filter_scanner_candidates_with_radar(
        [{"symbol": "AAPL"}], _BoomClient()  # type: ignore[arg-type]
    )
    assert result.approved_by_radar == ()
    assert len(result.rejected_by_radar) == 1
    codes = [d.code for d in result.radar_degradations]
    assert INTAKE_CODE_UNREACHABLE in codes


# ---------------------------------------------------------------------------
# Default min_score
# ---------------------------------------------------------------------------


def test_default_min_score_is_70():
    """Smoke: el default documentado es 70."""
    assert DEFAULT_MIN_SCORE == 70.0


def test_filter_classification_reject_native_passthrough():
    """Si Radar devuelve classification=reject explícito, aunque el score
    supere min_score, debe quedar rejected (defensivo)."""

    def handler(request: httpx.Request) -> httpx.Response:
        items = [_opp_payload("AAPL", score=99.0, classification="reject")]
        return httpx.Response(200, json=_batch_payload(items))

    client = _make_client(handler)
    result = filter_scanner_candidates_with_radar([{"symbol": "AAPL"}], client)
    assert result.approved_by_radar == ()
    assert len(result.rejected_by_radar) == 1
    assert result.rejected_by_radar[0].classification == "reject"


# ---------------------------------------------------------------------------
# Confirmación: scanner sigue intacto
# ---------------------------------------------------------------------------


def test_filter_does_not_mutate_scanner_input():
    """El filtro no debe modificar la lista original ni los dicts."""
    cand_a = {"symbol": "AAPL", "asset_class": "equity", "score": 0.5}
    cand_b = {"symbol": "MSFT", "asset_class": "equity", "score": 0.6}
    original = [cand_a, cand_b]
    snapshot = [dict(cand_a), dict(cand_b)]

    def handler(request: httpx.Request) -> httpx.Response:
        items = [_opp_payload("AAPL", score=85.0, classification="high_conviction")]
        return httpx.Response(200, json=_batch_payload(items))

    client = _make_client(handler)
    filter_scanner_candidates_with_radar(original, client)

    assert original == snapshot  # nada mutado
    assert cand_a == snapshot[0]
    assert cand_b == snapshot[1]
