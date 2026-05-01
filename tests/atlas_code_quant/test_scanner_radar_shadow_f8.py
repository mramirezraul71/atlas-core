"""Tests F8 — consumer runtime-safe scanner→Radar shadow.

Cubre:
    * Path feliz: scanner propone N candidatos, Radar aprueba subset,
      reporte agregado correcto (counts, symbols, divergence_ratio).
    * Caso 0 candidatos: reporte vacío sin abrir socket.
    * Degradaciones del Radar: timeout / 5xx / payload inválido / connect
      error → reporte con ``degradations`` poblado y todos los candidatos
      contados como rechazados (no_data).
    * ``emit_logs=True`` produce eventos started/completed/degraded
      en el logger ``atlas.code_quant.shadow.scanner_radar`` con shape
      esperado.
    * ``emit_logs=False`` silencia logs.
    * Defense in depth: si el filtro F7 lanzara excepción inesperada
      (mock), el reporte sigue saliendo.
    * El consumer NO muta la entrada del scanner (confirmación de no-side
      effect operativo).
    * No hay hook activo: F8 no ejecuta automáticamente sobre nada de
      runtime — la flag ``ATLAS_RADAR_FILTER_SHADOW_RUNTIME_ENABLED``
      sigue ``False`` y no se consulta.

Sin app FastAPI completa: usamos ``httpx.MockTransport`` a través del
``RadarClient`` de F6.
"""

from __future__ import annotations

import logging
from typing import Any
from unittest.mock import patch

import httpx
import pytest

from atlas_code_quant.config import legacy_flags
from atlas_code_quant.intake.radar_client import (
    INTAKE_CODE_HTTP_ERROR,
    INTAKE_CODE_INVALID_PAYLOAD,
    INTAKE_CODE_TIMEOUT,
    INTAKE_CODE_UNREACHABLE,
    RadarClient,
    RadarClientConfig,
)
from atlas_code_quant.monitoring.scanner_radar_shadow import (
    MAX_LOGGED_SYMBOLS,
    ScannerRadarShadowReport,
    run_scanner_radar_shadow,
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
) -> dict[str, Any]:
    return {
        "symbol": symbol,
        "asset_class": asset_class,
        "sector": "tech",
        "optionable": True,
        "score": score,
        "classification": classification,
        "direction": "long",
        "timestamp": "2026-04-27T12:00:00+00:00",
        "horizon_min": 60,
        "snapshot": {
            "timestamp": "2026-04-27T12:00:00+00:00",
            "snapshot_classification": "bullish",
        },
        "degradations_active": [],
        "source": "quant",
        "trace_id": f"trace-{symbol.lower()}",
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
        "trace_id": "trace-batch-f8",
    }


def _make_client(handler) -> RadarClient:
    transport = httpx.MockTransport(handler)
    cfg = RadarClientConfig(base_url="http://radar.test", timeout_ms=2000)
    return RadarClient(cfg, transport=transport)


# ---------------------------------------------------------------------------
# Path feliz
# ---------------------------------------------------------------------------


def test_shadow_happy_path_aggregates_correctly():
    def handler(request: httpx.Request) -> httpx.Response:
        items = [
            _opp_payload("AAPL", score=85.0, classification="high_conviction"),
            _opp_payload("SPY", score=72.0, classification="watchlist", asset_class="etf"),
            _opp_payload("MSFT", score=40.0, classification="watchlist"),
        ]
        return httpx.Response(200, json=_batch_payload(items))

    client = _make_client(handler)
    candidates = [
        {"symbol": "AAPL"},
        {"symbol": "SPY"},
        {"symbol": "MSFT"},
    ]
    report = run_scanner_radar_shadow(
        candidates, radar_client=client, emit_logs=False
    )

    assert isinstance(report, ScannerRadarShadowReport)
    assert report.scanner_candidate_count == 3
    assert report.radar_approved_count == 2
    assert report.radar_rejected_count == 1
    assert set(report.approved_symbols) == {"AAPL", "SPY"}
    assert report.rejected_symbols == ("MSFT",)
    assert report.degradations == ()
    assert report.divergence_ratio == pytest.approx(1 / 3)
    assert report.metadata.get("trace_id") == "trace-batch-f8"
    assert report.metadata.get("batch_failed") is False


def test_shadow_empty_input_returns_empty_report():
    calls = {"n": 0}

    def handler(request: httpx.Request) -> httpx.Response:  # pragma: no cover
        calls["n"] += 1
        return httpx.Response(200, json=_batch_payload([]))

    client = _make_client(handler)
    report = run_scanner_radar_shadow([], radar_client=client, emit_logs=False)

    assert report.scanner_candidate_count == 0
    assert report.radar_approved_count == 0
    assert report.radar_rejected_count == 0
    assert report.divergence_ratio == 0.0
    assert report.degradations == ()
    assert calls["n"] == 0  # sin entrada, no se toca el Radar


def test_shadow_none_input_treated_as_empty():
    report = run_scanner_radar_shadow(None, emit_logs=False)  # type: ignore[arg-type]
    assert report.scanner_candidate_count == 0
    assert report.degradations == ()


# ---------------------------------------------------------------------------
# Degradaciones
# ---------------------------------------------------------------------------


@pytest.mark.parametrize(
    "exc_factory,expected_code",
    [
        (lambda req: httpx.ReadTimeout("t", request=req), INTAKE_CODE_TIMEOUT),
        (lambda req: httpx.ConnectError("u", request=req), INTAKE_CODE_UNREACHABLE),
    ],
)
def test_shadow_handles_transport_failures(exc_factory, expected_code):
    def handler(request: httpx.Request) -> httpx.Response:
        raise exc_factory(request)

    client = _make_client(handler)
    candidates = [{"symbol": "AAPL"}, {"symbol": "MSFT"}]
    report = run_scanner_radar_shadow(
        candidates, radar_client=client, emit_logs=False
    )

    # Scanner count NUNCA se altera por degradación: sigue siendo 2.
    assert report.scanner_candidate_count == 2
    assert report.radar_approved_count == 0
    assert report.radar_rejected_count == 2
    assert any(d.code == expected_code for d in report.degradations)
    assert report.has_batch_degradation is True
    assert report.divergence_ratio == 1.0


def test_shadow_handles_http_5xx():
    def handler(request: httpx.Request) -> httpx.Response:
        return httpx.Response(503)

    client = _make_client(handler)
    report = run_scanner_radar_shadow(
        [{"symbol": "AAPL"}], radar_client=client, emit_logs=False
    )
    assert report.scanner_candidate_count == 1
    assert any(d.code == INTAKE_CODE_HTTP_ERROR for d in report.degradations)


def test_shadow_handles_invalid_payload():
    def handler(request: httpx.Request) -> httpx.Response:
        return httpx.Response(
            200, content=b"not-json", headers={"content-type": "application/json"}
        )

    client = _make_client(handler)
    report = run_scanner_radar_shadow(
        [{"symbol": "AAPL"}], radar_client=client, emit_logs=False
    )
    assert any(
        d.code == INTAKE_CODE_INVALID_PAYLOAD for d in report.degradations
    )


# ---------------------------------------------------------------------------
# Logging estructurado
# ---------------------------------------------------------------------------


def test_shadow_emits_started_and_completed_when_emit_logs_true(caplog):
    def handler(request: httpx.Request) -> httpx.Response:
        items = [_opp_payload("AAPL", score=85.0, classification="high_conviction")]
        return httpx.Response(200, json=_batch_payload(items))

    client = _make_client(handler)
    with caplog.at_level(
        logging.INFO, logger="atlas.code_quant.shadow.scanner_radar"
    ):
        run_scanner_radar_shadow(
            [{"symbol": "AAPL"}], radar_client=client, emit_logs=True
        )

    events = [
        getattr(r, "event", None)
        for r in caplog.records
        if r.name == "atlas.code_quant.shadow.scanner_radar"
    ]
    assert "scanner_radar_shadow_started" in events
    assert "scanner_radar_shadow_completed" in events
    # 'degraded' NO debería emitirse en path feliz.
    assert "scanner_radar_shadow_degraded" not in events


def test_shadow_emits_degraded_event_on_failure(caplog):
    def handler(request: httpx.Request) -> httpx.Response:
        return httpx.Response(503)

    client = _make_client(handler)
    with caplog.at_level(
        logging.INFO, logger="atlas.code_quant.shadow.scanner_radar"
    ):
        run_scanner_radar_shadow(
            [{"symbol": "AAPL"}], radar_client=client, emit_logs=True
        )

    events = [
        getattr(r, "event", None)
        for r in caplog.records
        if r.name == "atlas.code_quant.shadow.scanner_radar"
    ]
    assert "scanner_radar_shadow_degraded" in events


def test_shadow_emit_logs_false_silences_logger(caplog):
    def handler(request: httpx.Request) -> httpx.Response:
        items = [_opp_payload("AAPL", score=85.0, classification="high_conviction")]
        return httpx.Response(200, json=_batch_payload(items))

    client = _make_client(handler)
    with caplog.at_level(
        logging.DEBUG, logger="atlas.code_quant.shadow.scanner_radar"
    ):
        run_scanner_radar_shadow(
            [{"symbol": "AAPL"}], radar_client=client, emit_logs=False
        )

    # Sin eventos del logger F8.
    events = [
        getattr(r, "event", None)
        for r in caplog.records
        if r.name == "atlas.code_quant.shadow.scanner_radar"
    ]
    assert events == []


def test_shadow_truncates_large_symbol_lists_in_logs(caplog):
    big = MAX_LOGGED_SYMBOLS + 5
    items = [
        _opp_payload(f"S{i:03d}", score=85.0, classification="high_conviction")
        for i in range(big)
    ]

    def handler(request: httpx.Request) -> httpx.Response:
        return httpx.Response(200, json=_batch_payload(items))

    client = _make_client(handler)
    candidates = [{"symbol": f"S{i:03d}"} for i in range(big)]
    with caplog.at_level(
        logging.INFO, logger="atlas.code_quant.shadow.scanner_radar"
    ):
        run_scanner_radar_shadow(
            candidates, radar_client=client, emit_logs=True
        )

    completed = [
        r
        for r in caplog.records
        if getattr(r, "event", None) == "scanner_radar_shadow_completed"
    ]
    assert completed
    approved_logged = getattr(completed[0], "approved_symbols", [])
    # Debe estar truncado al límite + marcador de "+N".
    assert len(approved_logged) == MAX_LOGGED_SYMBOLS + 1
    assert any(
        isinstance(s, str) and s.startswith("…(+") for s in approved_logged
    )


# ---------------------------------------------------------------------------
# Defense in depth: filtro F7 lanza excepción
# ---------------------------------------------------------------------------


def test_shadow_survives_filter_f7_unexpected_exception():
    target = (
        "atlas_code_quant.monitoring.scanner_radar_shadow."
        "filter_scanner_candidates_with_radar"
    )
    with patch(target, side_effect=RuntimeError("boom inesperado")):
        report = run_scanner_radar_shadow(
            [{"symbol": "AAPL"}], emit_logs=False
        )

    assert report.scanner_candidate_count == 1
    assert report.radar_approved_count == 0
    assert report.has_batch_degradation is True
    codes = [d.code for d in report.degradations]
    assert "RADAR_UNREACHABLE" in codes


# ---------------------------------------------------------------------------
# No-side-effect operativo
# ---------------------------------------------------------------------------


def test_shadow_does_not_mutate_scanner_input():
    cand_a = {"symbol": "AAPL", "asset_class": "equity", "score": 0.5}
    cand_b = {"symbol": "MSFT", "asset_class": "equity", "score": 0.6}
    original = [cand_a, cand_b]
    snapshot = [dict(cand_a), dict(cand_b)]

    def handler(request: httpx.Request) -> httpx.Response:
        items = [_opp_payload("AAPL", score=85.0, classification="high_conviction")]
        return httpx.Response(200, json=_batch_payload(items))

    client = _make_client(handler)
    run_scanner_radar_shadow(original, radar_client=client, emit_logs=False)

    assert original == snapshot
    assert cand_a == snapshot[0]
    assert cand_b == snapshot[1]


# ---------------------------------------------------------------------------
# Confirmación de wiring: no se invoca desde runtime / hook ausente
# ---------------------------------------------------------------------------


def test_runtime_flag_default_is_false():
    assert legacy_flags.ATLAS_RADAR_FILTER_SHADOW_RUNTIME_ENABLED is False


def test_no_runtime_module_consults_runtime_flag():
    """F8 NO debe consultar la flag en runtime: ningún módulo F8 la lee
    como condicional. Verificación estática barata."""
    import atlas_code_quant.monitoring.scanner_radar_shadow as mod
    src = open(mod.__file__, encoding="utf-8").read()
    assert "ATLAS_RADAR_FILTER_SHADOW_RUNTIME_ENABLED" not in src


def test_no_module_under_scanner_or_strategies_imports_shadow_consumer():
    """Confirmación: el shadow consumer NO está enganchado a scanner ni
    strategies en F8. Si en alguna fase futura se introduce un import,
    este test falla y obliga a documentarlo en el doc de cutover.

    Lectura textual pura sobre archivos .py: NO importa los paquetes
    (algunos tienen errores de entorno preexistentes).
    """
    from pathlib import Path

    import atlas_code_quant

    repo_root = Path(atlas_code_quant.__file__).resolve().parent
    target = "scanner_radar_shadow"
    api_target = "run_scanner_radar_shadow"

    for sub in ("scanner", "strategies", "execution", "api"):
        sub_root = repo_root / sub
        if not sub_root.exists():
            continue
        for path in sub_root.rglob("*.py"):
            try:
                text = path.read_text(encoding="utf-8")
            except OSError:
                continue
            assert target not in text, (
                f"{path} importa el shadow consumer; "
                "F8 debe permanecer no enganchado a scanner/strategies/"
                "execution/api"
            )
            assert api_target not in text, (
                f"{path} llama a run_scanner_radar_shadow; "
                "F8 debe permanecer no enganchado en runtime"
            )
