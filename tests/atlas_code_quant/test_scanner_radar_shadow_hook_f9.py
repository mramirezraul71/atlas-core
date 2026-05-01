"""Tests F9 — runtime hook (opt-in) scanner→Radar shadow.

Cubre:

* Flag default ``False`` → hook devuelve ``None`` y NO toca el Radar
  ni emite logs del logger F8.
* Flag ``True`` → hook delega a ``run_scanner_radar_shadow`` (F8),
  retornando un :class:`ScannerRadarShadowReport` poblado.
* Degradaciones (timeout, 5xx, payload inválido) → hook NUNCA lanza,
  reporte trae ``degradations``.
* Hook NO muta ``scanner_candidates``.
* La flag ``ATLAS_RADAR_FILTER_ENFORCED`` existe, default ``False`` y
  NO se consulta en runtime (verificación textual).
* Static guard: scanner/strategies/execution/api siguen sin importar
  ni el hook ni el consumer F8.
* Smoke: la flag F8 sigue ausente del cuerpo de
  ``scanner_radar_shadow.py`` (preserva la invariante F8).
"""

from __future__ import annotations

import logging
from pathlib import Path
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
from atlas_code_quant.monitoring import scanner_radar_shadow_hook
from atlas_code_quant.monitoring.scanner_radar_shadow import (
    ScannerRadarShadowReport,
)
from atlas_code_quant.monitoring.scanner_radar_shadow_hook import (
    ENFORCEMENT_FLAG_NAME,
    RUNTIME_FLAG_NAME,
    maybe_run_scanner_radar_shadow,
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
        "trace_id": "trace-batch-f9",
    }


def _make_client(handler) -> RadarClient:
    transport = httpx.MockTransport(handler)
    cfg = RadarClientConfig(base_url="http://radar.test", timeout_ms=2000)
    return RadarClient(cfg, transport=transport)


@pytest.fixture
def runtime_flag_enabled(monkeypatch):
    """Activa ``ATLAS_RADAR_FILTER_SHADOW_RUNTIME_ENABLED`` en runtime
    sin escribir en el módulo permanente."""
    monkeypatch.setattr(
        legacy_flags, RUNTIME_FLAG_NAME, True, raising=True
    )
    yield


# ---------------------------------------------------------------------------
# Default: flag False — hook es no-op
# ---------------------------------------------------------------------------


def test_runtime_flag_default_remains_false():
    assert legacy_flags.ATLAS_RADAR_FILTER_SHADOW_RUNTIME_ENABLED is False


def test_hook_returns_none_when_runtime_flag_disabled():
    """Comportamiento por defecto: la flag está ``False`` → hook
    devuelve ``None`` sin construir Radar ni iterar entradas."""

    calls = {"radar": 0}

    class _SpyClient:
        def fetch_opportunities_snapshot(self, *args, **kwargs):  # pragma: no cover
            calls["radar"] += 1
            raise AssertionError("Radar no debe ser tocado con flag False")

    result = maybe_run_scanner_radar_shadow(
        [{"symbol": "AAPL"}], radar_client=_SpyClient()  # type: ignore[arg-type]
    )

    assert result is None
    assert calls["radar"] == 0


def test_hook_disabled_does_not_consume_iterable():
    """Con flag ``False`` el hook NO debe iterar la entrada."""

    consumed = {"n": 0}

    def gen():
        consumed["n"] += 1
        yield {"symbol": "AAPL"}

    iterator = gen()
    result = maybe_run_scanner_radar_shadow(iterator)

    assert result is None
    assert consumed["n"] == 0  # generador no se tocó


def test_hook_disabled_emits_no_f8_logs(caplog):
    with caplog.at_level(
        logging.DEBUG, logger="atlas.code_quant.shadow.scanner_radar"
    ):
        result = maybe_run_scanner_radar_shadow(
            [{"symbol": "AAPL"}], emit_logs=True
        )

    assert result is None
    f8_events = [
        getattr(r, "event", None)
        for r in caplog.records
        if r.name == "atlas.code_quant.shadow.scanner_radar"
    ]
    assert f8_events == []


# ---------------------------------------------------------------------------
# Flag True: hook delega a F8
# ---------------------------------------------------------------------------


def test_hook_enabled_delegates_to_run_scanner_radar_shadow(
    runtime_flag_enabled,
):
    def handler(request: httpx.Request) -> httpx.Response:
        items = [
            _opp_payload("AAPL", score=85.0, classification="high_conviction"),
            _opp_payload("MSFT", score=40.0, classification="watchlist"),
        ]
        return httpx.Response(200, json=_batch_payload(items))

    client = _make_client(handler)
    candidates = [{"symbol": "AAPL"}, {"symbol": "MSFT"}]

    report = maybe_run_scanner_radar_shadow(
        candidates, radar_client=client, emit_logs=False
    )

    assert isinstance(report, ScannerRadarShadowReport)
    assert report.scanner_candidate_count == 2
    assert report.radar_approved_count == 1
    assert report.radar_rejected_count == 1
    assert report.approved_symbols == ("AAPL",)
    assert report.rejected_symbols == ("MSFT",)
    assert report.degradations == ()


def test_hook_enabled_forwards_kwargs(runtime_flag_enabled):
    """Verifica que ``radar_client``, ``min_score`` y ``emit_logs``
    se forwardean al consumer F8 sin alteración."""

    sentinel_report = ScannerRadarShadowReport(
        scanner_candidate_count=7,
        radar_approved_count=0,
        radar_rejected_count=0,
        approved_symbols=(),
        rejected_symbols=(),
        degradations=(),
        metadata={"trace_id": "sentinel"},
        filter_result=None,  # type: ignore[arg-type]
    )

    target = (
        "atlas_code_quant.monitoring.scanner_radar_shadow_hook."
        "run_scanner_radar_shadow"
    )
    with patch(target, return_value=sentinel_report) as mock_run:
        client = object()
        out = maybe_run_scanner_radar_shadow(
            [{"symbol": "AAPL"}],
            radar_client=client,  # type: ignore[arg-type]
            min_score=42.0,
            emit_logs=False,
        )

    assert out is sentinel_report
    mock_run.assert_called_once()
    args, kwargs = mock_run.call_args
    assert args[0] == [{"symbol": "AAPL"}]
    assert kwargs["radar_client"] is client
    assert kwargs["min_score"] == 42.0
    assert kwargs["emit_logs"] is False


def test_hook_enabled_with_empty_input_returns_empty_report(
    runtime_flag_enabled,
):
    report = maybe_run_scanner_radar_shadow([], emit_logs=False)
    assert isinstance(report, ScannerRadarShadowReport)
    assert report.scanner_candidate_count == 0
    assert report.degradations == ()


def test_hook_enabled_with_none_returns_empty_report(runtime_flag_enabled):
    report = maybe_run_scanner_radar_shadow(None, emit_logs=False)
    assert isinstance(report, ScannerRadarShadowReport)
    assert report.scanner_candidate_count == 0


# ---------------------------------------------------------------------------
# Degradaciones (heredadas de F8 — el hook nunca lanza)
# ---------------------------------------------------------------------------


@pytest.mark.parametrize(
    "exc_factory,expected_code",
    [
        (lambda req: httpx.ReadTimeout("t", request=req), INTAKE_CODE_TIMEOUT),
        (lambda req: httpx.ConnectError("u", request=req), INTAKE_CODE_UNREACHABLE),
    ],
)
def test_hook_handles_transport_failures(
    runtime_flag_enabled, exc_factory, expected_code
):
    def handler(request: httpx.Request) -> httpx.Response:
        raise exc_factory(request)

    client = _make_client(handler)
    report = maybe_run_scanner_radar_shadow(
        [{"symbol": "AAPL"}], radar_client=client, emit_logs=False
    )

    assert report is not None
    assert report.scanner_candidate_count == 1
    assert any(d.code == expected_code for d in report.degradations)


def test_hook_handles_http_5xx(runtime_flag_enabled):
    def handler(request: httpx.Request) -> httpx.Response:
        return httpx.Response(503)

    client = _make_client(handler)
    report = maybe_run_scanner_radar_shadow(
        [{"symbol": "AAPL"}], radar_client=client, emit_logs=False
    )

    assert report is not None
    assert any(d.code == INTAKE_CODE_HTTP_ERROR for d in report.degradations)


def test_hook_handles_invalid_payload(runtime_flag_enabled):
    def handler(request: httpx.Request) -> httpx.Response:
        return httpx.Response(
            200,
            content=b"not-json",
            headers={"content-type": "application/json"},
        )

    client = _make_client(handler)
    report = maybe_run_scanner_radar_shadow(
        [{"symbol": "AAPL"}], radar_client=client, emit_logs=False
    )

    assert report is not None
    assert any(
        d.code == INTAKE_CODE_INVALID_PAYLOAD for d in report.degradations
    )


# ---------------------------------------------------------------------------
# No-side-effect operativo
# ---------------------------------------------------------------------------


def test_hook_does_not_mutate_scanner_input(runtime_flag_enabled):
    cand_a = {"symbol": "AAPL", "asset_class": "equity", "score": 0.5}
    cand_b = {"symbol": "MSFT", "asset_class": "equity", "score": 0.6}
    original = [cand_a, cand_b]
    snapshot = [dict(cand_a), dict(cand_b)]

    def handler(request: httpx.Request) -> httpx.Response:
        items = [_opp_payload("AAPL", score=85.0, classification="high_conviction")]
        return httpx.Response(200, json=_batch_payload(items))

    client = _make_client(handler)
    maybe_run_scanner_radar_shadow(
        original, radar_client=client, emit_logs=False
    )

    assert original == snapshot
    assert cand_a == snapshot[0]
    assert cand_b == snapshot[1]


# ---------------------------------------------------------------------------
# Enforcement gate (doc-only en F9)
# ---------------------------------------------------------------------------


def test_enforcement_flag_exists_and_default_false():
    assert hasattr(legacy_flags, ENFORCEMENT_FLAG_NAME)
    assert getattr(legacy_flags, ENFORCEMENT_FLAG_NAME) is False
    assert ENFORCEMENT_FLAG_NAME in legacy_flags.__all__


def test_enforcement_flag_not_consulted_in_runtime_modules():
    """``ATLAS_RADAR_FILTER_ENFORCED`` debe ser doc-only en F9: ningún
    módulo runtime de la pipeline shadow puede leerla todavía.

    Aceptamos menciones en docstrings/comentarios (referencia documental).
    Rechazamos cualquier patrón que indique consumo real:
        * acceso a atributo: ``legacy_flags.ATLAS_RADAR_FILTER_ENFORCED``
        * import directo:    ``from ... import ATLAS_RADAR_FILTER_ENFORCED``
    """
    import atlas_code_quant.intake.radar_client as m_client
    import atlas_code_quant.intake.scanner_radar_filter as m_filter
    import atlas_code_quant.intake.opportunity as m_intake
    import atlas_code_quant.monitoring.scanner_radar_shadow as m_f8
    import atlas_code_quant.monitoring.scanner_radar_shadow_hook as m_hook

    for mod in (m_client, m_filter, m_intake, m_f8, m_hook):
        src = Path(mod.__file__).read_text(encoding="utf-8")
        assert (
            "legacy_flags.ATLAS_RADAR_FILTER_ENFORCED" not in src
        ), f"{mod.__file__} consume ENFORCED en runtime (prohibido en F9)"
        assert (
            "import ATLAS_RADAR_FILTER_ENFORCED" not in src
        ), f"{mod.__file__} importa ENFORCED en runtime (prohibido en F9)"


# ---------------------------------------------------------------------------
# Static guards
# ---------------------------------------------------------------------------


def test_f8_invariant_preserved_runtime_flag_absent_from_f8_module():
    """La invariante F8 sigue intacta: la flag runtime NO debe aparecer
    textualmente en ``scanner_radar_shadow.py``."""
    import atlas_code_quant.monitoring.scanner_radar_shadow as f8_mod

    src = Path(f8_mod.__file__).read_text(encoding="utf-8")
    assert "ATLAS_RADAR_FILTER_SHADOW_RUNTIME_ENABLED" not in src


def test_hook_module_consults_runtime_flag_textually():
    """El hook F9 SÍ debe consultar la flag runtime — única lectura
    permitida en runtime de toda la pipeline shadow."""
    import atlas_code_quant.monitoring.scanner_radar_shadow_hook as hook_mod

    src = Path(hook_mod.__file__).read_text(encoding="utf-8")
    assert "ATLAS_RADAR_FILTER_SHADOW_RUNTIME_ENABLED" in src


def test_no_module_under_scanner_strategies_execution_api_imports_hook():
    """Confirmación: el hook F9 NO está enganchado todavía a scanner,
    strategies, execution ni api. Si alguna fase futura introduce el
    enganche, este test falla y obliga a documentarlo en cutover.

    Lectura textual pura sobre archivos .py: NO importa los paquetes.
    """
    import atlas_code_quant

    repo_root = Path(atlas_code_quant.__file__).resolve().parent
    targets = (
        "scanner_radar_shadow_hook",
        "maybe_run_scanner_radar_shadow",
    )

    for sub in ("scanner", "strategies", "execution", "api"):
        sub_root = repo_root / sub
        if not sub_root.exists():
            continue
        for path in sub_root.rglob("*.py"):
            try:
                text = path.read_text(encoding="utf-8")
            except OSError:
                continue
            for needle in targets:
                assert needle not in text, (
                    f"{path} referencia el hook F9 ({needle}); "
                    "F9 debe permanecer no enganchado a "
                    "scanner/strategies/execution/api"
                )


def test_hook_module_logger_name_canonical():
    """Smoke: el logger del hook sigue el namespace acordado."""
    assert (
        scanner_radar_shadow_hook.logger.name
        == "atlas.code_quant.shadow.scanner_radar.hook"
    )
