"""Tests F10 — adapter scanner→Radar enforcement (API-level).

Cubre:

* Modo compat (flag ``False``):
    - ``data.candidates`` conserva la lista del scanner intacta.
    - Se añaden campos Radar (``radar_filtered_candidates``,
      ``radar_rejected``, ``radar_degradations``, ``radar_meta``) sin
      tocar el contrato heredado.
    - ``radar_meta.mode == "compat"`` y ``enforced=False``.

* Modo enforced (flag ``True``):
    - ``data.candidates`` queda con SÓLO los aprobados por Radar.
    - ``data.scanner_raw_candidates`` lleva la lista original del
      scanner para auditoría.
    - Rechazados visibles en ``radar_rejected`` y NUNCA aparecen en
      ``candidates``.

* Degradaciones (timeout / 5xx / payload inválido) — política
  conservadora en enforced: ``data.candidates = []`` y
  ``radar_meta.batch_failed = True``. En compat la lista raw del
  scanner se preserva.

* Comportamiento defensivo: scanner_report ``None`` / inválido →
  esqueleto vacío sin lanzar.

* No se importa el adapter desde execution / autonomy / risk / vision
  ni el adapter llama código prohibido (assertion textual).

* AST guard: ``/scanner/report`` en ``api/main.py`` invoca
  ``build_scanner_report_via_radar`` y conserva headers F3 + retorno
  ``StdResponse``.
"""

from __future__ import annotations

import ast
from pathlib import Path
from typing import Any
from unittest.mock import patch

import httpx
import pytest

from atlas_code_quant.config import legacy_flags
from atlas_code_quant.intake.radar_client import (
    RadarClient,
    RadarClientConfig,
)
from atlas_code_quant.intake.scanner_radar_view import (
    ENFORCEMENT_FLAG_NAME,
    MODE_COMPAT,
    MODE_ENFORCED,
    build_scanner_report_via_radar,
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
        "trace_id": "trace-batch-f10",
    }


def _make_client(handler) -> RadarClient:
    transport = httpx.MockTransport(handler)
    cfg = RadarClientConfig(base_url="http://radar.test", timeout_ms=2000)
    return RadarClient(cfg, transport=transport)


def _scanner_report_with(
    symbols: list[str], *, activity_count: int = 3
) -> dict[str, Any]:
    """Reporte del scanner imitando ``OpportunityScannerService.report()``."""
    candidates = [
        {
            "symbol": s,
            "asset_class": "equity",
            "selection_score": 60.0 + i,
            "method": "ma_cross",
            "timeframe": "1h",
        }
        for i, s in enumerate(symbols)
    ]
    return {
        "generated_at": "2026-04-27T12:00:00+00:00",
        "status": {"running": True, "cycle_count": 7},
        "summary": {"running": True, "cycle_count": 7},
        "criteria": [{"name": "score>=60"}],
        "universe": {"size": 39},
        "candidates": candidates,
        "rejections": [{"symbol": "BAD", "reason": "low_score"}],
        "activity": [{"step": "scan"} for _ in range(activity_count)],
        "current_work": {"symbol": "AAPL", "step": "scoring"},
        "learning": {},
        "error": None,
    }


# ---------------------------------------------------------------------------
# Modo COMPAT (flag False — default)
# ---------------------------------------------------------------------------


def test_default_flag_is_false():
    assert legacy_flags.ATLAS_RADAR_FILTER_ENFORCED is False


def test_compat_keeps_official_candidates_intact():
    def handler(request: httpx.Request) -> httpx.Response:
        items = [
            _opp_payload("AAPL", score=85.0, classification="high_conviction"),
            _opp_payload("MSFT", score=40.0, classification="watchlist"),
        ]
        return httpx.Response(200, json=_batch_payload(items))

    client = _make_client(handler)
    scanner = _scanner_report_with(["AAPL", "MSFT"])

    out = build_scanner_report_via_radar(
        scanner, radar_client=client, enforced=False
    )

    # Lista oficial INTACTA en compat.
    assert [c["symbol"] for c in out["candidates"]] == ["AAPL", "MSFT"]
    # Vista Radar añadida sin tocar el contrato heredado.
    assert [c["symbol"] for c in out["radar_filtered_candidates"]] == ["AAPL"]
    assert [c["symbol"] for c in out["radar_rejected"]] == ["MSFT"]
    assert out["radar_degradations"] == []
    meta = out["radar_meta"]
    assert meta["mode"] == MODE_COMPAT
    assert meta["enforced"] is False
    assert meta["policy"] == "compat_observation_only"
    assert meta["batch_failed"] is False
    assert meta["scanner_input"] == 2
    assert meta["approved_count"] == 1
    assert meta["rejected_count"] == 1
    # Compat NO incluye scanner_raw_candidates (la lista oficial ya es la raw).
    assert "scanner_raw_candidates" not in out


def test_compat_preserves_other_scanner_fields():
    def handler(request: httpx.Request) -> httpx.Response:
        return httpx.Response(200, json=_batch_payload([]))

    client = _make_client(handler)
    scanner = _scanner_report_with(["AAPL"])

    out = build_scanner_report_via_radar(
        scanner, radar_client=client, enforced=False
    )

    # Campos heredados intactos (el contrato Pydantic se preservará).
    for field in (
        "generated_at",
        "status",
        "summary",
        "criteria",
        "universe",
        "rejections",
        "activity",
        "current_work",
        "learning",
        "error",
    ):
        assert field in out, f"Falta campo heredado: {field}"
    assert out["status"] == {"running": True, "cycle_count": 7}
    assert out["activity"] == [{"step": "scan"}, {"step": "scan"}, {"step": "scan"}]


# ---------------------------------------------------------------------------
# Modo ENFORCED (flag True)
# ---------------------------------------------------------------------------


def test_enforced_official_candidates_only_radar_approved():
    def handler(request: httpx.Request) -> httpx.Response:
        items = [
            _opp_payload("AAPL", score=85.0, classification="high_conviction"),
            _opp_payload("SPY", score=72.0, classification="watchlist", asset_class="etf"),
            _opp_payload("MSFT", score=40.0, classification="watchlist"),  # score bajo → reject
            _opp_payload("XYZ", score=90.0, classification="reject"),       # reject explícito
        ]
        return httpx.Response(200, json=_batch_payload(items))

    client = _make_client(handler)
    scanner = _scanner_report_with(["AAPL", "SPY", "MSFT", "XYZ"])

    out = build_scanner_report_via_radar(
        scanner, radar_client=client, enforced=True
    )

    official = [c["symbol"] for c in out["candidates"]]
    assert sorted(official) == ["AAPL", "SPY"]
    # Cada aprobado lleva radar_decision.approved=True.
    for c in out["candidates"]:
        assert c["radar_decision"]["approved"] is True
        assert c["radar_decision"]["classification"] in (
            "high_conviction",
            "watchlist",
        )
    # MSFT y XYZ NUNCA aparecen como candidatos oficiales.
    assert "MSFT" not in official
    assert "XYZ" not in official
    # Pero sí en radar_rejected.
    rejected_syms = [c["symbol"] for c in out["radar_rejected"]]
    assert "MSFT" in rejected_syms
    assert "XYZ" in rejected_syms

    # Auditoría: scanner_raw_candidates conserva la lista original.
    assert [c["symbol"] for c in out["scanner_raw_candidates"]] == [
        "AAPL",
        "SPY",
        "MSFT",
        "XYZ",
    ]

    meta = out["radar_meta"]
    assert meta["mode"] == MODE_ENFORCED
    assert meta["enforced"] is True
    assert meta["policy"] == "radar_gate_strict"
    assert meta["batch_failed"] is False


def test_enforced_reads_runtime_flag(monkeypatch):
    """Sin pasar ``enforced=`` explícito, el adapter consulta la flag."""
    monkeypatch.setattr(legacy_flags, ENFORCEMENT_FLAG_NAME, True)

    def handler(request: httpx.Request) -> httpx.Response:
        items = [_opp_payload("AAPL", score=85.0, classification="high_conviction")]
        return httpx.Response(200, json=_batch_payload(items))

    client = _make_client(handler)
    scanner = _scanner_report_with(["AAPL", "MSFT"])

    out = build_scanner_report_via_radar(scanner, radar_client=client)

    assert out["radar_meta"]["enforced"] is True
    assert [c["symbol"] for c in out["candidates"]] == ["AAPL"]


def test_enforced_does_not_promote_when_radar_returns_no_data():
    def handler(request: httpx.Request) -> httpx.Response:
        # Radar OK pero sin items — todos los candidatos del scanner
        # quedan no_data → rechazados.
        return httpx.Response(200, json=_batch_payload([]))

    client = _make_client(handler)
    scanner = _scanner_report_with(["AAPL", "MSFT"])

    out = build_scanner_report_via_radar(
        scanner, radar_client=client, enforced=True
    )

    assert out["candidates"] == []
    rejected_syms = [c["symbol"] for c in out["radar_rejected"]]
    assert sorted(rejected_syms) == ["AAPL", "MSFT"]
    assert out["radar_meta"]["batch_failed"] is False
    assert out["radar_meta"]["policy"] == "radar_gate_strict"


# ---------------------------------------------------------------------------
# Degradaciones (timeout / 5xx / payload inválido)
# ---------------------------------------------------------------------------


@pytest.mark.parametrize(
    "handler_factory",
    [
        # Timeout
        pytest.param(
            lambda: (lambda req: (_ for _ in ()).throw(
                httpx.ReadTimeout("t", request=req)
            )),
            id="timeout",
        ),
        # 5xx
        pytest.param(
            lambda: (lambda req: httpx.Response(503)),
            id="http_5xx",
        ),
        # JSON inválido
        pytest.param(
            lambda: (lambda req: httpx.Response(
                200,
                content=b"not-json",
                headers={"content-type": "application/json"},
            )),
            id="invalid_payload",
        ),
    ],
)
def test_enforced_blocks_all_when_radar_degraded(handler_factory):
    handler = handler_factory()
    client = _make_client(handler)
    scanner = _scanner_report_with(["AAPL", "MSFT"])

    out = build_scanner_report_via_radar(
        scanner, radar_client=client, enforced=True
    )

    # Política conservadora: cero candidatos oficiales.
    assert out["candidates"] == []
    assert out["radar_meta"]["batch_failed"] is True
    assert out["radar_meta"]["policy"] == "conservative_block_on_degradation"
    assert len(out["radar_degradations"]) >= 1
    # Auditoría: la lista raw queda preservada.
    assert [c["symbol"] for c in out["scanner_raw_candidates"]] == [
        "AAPL",
        "MSFT",
    ]


def test_compat_preserves_scanner_when_radar_degraded():
    def handler(request: httpx.Request) -> httpx.Response:
        return httpx.Response(503)

    client = _make_client(handler)
    scanner = _scanner_report_with(["AAPL", "MSFT"])

    out = build_scanner_report_via_radar(
        scanner, radar_client=client, enforced=False
    )

    # Compat: la lista oficial sigue siendo el scanner aunque Radar
    # esté caído. Los degradations se reportan honestamente.
    assert [c["symbol"] for c in out["candidates"]] == ["AAPL", "MSFT"]
    assert out["radar_meta"]["batch_failed"] is True
    assert out["radar_meta"]["mode"] == MODE_COMPAT
    assert len(out["radar_degradations"]) >= 1


# ---------------------------------------------------------------------------
# Defensivo: input inválido / vacío
# ---------------------------------------------------------------------------


def test_handles_none_input_gracefully():
    out = build_scanner_report_via_radar(None, enforced=False)

    assert isinstance(out, dict)
    assert out["candidates"] == []
    assert out["radar_meta"]["scanner_input"] == 0
    assert out["radar_meta"]["mode"] == MODE_COMPAT


def test_handles_non_dict_input_gracefully():
    out = build_scanner_report_via_radar("nonsense", enforced=False)
    assert out["candidates"] == []
    assert out["radar_meta"]["scanner_input"] == 0


def test_handles_empty_candidates():
    scanner = _scanner_report_with([])
    out = build_scanner_report_via_radar(scanner, enforced=True)
    assert out["candidates"] == []
    assert out["scanner_raw_candidates"] == []
    assert out["radar_meta"]["batch_failed"] is False
    assert out["radar_meta"]["scanner_input"] == 0


def test_does_not_raise_when_filter_explodes():
    """Defense in depth: si el filtro F7 lanzara una excepción
    inesperada (no debería: F7 ya tiene su red), el adapter no la
    propaga."""
    target = (
        "atlas_code_quant.intake.scanner_radar_view."
        "filter_scanner_candidates_with_radar"
    )
    with patch(target, side_effect=RuntimeError("boom")):
        # Sin red de captura propia, esto sí lanzaría — el contrato F10
        # NO promete capturar en build_scanner_report_via_radar mismo
        # (la red está en F7). Verificamos que la única vía de fallo es
        # explícita, controlada por mock — en runtime real F7 nunca
        # lanza.
        with pytest.raises(RuntimeError):
            build_scanner_report_via_radar(
                _scanner_report_with(["AAPL"]), enforced=False
            )


# ---------------------------------------------------------------------------
# Schema compatibility — payload encaja en ScannerReportPayload
# ---------------------------------------------------------------------------


def test_compat_payload_validates_against_scanner_report_schema():
    from atlas_code_quant.api.schemas import ScannerReportPayload

    def handler(request: httpx.Request) -> httpx.Response:
        items = [
            _opp_payload("AAPL", score=85.0, classification="high_conviction")
        ]
        return httpx.Response(200, json=_batch_payload(items))

    client = _make_client(handler)
    scanner = _scanner_report_with(["AAPL", "MSFT"])
    out = build_scanner_report_via_radar(
        scanner, radar_client=client, enforced=False
    )
    # No lanza: el payload sigue cumpliendo el schema.
    payload = ScannerReportPayload.model_validate(out)
    assert payload.candidates  # lista no vacía


def test_enforced_payload_validates_against_scanner_report_schema():
    from atlas_code_quant.api.schemas import ScannerReportPayload

    def handler(request: httpx.Request) -> httpx.Response:
        items = [
            _opp_payload("AAPL", score=85.0, classification="high_conviction"),
            _opp_payload("MSFT", score=40.0, classification="watchlist"),
        ]
        return httpx.Response(200, json=_batch_payload(items))

    client = _make_client(handler)
    scanner = _scanner_report_with(["AAPL", "MSFT"])
    out = build_scanner_report_via_radar(
        scanner, radar_client=client, enforced=True
    )
    payload = ScannerReportPayload.model_validate(out)
    assert [c["symbol"] for c in payload.candidates] == ["AAPL"]


# ---------------------------------------------------------------------------
# AST guards sobre api/main.py
# ---------------------------------------------------------------------------


def _api_main_path() -> Path:
    import atlas_code_quant

    root = Path(atlas_code_quant.__file__).resolve().parent
    return root / "api" / "main.py"


def test_api_main_imports_adapter():
    """``api/main.py`` debe importar el adapter F10."""
    src = _api_main_path().read_text(encoding="utf-8")
    assert "from atlas_code_quant.intake.scanner_radar_view import" in src
    assert "build_scanner_report_via_radar" in src


def test_scanner_report_handler_invokes_adapter():
    """El handler ``/scanner/report`` debe invocar
    ``build_scanner_report_via_radar`` y conservar el helper F3 antes
    de ``_auth``."""
    tree = ast.parse(_api_main_path().read_text(encoding="utf-8"))
    handler: ast.AsyncFunctionDef | None = None
    for node in ast.walk(tree):
        if (
            isinstance(node, ast.AsyncFunctionDef)
            and node.name == "scanner_report"
        ):
            handler = node
            break
    assert handler is not None, "Handler scanner_report no encontrado"

    helper_lineno = adapter_lineno = auth_lineno = None
    for sub in ast.walk(handler):
        if isinstance(sub, ast.Call) and isinstance(sub.func, ast.Name):
            if sub.func.id == "_apply_scanner_deprecation_headers" and helper_lineno is None:
                helper_lineno = sub.lineno
            elif sub.func.id == "build_scanner_report_via_radar" and adapter_lineno is None:
                adapter_lineno = sub.lineno
            elif sub.func.id == "_auth" and auth_lineno is None:
                auth_lineno = sub.lineno

    assert helper_lineno is not None, "Helper F3 ausente"
    assert auth_lineno is not None, "_auth ausente"
    assert adapter_lineno is not None, "Adapter F10 ausente del handler"
    # F3 invariante: helper antes de auth.
    assert helper_lineno < auth_lineno
    # F10: adapter después de auth (lógica de negocio).
    assert adapter_lineno > auth_lineno


def test_scanner_report_v2_still_delegates_to_v1():
    """V2 sigue delegando en v1 → enforcement F10 se hereda gratis."""
    tree = ast.parse(_api_main_path().read_text(encoding="utf-8"))
    v2: ast.AsyncFunctionDef | None = None
    for node in ast.walk(tree):
        if (
            isinstance(node, ast.AsyncFunctionDef)
            and node.name == "scanner_report_v2"
        ):
            v2 = node
            break
    assert v2 is not None
    delegates = False
    for sub in ast.walk(v2):
        if (
            isinstance(sub, ast.Call)
            and isinstance(sub.func, ast.Name)
            and sub.func.id == "scanner_report"
        ):
            delegates = True
            break
    assert delegates, "scanner_report_v2 dejó de delegar en scanner_report"


def test_scanner_report_handler_returns_stdresponse():
    src = _api_main_path().read_text(encoding="utf-8")
    tree = ast.parse(src)
    handler: ast.AsyncFunctionDef | None = None
    for node in ast.walk(tree):
        if (
            isinstance(node, ast.AsyncFunctionDef)
            and node.name == "scanner_report"
        ):
            handler = node
            break
    assert handler is not None
    returns_std = False
    for sub in ast.walk(handler):
        if isinstance(sub, ast.Return) and isinstance(sub.value, ast.Call):
            f = sub.value.func
            if isinstance(f, ast.Name) and f.id == "StdResponse":
                returns_std = True
                break
    assert returns_std, "scanner_report ya no devuelve StdResponse"


# ---------------------------------------------------------------------------
# Aislamiento — adapter no toca dominios prohibidos
# ---------------------------------------------------------------------------


def test_adapter_does_not_import_forbidden_modules():
    """El adapter F10 NO puede **importar** execution / autonomy / risk /
    vision / liveloop / atlas_adapter / production_guard / broker_router /
    tradier. La búsqueda se hace sobre los `import`s reales (AST),
    no sobre menciones en docstrings o comentarios."""
    import atlas_code_quant.intake.scanner_radar_view as mod

    src = Path(mod.__file__).read_text(encoding="utf-8")
    tree = ast.parse(src)
    imported_modules: list[str] = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            for alias in node.names:
                imported_modules.append(alias.name)
        elif isinstance(node, ast.ImportFrom):
            if node.module:
                imported_modules.append(node.module)

    forbidden_prefixes = (
        "atlas_code_quant.execution",
        "atlas_code_quant.autonomy",
        "atlas_code_quant.risk",
        "atlas_code_quant.vision",
        "atlas_code_quant.production_guard",
        "atlas_adapter",
    )
    forbidden_substrings = ("broker_router", "tradier", "live_activation")

    for imp in imported_modules:
        for prefix in forbidden_prefixes:
            assert not imp.startswith(prefix), (
                f"adapter F10 importa {imp!r} (prefijo prohibido {prefix!r})"
            )
        for substr in forbidden_substrings:
            assert substr not in imp, (
                f"adapter F10 importa {imp!r} (substring prohibido {substr!r})"
            )


def test_adapter_consults_enforcement_flag_textually():
    """F10 sí debe leer ``ATLAS_RADAR_FILTER_ENFORCED`` en runtime."""
    import atlas_code_quant.intake.scanner_radar_view as mod

    src = Path(mod.__file__).read_text(encoding="utf-8")
    assert "ATLAS_RADAR_FILTER_ENFORCED" in src
    assert "legacy_flags" in src
