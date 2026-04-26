"""Contrato Quant→PUSH para el resumen del Radar (mapper vs shape del scanner).

Detecta deriva silenciosa: si Quant renombra campos que el mapper lee para scores,
los tests de oro o de umbral deben fallar.
"""
from __future__ import annotations

from atlas_adapter.routes.radar_quant_mapper import build_dashboard_summary, compute_degradations_active
from atlas_adapter.routes.radar_schemas import RadarSummaryPayload

# Claves que ``build_dashboard_summary`` / ``pick_candidate`` esperan en el reporte.
_REPORT_CONTRACT_KEYS = (
    "generated_at",
    "status",
    "summary",
    "candidates",
    "rejections",
    "activity",
    "order_flow_system",
)


def _golden_scanner_report() -> dict:
    """Fixture representativo alineado con ``ScannerReportPayload`` y uso real del mapper."""
    return {
        "generated_at": "2026-04-26T12:00:00+00:00",
        "error": None,
        "status": {
            "running": True,
            "cycle_count": 42,
            "current_symbol": "SPY",
            "current_step": "scan",
            "last_error": None,
            "last_cycle_ms": 1500.0,
        },
        "summary": {"cycle_count": 42},
        "criteria": [],
        "universe": {},
        "candidates": [
            {
                "symbol": "SPY",
                "direction": "long",
                "signal_strength_pct": 72.0,
                "selection_score": 80.0,
                "local_win_rate_pct": 65.0,
                "macd_hist": 0.35,
                "bb_pct": 0.12,
                "timeframe": "5m",
                "strategy_label": "momentum",
                "vix_context": {"gate": "ok"},
                "iv_rank": 45.0,
                "iv_hv_ratio": 1.1,
                "rsi": 55.0,
                "volume_ratio": 1.25,
            }
        ],
        "rejections": [],
        "activity": [],
        "current_work": {},
        "learning": {},
        "order_flow_system": {"provider_ready": True, "provider_error": ""},
    }


def test_scanner_report_fixture_has_contract_keys() -> None:
    rep = _golden_scanner_report()
    for k in _REPORT_CONTRACT_KEYS:
        assert k in rep, f"Falta clave de contrato en fixture: {k}"


def test_build_dashboard_summary_golden_scores_and_schema() -> None:
    rep = _golden_scanner_report()
    body = build_dashboard_summary("SPY", rep, quant_ms=12.34)
    meta = body["radar"]["signal"]["meta"]
    assert body["symbol"] == "SPY"
    assert body["transport"]["stub"] is False
    assert body["transport"]["quant"] is True
    assert meta["snapshot_classification"] == "fully_operable"
    # Umbral: si Quant deja de enviar signal_strength/selection, el mapper cae a 0.0 sin error.
    assert meta["fast_pressure_score"] > 0.5, "contrato: fast_pressure debería reflejar signal_strength_pct"
    assert meta["structural_confidence_score"] > 0.5
    assert body["quant"]["scanner_running"] is True
    RadarSummaryPayload.model_validate(body)


def test_build_dashboard_summary_quant_field_rename_drops_pressure() -> None:
    """Si renombran la fuerza sin actualizar el mapper, el score cae a 0 (fallo silencioso)."""
    rep = _golden_scanner_report()
    c0 = rep["candidates"][0]
    c0.pop("signal_strength_pct", None)
    c0.pop("selection_score", None)
    c0["signal_strength_pct_LEGACY"] = 99.0
    body = build_dashboard_summary("SPY", rep)
    fp = body["radar"]["signal"]["meta"]["fast_pressure_score"]
    assert fp == 0.0


def test_build_dashboard_summary_degraded_without_running() -> None:
    rep = _golden_scanner_report()
    rep["status"] = {"running": False, "cycle_count": 0, "last_error": None}
    body = build_dashboard_summary("SPY", rep)
    cls = body["radar"]["signal"]["meta"]["snapshot_classification"]
    assert cls in ("structural_only", "non_operable", "operable_with_degradation")
    codes = {d["code"] for d in (body.get("degradations_active") or [])}
    assert "PROVIDERS_DEGRADED" in codes
    RadarSummaryPayload.model_validate(body)


def test_compute_degradations_active_stub_and_quant() -> None:
    d = compute_degradations_active(
        {"stub": True, "quant": False, "sse": True},
        {"providers_checked": 1, "degraded_count": 0},
        {"provider_ready": True, "state": "ready"},
    )
    codes = {x["code"] for x in d}
    assert "STUB_MODE" in codes
    assert "QUANT_UNREACHABLE" in codes
    assert "CAMERA_UNAVAILABLE" not in codes


def test_compute_degradations_providers_and_camera() -> None:
    d = compute_degradations_active(
        {"stub": False, "quant": True, "sse": True},
        {"providers_checked": 2, "degraded_count": 1},
        {"provider_ready": False, "state": "unavailable"},
    )
    codes = {x["code"] for x in d}
    assert "PROVIDERS_DEGRADED" in codes
    assert "CAMERA_UNAVAILABLE" in codes
    assert "STUB_MODE" not in codes


def test_build_dashboard_summary_missing_spy_candidate_degraded() -> None:
    rep = _golden_scanner_report()
    rep["candidates"][0]["symbol"] = "QQQ"
    body = build_dashboard_summary("SPY", rep)
    assert body["radar"]["signal"]["meta"]["snapshot_classification"] == "operable_with_degradation"
