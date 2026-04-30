"""Institutional Radar: UI estática + API en PUSH (:8791).

Modo demostración: respuestas mínimas si no hay motor ni Quant disponible.

Motor real (recomendado): proceso **Atlas Code-Quant** en HTTP separado; PUSH
hace de fachada y **no** expone ``ATLAS_QUANT_API_KEY`` al navegador.

Variables de entorno (PUSH):
- ``QUANT_BASE_URL`` o ``ATLAS_QUANT_API_URL``: base del API Quant (p. ej. ``http://127.0.0.1:8792``).
- ``ATLAS_QUANT_API_KEY`` o ``QUANT_API_KEY``: clave servidor→servidor (obligatoria para activar el puente).
- ``ATLAS_RADAR_QUANT_HTTP=0``: desactiva llamadas HTTP a Quant (solo stub).
- ``ATLAS_RADAR_SSE_HEARTBEAT_SEC`` / ``ATLAS_RADAR_SSE_SNAPSHOT_SEC``: intervalos SSE.
"""

from __future__ import annotations

import asyncio
import hashlib
import json
import logging
import os
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, AsyncIterator

from fastapi import APIRouter, HTTPException, Response
from fastapi.responses import FileResponse, HTMLResponse, StreamingResponse

from atlas_adapter.routes.radar_schemas import (
    RadarCameraHealthPayload,
    RadarCameraStatusPayload,
    RadarDealerPayload,
    RadarDecisionsRecentPayload,
    RadarDecisionsStatsPayload,
    RadarDiagnosticsPayload,
    RadarFastPayload,
    RadarPoliticalPayload,
    RadarSearchResponse,
    RadarStructuralPayload,
    RadarSummaryPayload,
)

from atlas_adapter.routes.radar_quant_client import (
    fetch_quant_camera_health,
    fetch_scanner_report_cached,
    fetch_universe_search,
    radar_quant_http_enabled,
)
from atlas_code_quant.config.feature_flags import AtlasFeatureFlags
from atlas_adapter.routes.radar_quant_mapper import (
    build_dashboard_summary,
    build_decisions_recent,
    build_decisions_stats,
    build_dealer,
    build_fast,
    build_political,
    build_providers_payload,
    build_structural,
    compute_degradations_active,
)

logger = logging.getLogger("atlas.radar.public")

_RADAR_JSON_RESPONSES_200 = {
    200: {
        "description": "JSON del radar (Quant vivo o stub; ver cabeceras X-Atlas-Radar-Source / X-Atlas-Radar-Stub).",
    },
}

# Tiempo real (SSE): ajustable sin tocar código (segundos).
_RADAR_SSE_HEARTBEAT_SEC = max(2, min(60, int(os.getenv("ATLAS_RADAR_SSE_HEARTBEAT_SEC", "5"))))
_RADAR_SSE_SNAPSHOT_SEC = max(
    _RADAR_SSE_HEARTBEAT_SEC,
    min(120, int(os.getenv("ATLAS_RADAR_SSE_SNAPSHOT_SEC", "12"))),
)


def _apply_radar_json_headers(response: Response, *, live: bool) -> None:
    response.headers["Cache-Control"] = "no-store, max-age=0, must-revalidate"
    response.headers["Pragma"] = "no-cache"
    if live:
        response.headers["X-Atlas-Radar-Source"] = "quant"
    else:
        response.headers["X-Atlas-Radar-Stub"] = "1"


def _apply_radar_symbols_search_headers(response: Response, *, live: bool) -> None:
    """Cabeceras de búsqueda de símbolos (TTL corta + SWR)."""
    response.headers["Cache-Control"] = (
        "private, max-age=12, stale-while-revalidate=24, must-revalidate"
    )
    response.headers["Pragma"] = "no-cache"
    response.headers["Vary"] = "Accept-Encoding"
    if live:
        response.headers["X-Atlas-Radar-Source"] = "quant"
    else:
        response.headers["X-Atlas-Radar-Stub"] = "1"


def _stream_headers(*, live: bool) -> dict[str, str]:
    h: dict[str, str] = {
        "Cache-Control": "no-cache, no-store",
        "Connection": "keep-alive",
        "X-Accel-Buffering": "no",
    }
    if live:
        h["X-Atlas-Radar-Source"] = "quant"
    else:
        h["X-Atlas-Radar-Stub"] = "1"
    return h


async def _scanner_report_live() -> dict[str, Any] | None:
    if not radar_quant_http_enabled():
        return None
    return await fetch_scanner_report_cached()

_ROUTER_DIR = Path(__file__).resolve().parent
_ADAPTER_DIR = _ROUTER_DIR.parent
_STATIC_RADAR = _ADAPTER_DIR / "static" / "radar"


def _utc_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


_CAMERA_STATE_LABELS_ES: dict[str, str] = {
    "ready": "Listo",
    "unavailable": "No disponible",
    "degraded": "Degradado",
    "disabled": "Desactivado",
    "not_configured": "Sin configurar",
}


def _decorate_radar_camera_fields(inner: dict[str, Any]) -> dict[str, Any]:
    """Campos derivados para la UI del radar (compat con dashboard.js)."""
    cam = dict(inner)
    state = str(cam.get("state") or "unknown")
    label = _CAMERA_STATE_LABELS_ES.get(state, state.replace("_", " "))
    reason = cam.get("degradation_reason")
    lines: list[str] = []
    if cam.get("provider"):
        lines.append(str(cam["provider"]))
    lines.append(f"Estado operativo: {label}.")
    if reason and str(reason).strip().lower() not in {"quant_unreachable", "none", ""}:
        lines.append(str(reason))
    pct = cam.get("availability_pct")
    try:
        if pct is not None:
            lines.append(f"Disponibilidad estimada: {float(pct):.1f}%.")
    except (TypeError, ValueError):
        logger.debug("decorate_radar_camera: availability_pct no numérico (%r)", pct)
    mode = cam.get("mode_detected")
    if mode:
        be = cam.get("backend")
        lines.append(f"Modo físico: {mode}" + (f" ({be})" if be else ""))
    if cam.get("device_index") is not None:
        lines.append(f"Índice USB resuelto: {cam.get('device_index')}.")
    cam["status"] = " ".join(lines).strip()
    cam["provider_ready"] = state == "ready"
    cam["last_capture"] = cam.get("last_capture_ts")
    pres = cam.get("presence")
    if isinstance(pres, bool):
        cam["presence_score"] = 1.0 if pres else 0.0
    else:
        cam["presence_score"] = None
    act = cam.get("activity")
    if isinstance(act, (int, float)):
        cam["activity_level"] = float(act)
    else:
        cam["activity_level"] = None
    cam["activity_note"] = act if isinstance(act, str) else None
    return cam


def _camera_quant_unreachable_envelope() -> dict[str, Any]:
    ts = _utc_iso()
    inner = {
        "provider": None,
        "state": "unavailable",
        "mode_detected": None,
        "backend": None,
        "device_index": None,
        "pnp_hints": [],
        "last_capture_ts": None,
        "presence": None,
        "activity": None,
        "availability_pct": 0.0,
        "degradation_reason": "quant_unreachable",
        "vision_provider": None,
        "provider_ready": False,
        "notes": None,
    }
    decorated = _decorate_radar_camera_fields(inner)
    decorated["status"] = (
        "Estado de cámara no disponible: motor Quant no accesible o radar sin HTTP/clave "
        "en el servidor (ATLAS_QUANT_API_KEY / QUANT_API_KEY)."
    )
    return {"ok": False, "source": "unavailable", "timestamp": ts, "camera": decorated}


async def build_live_camera_envelope() -> tuple[dict[str, Any], bool]:
    """Carga salud de cámara desde Quant; honesto si no hay puente HTTP o Quant caído."""
    if not radar_quant_http_enabled():
        return _camera_quant_unreachable_envelope(), False
    raw = await fetch_quant_camera_health()
    if raw is None:
        return _camera_quant_unreachable_envelope(), False
    cam = _decorate_radar_camera_fields(raw)
    return {"ok": True, "source": "quant", "timestamp": _utc_iso(), "camera": cam}, True


def _stub_camera() -> dict[str, Any]:
    return {
        "camera": {
            "provider": "N/D (sin sensor en modo demostración)",
            "status": "Sin captura en vivo: despliega atlas_scanner para habilitar el sensor.",
            "provider_ready": None,
            "last_capture": None,
            "presence_score": None,
            "activity_level": None,
        }
    }


def _stub_summary(symbol: str) -> dict[str, Any]:
    """Snapshot mínimo: clasificación honesta (no es degradación operativa real)."""
    ts = _utc_iso()
    return {
        "symbol": symbol,
        "last_update": ts,
        "stream_available": True,
        "transport": {
            "sse": True,
            "sse_heartbeat_sec": _RADAR_SSE_HEARTBEAT_SEC,
            "sse_snapshot_sec": _RADAR_SSE_SNAPSHOT_SEC,
            "stub": True,
            "quant": False,
        },
        "radar": {
            "signal": {
                "timestamp": ts,
                "bias": "neutral",
                "meta": {
                    # No usar operable_with_degradation aquí: implica señal real degradada.
                    # Sin atlas_scanner solo hay UI + stub; el motor debe fijar la clasificación real.
                    "snapshot_classification": "demonstration_without_engine",
                    "fast_pressure_score": 0.0,
                    "structural_confidence_score": 0.0,
                    "fast_structural_alignment": 0.0,
                    "fast_structural_divergence_score": 0.0,
                    "horizon_conflict": False,
                    "cross_horizon_alignment": "-",
                },
            }
        },
        "decision_gate": {"recent": [], "latest": None},
        "camera_context": _stub_camera()["camera"],
        "provider_health_summary": {"providers_checked": 1, "degraded_count": 0},
    }


async def radar_build_dashboard_for_sse(symbol: str) -> tuple[dict[str, Any], bool, dict[str, Any] | None]:
    """Cuerpo dashboard/summary + flag live + reporte scanner (para SSE y REST sin duplicar logica)."""
    sym = (symbol or "SPY").strip().upper() or "SPY"
    report = await _scanner_report_live()
    cam_env, _ = await build_live_camera_envelope()
    cam_panel = cam_env.get("camera") if isinstance(cam_env.get("camera"), dict) else {}
    if report is not None:
        t0 = time.perf_counter()
        body = build_dashboard_summary(sym, report, quant_ms=round((time.perf_counter() - t0) * 1000.0, 2))
        tr = dict(body.get("transport") or {})
        tr["sse_heartbeat_sec"] = _RADAR_SSE_HEARTBEAT_SEC
        tr["sse_snapshot_sec"] = _RADAR_SSE_SNAPSHOT_SEC
        body["transport"] = tr
        body["camera_context"] = cam_panel
        body["degradations_active"] = compute_degradations_active(
            body.get("transport") or {},
            body.get("provider_health_summary") or {},
            body.get("camera_context") or {},
        )
        return body, True, report
    stub = _stub_summary(sym)
    stub["camera_context"] = cam_panel
    stub["degradations_active"] = compute_degradations_active(
        stub.get("transport") or {},
        stub.get("provider_health_summary") or {},
        stub.get("camera_context") or {},
    )
    return stub, False, None


def _radar_json_sig(obj: Any) -> str:
    raw = json.dumps(obj, sort_keys=True, default=str, separators=(",", ":")).encode("utf-8")
    return hashlib.sha256(raw).hexdigest()[:20]


def _sse_event_block(seq: int, event_name: str, envelope: dict[str, Any]) -> str:
    """Un evento SSE con id (reconexion) y cuerpo JSON una linea."""
    line = json.dumps(envelope, ensure_ascii=False, separators=(",", ":"))
    return f"id: {seq}\nevent: {event_name}\ndata: {line}\n\n"


def build_radar_stub_api_router() -> APIRouter:
    router = APIRouter(prefix="/api/radar", tags=["Radar"])

    async def _summary_payload(symbol: str) -> tuple[dict[str, Any], bool]:
        body, live, _rep = await radar_build_dashboard_for_sse(symbol)
        return body, live

    @router.get(
        "/dashboard/summary",
        response_model=RadarSummaryPayload,
        responses=_RADAR_JSON_RESPONSES_200,
    )
    async def dashboard_summary(response: Response, symbol: str = "SPY") -> Any:
        body, live = await _summary_payload(symbol)
        _apply_radar_json_headers(response, live=live)
        return body

    @router.get(
        "/summary",
        response_model=RadarSummaryPayload,
        responses=_RADAR_JSON_RESPONSES_200,
    )
    async def summary_alias(response: Response, symbol: str = "SPY") -> Any:
        """Alias del resumen (misma carga que ``/dashboard/summary``)."""
        body, live = await _summary_payload(symbol)
        _apply_radar_json_headers(response, live=live)
        return body

    @router.get(
        "/symbols/search",
        response_model=RadarSearchResponse,
        responses=_RADAR_JSON_RESPONSES_200,
    )
    async def symbols_search(
        response: Response,
        q: str = "",
        limit: int = 25,
    ) -> Any:
        """Búsqueda de tickers sobre el universo del scanner Quant (fachada; API key solo en servidor)."""
        lim = max(1, min(int(limit or 25), 100))
        data = await fetch_universe_search(q, limit=lim)
        if data is not None:
            _apply_radar_symbols_search_headers(response, live=True)
            return {
                "ok": True,
                "source": "quant",
                "message": None,
                **data,
            }
        logger.warning(
            "symbols_search: Quant no disponible o sin clave; devolviendo stub (q=%r limit=%s)",
            q,
            lim,
        )
        # F2: si multi-símbolo está habilitado, ofrecer búsqueda local sobre universo curado
        # (no sustituye al motor Quant; mejora UX offline del Radar).
        if AtlasFeatureFlags().radar_multi_symbol_enabled:
            try:
                from atlas_adapter.services.universe_provider import UniverseProvider

                matches = [
                    {"symbol": e.symbol, "score": None, "asset_class": e.asset_class, "sector": e.sector}
                    for e in UniverseProvider().search(q, limit=lim)
                ]
                _apply_radar_symbols_search_headers(response, live=False)
                return {
                    "ok": True,
                    "source": "universe_curated",
                    "message": "Catálogo curado local (Quant no disponible).",
                    "query": q,
                    "matches": matches,
                    "truncated": len(matches) >= lim,
                }
            except Exception as exc:
                logger.debug("symbols_search: fallback universo F2 no disponible (%s)", exc)

        _apply_radar_symbols_search_headers(response, live=False)
        return {
            "ok": False,
            "source": "unavailable",
            "message": "Búsqueda no disponible: motor Quant no accesible o sin clave de API en el servidor.",
            "query": q,
            "matches": [],
            "truncated": False,
        }

    @router.get(
        "/diagnostics/providers",
        response_model=RadarDiagnosticsPayload,
        responses=_RADAR_JSON_RESPONSES_200,
    )
    async def diagnostics_providers(response: Response) -> Any:
        report = await _scanner_report_live()
        if report is not None:
            _apply_radar_json_headers(response, live=True)
            return build_providers_payload(report)
        logger.info("diagnostics_providers: sin reporte Quant; stub modo demostración")
        _apply_radar_json_headers(response, live=False)
        return {
            "providers": [
                {
                    "provider": "atlas_scanner (no desplegado en este host)",
                    "name": "atlas_scanner (no desplegado en este host)",
                    "is_ready": False,
                    "ready": False,
                    "stale_indicator": False,
                    "latency_ms": 0,
                    "p95_latency_ms": 0,
                    "active_fallback_indicator": False,
                    "circuit_open_indicator": False,
                    "consecutive_errors": 0,
                    "availability_ratio": 0.0,
                    "last_error_type": "modo_demostracion_sin_motor",
                    "ui_status_hint": "modo_demostracion",
                }
            ]
        }

    @router.get(
        "/decisions/recent",
        response_model=RadarDecisionsRecentPayload,
        responses=_RADAR_JSON_RESPONSES_200,
    )
    async def decisions_recent(response: Response, limit: int = 40) -> Any:
        report = await _scanner_report_live()
        if report is not None:
            _apply_radar_json_headers(response, live=True)
            return build_decisions_recent(report, limit=max(1, min(limit, 200)))
        _apply_radar_json_headers(response, live=False)
        return {"recent": []}

    @router.get(
        "/decisions/stats",
        response_model=RadarDecisionsStatsPayload,
        responses=_RADAR_JSON_RESPONSES_200,
    )
    async def decisions_stats(response: Response = Response()) -> Any:
        report = await _scanner_report_live()
        if report is not None:
            _apply_radar_json_headers(response, live=True)
            return build_decisions_stats(report)
        _apply_radar_json_headers(response, live=False)
        return {
            "stats": {
                "by_decision": {"accepted": 0, "rejected": 0},
                "avg_fast_pressure_score": None,
                "avg_structural_confidence_score": None,
            }
        }

    @router.get(
        "/dealer/{symbol}",
        response_model=RadarDealerPayload,
        responses=_RADAR_JSON_RESPONSES_200,
    )
    async def dealer(symbol: str, response: Response) -> Any:
        report = await _scanner_report_live()
        if report is not None:
            _apply_radar_json_headers(response, live=True)
            return build_dealer(symbol, report)
        _apply_radar_json_headers(response, live=False)
        return {
            "symbol": symbol,
            "timestamp": _utc_iso(),
            "gamma_flip_level": "-",
            "dealer_skew_score": None,
            "call_wall": "-",
            "put_wall": "-",
            "acceleration_zone_score": None,
        }

    @router.get(
        "/diagnostics/fast/{symbol}",
        response_model=RadarFastPayload,
        responses=_RADAR_JSON_RESPONSES_200,
    )
    async def fast_diag(symbol: str, response: Response) -> Any:
        report = await _scanner_report_live()
        if report is not None:
            _apply_radar_json_headers(response, live=True)
            return build_fast(symbol, report)
        _apply_radar_json_headers(response, live=False)
        return {
            "symbol": symbol,
            "timestamp": _utc_iso(),
            "fast_pressure_score": None,
            "fast_risk_score": None,
            "fast_directional_bias_score": None,
        }

    @router.get(
        "/diagnostics/structural/{symbol}",
        response_model=RadarStructuralPayload,
        responses=_RADAR_JSON_RESPONSES_200,
    )
    async def structural_diag(symbol: str, response: Response) -> Any:
        report = await _scanner_report_live()
        if report is not None:
            _apply_radar_json_headers(response, live=True)
            return build_structural(symbol, report)
        _apply_radar_json_headers(response, live=False)
        return {
            "symbol": symbol,
            "timestamp": _utc_iso(),
            "structural_confidence_score": None,
            "structural_bullish_score": None,
            "structural_bearish_score": None,
        }

    @router.get(
        "/political/{symbol}",
        response_model=RadarPoliticalPayload,
        responses=_RADAR_JSON_RESPONSES_200,
    )
    async def political(symbol: str, response: Response) -> Any:
        report = await _scanner_report_live()
        if report is not None:
            _apply_radar_json_headers(response, live=True)
            return build_political(symbol, report)
        _apply_radar_json_headers(response, live=False)
        return {
            "symbol": symbol,
            "timestamp": _utc_iso(),
            "aggregate_signal_score": None,
        }

    @router.get(
        "/sensors/camera/health",
        response_model=RadarCameraHealthPayload,
        responses=_RADAR_JSON_RESPONSES_200,
    )
    async def camera_health(response: Response) -> Any:
        env, live = await build_live_camera_envelope()
        _apply_radar_json_headers(response, live=live)
        return {"camera": env["camera"]}

    @router.get(
        "/camera/status",
        response_model=RadarCameraStatusPayload,
        responses=_RADAR_JSON_RESPONSES_200,
    )
    async def radar_camera_status(response: Response) -> Any:
        """Alias canónico Bloque 3: mismo cuerpo que health con metadatos ``ok`` / ``source``."""
        env, live = await build_live_camera_envelope()
        _apply_radar_json_headers(response, live=live)
        return env

    @router.get("/stream")
    async def radar_stream(symbol: str = "SPY") -> StreamingResponse:
        """SSE Bloque 4: envelopes consistentes (type, timestamp, symbol, source, sequence, data).

        Eventos: ``heartbeat``, ``snapshot_update``, ``camera_state_changed``,
        ``provider_state_changed``, ``degraded_state_changed``.
        Compat: se emite ``snapshot`` legado (solo ``symbol`` en data) para clientes antiguos.
        """
        stream_sym = (symbol or "SPY").strip().upper() or "SPY"
        live = radar_quant_http_enabled()

        async def _events() -> AsyncIterator[str]:
            yield ": atlas-radar-sse\n\n"
            yield "retry: 3000\n\n"
            seq = 0
            last_snap = time.monotonic() - _RADAR_SSE_SNAPSHOT_SEC
            last_cam_key: str | None = None
            last_provider_sig: str | None = None
            last_degraded_sig: str | None = None
            last_snapshot_sig: str | None = None
            last_report_ok = bool(live)

            def _next_seq() -> int:
                nonlocal seq
                seq += 1
                return seq

            def _source_label(quant_ok: bool) -> str:
                return "quant" if quant_ok else "stub"

            while True:
                now = time.monotonic()
                ts = _utc_iso()
                hb_env = {
                    "type": "heartbeat",
                    "timestamp": ts,
                    "symbol": stream_sym,
                    "source": _source_label(last_report_ok),
                    "sequence": _next_seq(),
                    "data": {
                        "heartbeat_interval_sec": _RADAR_SSE_HEARTBEAT_SEC,
                        "snapshot_interval_sec": _RADAR_SSE_SNAPSHOT_SEC,
                        "quant_reachable": last_report_ok,
                        "radar_quant_http_configured": live,
                    },
                }
                yield _sse_event_block(hb_env["sequence"], "heartbeat", hb_env)

                if now - last_snap >= _RADAR_SSE_SNAPSHOT_SEC:
                    last_snap = now
                    summary, summary_live, report = await radar_build_dashboard_for_sse(stream_sym)
                    last_report_ok = report is not None
                    sig = _radar_json_sig(
                        {
                            "sym": stream_sym,
                            "lu": summary.get("last_update"),
                            "cls": (summary.get("radar") or {})
                            .get("signal", {})
                            .get("meta", {})
                            .get("snapshot_classification"),
                            "stub": (summary.get("transport") or {}).get("stub"),
                            "gen": (report or {}).get("generated_at") if report else None,
                        }
                    )
                    if sig != last_snapshot_sig:
                        last_snapshot_sig = sig
                        snap_env = {
                            "type": "snapshot_update",
                            "timestamp": _utc_iso(),
                            "symbol": stream_sym,
                            "source": _source_label(last_report_ok),
                            "sequence": _next_seq(),
                            "data": {
                                "summary": summary,
                                "quant_reachable": last_report_ok,
                                "snapshot_signature": sig,
                            },
                        }
                        yield _sse_event_block(snap_env["sequence"], "snapshot_update", snap_env)
                    # Latido de refresco HTTP: cada ciclo aunque el resumen no cambie (compat + tablas dealer/etc.).
                    legacy = json.dumps({"type": "snapshot", "payload": {"symbol": stream_sym}}, separators=(",", ":"))
                    yield f"id: {_next_seq()}\nevent: snapshot\ndata: {legacy}\n\n"

                    try:
                        env, _cam_live = await build_live_camera_envelope()
                        cam = env.get("camera") if isinstance(env.get("camera"), dict) else {}
                        cam_key = json.dumps(cam, sort_keys=True, default=str)
                        if last_cam_key is None or cam_key != last_cam_key:
                            last_cam_key = cam_key
                            cam_env = {
                                "type": "camera_state_changed",
                                "timestamp": _utc_iso(),
                                "symbol": stream_sym,
                                "source": _source_label(last_report_ok),
                                "sequence": _next_seq(),
                                "data": {"camera": cam},
                            }
                            yield _sse_event_block(cam_env["sequence"], "camera_state_changed", cam_env)
                    except Exception:
                        logger.debug(
                            "radar_stream: error al construir envelope de cámara (no fata)",
                            exc_info=True,
                        )

                    if report is not None:
                        prov_body = build_providers_payload(report)
                    else:
                        prov_body = {
                            "providers": [
                                {
                                    "provider": "atlas_scanner (no desplegado en este host)",
                                    "name": "atlas_scanner (no desplegado en este host)",
                                    "is_ready": False,
                                    "ready": False,
                                    "stale_indicator": False,
                                    "latency_ms": 0,
                                    "p95_latency_ms": 0,
                                    "active_fallback_indicator": False,
                                    "circuit_open_indicator": False,
                                    "consecutive_errors": 0,
                                    "availability_ratio": 0.0,
                                    "last_error_type": "modo_demostracion_sin_motor",
                                    "ui_status_hint": "modo_demostracion",
                                }
                            ]
                        }
                    psig = _radar_json_sig(prov_body)
                    if psig != last_provider_sig:
                        last_provider_sig = psig
                        pv_env = {
                            "type": "provider_state_changed",
                            "timestamp": _utc_iso(),
                            "symbol": stream_sym,
                            "source": _source_label(last_report_ok),
                            "sequence": _next_seq(),
                            "data": {"providers_payload": prov_body, "signature": psig},
                        }
                        yield _sse_event_block(pv_env["sequence"], "provider_state_changed", pv_env)

                    cls = (
                        (summary.get("radar") or {})
                        .get("signal", {})
                        .get("meta", {})
                        .get("snapshot_classification")
                    )
                    stub_t = bool((summary.get("transport") or {}).get("stub"))
                    degraded = (not last_report_ok) or stub_t or (
                        str(cls or "")
                        in {
                            "operable_with_degradation",
                            "structural_only",
                            "non_operable",
                        }
                    )
                    d_sig = f"{cls}|{last_report_ok}|{stub_t}|{degraded}"
                    if d_sig != last_degraded_sig:
                        last_degraded_sig = d_sig
                        dg_env = {
                            "type": "degraded_state_changed",
                            "timestamp": _utc_iso(),
                            "symbol": stream_sym,
                            "source": _source_label(last_report_ok),
                            "sequence": _next_seq(),
                            "data": {
                                "snapshot_classification": cls,
                                "quant_reachable": last_report_ok,
                                "transport_stub": stub_t,
                                "operational_degraded": degraded,
                            },
                        }
                        yield _sse_event_block(dg_env["sequence"], "degraded_state_changed", dg_env)

                await asyncio.sleep(_RADAR_SSE_HEARTBEAT_SEC)

        hdrs = dict(_stream_headers(live=live))
        hdrs["Content-Type"] = "text/event-stream; charset=utf-8"
        return StreamingResponse(_events(), media_type="text/event-stream", headers=hdrs)

    return router


def build_radar_ui_router() -> APIRouter:
    router = APIRouter(tags=["Radar Dashboard"])

    @router.get("/radar/dashboard", response_class=HTMLResponse)
    async def radar_dashboard() -> HTMLResponse:
        path = _STATIC_RADAR / "dashboard.html"
        if not path.is_file():
            raise HTTPException(status_code=404, detail="dashboard_not_found")
        return HTMLResponse(
            path.read_text(encoding="utf-8"),
            headers={
                "Cache-Control": "no-cache, no-store, must-revalidate",
                "Pragma": "no-cache",
            },
        )

    @router.get("/radar/dashboard/assets/{asset_name}")
    async def radar_dashboard_assets(asset_name: str) -> FileResponse:
        safe = (asset_name or "").strip().replace("\\", "/")
        if "/" in safe or ".." in safe:
            raise HTTPException(status_code=400, detail="invalid_asset_path")
        path = _STATIC_RADAR / safe
        if not path.is_file():
            raise HTTPException(status_code=404, detail="asset_not_found")
        return FileResponse(
            path,
            headers={
                "Cache-Control": "no-cache, no-store, must-revalidate",
                "Pragma": "no-cache",
            },
        )

    return router
