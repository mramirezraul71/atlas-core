"""Institutional Radar: UI estática + API stub en PUSH (:8791).

El paquete histórico `atlas_scanner` no está presente en el árbol actual; este
módulo sirve el dashboard y respuestas JSON mínimas para que la SPA cargue y
haga polling sin 404. Sustituir por el router real cuando se restaure el motor.
"""

from __future__ import annotations

import asyncio
import json
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, AsyncIterator

from fastapi import APIRouter, HTTPException
from fastapi.responses import FileResponse, HTMLResponse, JSONResponse, StreamingResponse

# Evita que el navegador sirva JSON antiguo (p. ej. snapshot_classification ya corregida).
_NOCACHE_JSON_HEADERS = {
    "Cache-Control": "no-store, max-age=0, must-revalidate",
    "Pragma": "no-cache",
    "X-Atlas-Radar-Stub": "1",
}


def _json_nocache(body: Any) -> JSONResponse:
    return JSONResponse(content=body, headers=dict(_NOCACHE_JSON_HEADERS))

_ROUTER_DIR = Path(__file__).resolve().parent
_ADAPTER_DIR = _ROUTER_DIR.parent
_STATIC_RADAR = _ADAPTER_DIR / "static" / "radar"


def _utc_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


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
    }


def build_radar_stub_api_router() -> APIRouter:
    router = APIRouter(prefix="/api/radar", tags=["Radar"])

    @router.get("/dashboard/summary")
    async def dashboard_summary(symbol: str = "SPY") -> JSONResponse:
        sym = (symbol or "SPY").strip().upper() or "SPY"
        return _json_nocache(_stub_summary(sym))

    @router.get("/diagnostics/providers")
    async def diagnostics_providers() -> JSONResponse:
        return _json_nocache({
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
        })

    @router.get("/decisions/recent")
    async def decisions_recent(limit: int = 40) -> JSONResponse:  # noqa: ARG001
        return _json_nocache({"recent": []})

    @router.get("/decisions/stats")
    async def decisions_stats() -> JSONResponse:
        return _json_nocache({
            "stats": {
                "by_decision": {"accepted": 0, "rejected": 0},
                "avg_fast_pressure_score": None,
                "avg_structural_confidence_score": None,
            }
        })

    @router.get("/dealer/{symbol}")
    async def dealer(symbol: str) -> JSONResponse:
        return _json_nocache({
            "symbol": symbol,
            "timestamp": _utc_iso(),
            "gamma_flip_level": "-",
            "dealer_skew_score": None,
            "call_wall": "-",
            "put_wall": "-",
            "acceleration_zone_score": None,
        })

    @router.get("/diagnostics/fast/{symbol}")
    async def fast_diag(symbol: str) -> JSONResponse:
        return _json_nocache({
            "symbol": symbol,
            "timestamp": _utc_iso(),
            "fast_pressure_score": None,
            "fast_risk_score": None,
            "fast_directional_bias_score": None,
        })

    @router.get("/diagnostics/structural/{symbol}")
    async def structural_diag(symbol: str) -> JSONResponse:
        return _json_nocache({
            "symbol": symbol,
            "timestamp": _utc_iso(),
            "structural_confidence_score": None,
            "structural_bullish_score": None,
            "structural_bearish_score": None,
        })

    @router.get("/political/{symbol}")
    async def political(symbol: str) -> JSONResponse:
        return _json_nocache({
            "symbol": symbol,
            "timestamp": _utc_iso(),
            "aggregate_signal_score": None,
        })

    @router.get("/sensors/camera/health")
    async def camera_health() -> JSONResponse:
        return _json_nocache(_stub_camera())

    @router.get("/stream")
    async def radar_stream() -> StreamingResponse:
        """SSE mínimo: latidos en vivo + instantáneas periódicas para refrescar métricas en la UI."""

        async def _events() -> AsyncIterator[str]:
            n = 0
            while True:
                hb = json.dumps({"type": "heartbeat", "payload": {}})
                yield f"event: heartbeat\ndata: {hb}\n\n"
                n += 1
                if n % 3 == 0:
                    snap = json.dumps({"type": "snapshot", "payload": {"symbol": "SPY"}})
                    yield f"event: snapshot\ndata: {snap}\n\n"
                await asyncio.sleep(8)

        return StreamingResponse(
            _events(),
            media_type="text/event-stream",
            headers={
                "Cache-Control": "no-cache",
                "Connection": "keep-alive",
                "X-Accel-Buffering": "no",
            },
        )

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
