"""
ATLAS DOCTOR — FastAPI Router
Endpoints REST para el Sistema Nervioso Central.

Prefijo: /doctor
Tags:    Doctor
"""
from __future__ import annotations

import time
from typing import Optional

from fastapi import APIRouter, Query
from fastapi.responses import JSONResponse

router = APIRouter(prefix="/doctor", tags=["Doctor"])

_t0_module = time.monotonic()


def _ms() -> int:
    return int((time.monotonic() - _t0_module) * 1000)


def _std(ok: bool, data=None, error: Optional[str] = None) -> dict:
    return {"ok": ok, "data": data, "ms": _ms(), "error": error}


def _doctor():
    from atlas_adapter.services.doctor_nervous_system import get_doctor
    return get_doctor()


# ── GET /doctor/status ─────────────────────────────────────────────────────────
@router.get("/status")
def doctor_status():
    """Estado actual del Sistema Nervioso: anomalías activas, último ciclo."""
    try:
        d = _doctor()
        return _std(True, d.status_report())
    except Exception as e:
        return _std(False, error=str(e))


# ── GET /doctor/state ──────────────────────────────────────────────────────────
@router.get("/state")
def doctor_state():
    """Estado completo: puertos, daemons, chromadb."""
    try:
        from modules.humanoid.healing.atlas_doctor_daemon import get_daemon_status
        daemon_st = get_daemon_status()
        doctor_st = _doctor().status_report()
        return _std(True, {"doctor": doctor_st, "daemon": daemon_st})
    except Exception as e:
        return _std(False, error=str(e))


# ── GET /doctor/anomalies ─────────────────────────────────────────────────────
@router.get("/anomalies")
def doctor_anomalies():
    """Lista las anomalías detectadas en el último ciclo."""
    try:
        d = _doctor()
        return _std(True, {
            "count": len(d.last_anomalies),
            "items": [
                {
                    "component": a.component,
                    "layer": a.layer,
                    "tier": a.tier,
                    "tier_label": f"TIER{a.tier}",
                    "description": a.description,
                    "context": a.context,
                    "ts": a.ts,
                }
                for a in d.last_anomalies
            ],
        })
    except Exception as e:
        return _std(False, error=str(e))


# ── GET /doctor/history ────────────────────────────────────────────────────────
@router.get("/history")
def doctor_history(limit: int = Query(50, ge=1, le=500)):
    """Historial de eventos del doctor (SQLite)."""
    try:
        d = _doctor()
        return _std(True, {"items": d.history(limit)})
    except Exception as e:
        return _std(False, error=str(e))


# ── POST /doctor/diagnose ─────────────────────────────────────────────────────
@router.post("/diagnose")
async def doctor_diagnose(
    include_self_inspection: bool = Query(
        True,
        description="Si true, espera inspección visual y dual (cámara + screenshot).",
    )
):
    """Fuerza un ciclo de diagnóstico completo ahora."""
    try:
        d = _doctor()
        result = await d.run_once(include_self_inspection=include_self_inspection)
        return _std(True, result)
    except Exception as e:
        return _std(False, error=str(e))


# ── POST /doctor/emergency ────────────────────────────────────────────────────
@router.post("/emergency")
def doctor_emergency():
    """Emergency stop: detiene Quant LiveLoop y emite alerta crítica."""
    try:
        d = _doctor()
        result = d.emergency_stop()
        return _std(True, result)
    except Exception as e:
        return _std(False, error=str(e))


# ── GET /doctor/ports ─────────────────────────────────────────────────────────
@router.get("/ports")
def doctor_ports():
    """Snapshot instantáneo de todos los puertos monitorizados."""
    import socket

    from modules.humanoid.healing.atlas_doctor_daemon import MONITORED_PORTS

    def _tcp(port: int) -> bool:
        try:
            with socket.create_connection(("127.0.0.1", port), timeout=0.8):
                return True
        except Exception:
            return False

    result = {}
    for port, (name, layer, desc) in MONITORED_PORTS.items():
        result[name] = {
            "port": port,
            "layer": layer,
            "description": desc,
            "up": _tcp(port),
        }
    return _std(True, result)


# ── GET /doctor/chromadb ──────────────────────────────────────────────────────
@router.get("/chromadb")
def doctor_chromadb():
    """Estado de las colecciones ChromaDB."""
    try:
        from modules.humanoid.healing.atlas_doctor_daemon import _check_chromadb
        return _std(True, _check_chromadb())
    except Exception as e:
        return _std(False, error=str(e))


def build_router() -> APIRouter:
    return router
