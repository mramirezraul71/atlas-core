"""Payload estable para salud del sensor de visión/cámara (Quant API, sin streaming de vídeo)."""
from __future__ import annotations

import logging
import os
from typing import Any

from config.settings import settings

logger = logging.getLogger("quant.camera_health_payload")

try:
    from atlas_code_quant.vision.insta360_capture import InstaCapture
except ImportError:
    from vision.insta360_capture import InstaCapture  # type: ignore

_PROVIDER_LABELS: dict[str, str] = {
    "off": "Desactivado",
    "manual": "Manual",
    "desktop_capture": "Captura de escritorio",
    "direct_nexus": "NEXUS robot (directo)",
    "atlas_push_bridge": "Atlas PUSH bridge",
    "insta360": "Insta360 RTMP/USB",
}


def _insta360_check_detail(diagnose: dict[str, Any]) -> dict[str, Any]:
    for chk in diagnose.get("checks") or []:
        if isinstance(chk, dict) and chk.get("name") == "insta360":
            return chk
    return {}


def _classify_operational_state(
    *,
    enable_camera: bool,
    provider: str,
    provider_ready: bool,
    screen_integrity_ok: bool,
    reasons_not_ready: list[str],
    insta360_source: str | None,
) -> tuple[str, str | None, float]:
    """Devuelve (state, degradation_reason, availability_pct)."""
    if not enable_camera:
        return "disabled", None, 0.0
    if provider == "off":
        return "disabled", None, 0.0

    if provider_ready:
        if not screen_integrity_ok:
            return "degraded", "Integridad de pantalla marcada como no OK.", max(40.0, min(85.0, 55.0))
        return "ready", None, 100.0

    joined = " ".join(reasons_not_ready or "").lower()
    rtmp = bool((os.getenv("INSTA360_RTMP_URL") or "").strip())
    src = str(insta360_source or "none").strip().lower()

    if provider == "insta360" and not provider_ready:
        if rtmp:
            return (
                "unavailable",
                (reasons_not_ready[0] if reasons_not_ready else "Stream RTMP Insta360 no accesible."),
                0.0,
            )
        if src in {"", "none"}:
            return (
                "not_configured",
                "Insta360 sin INSTA360_RTMP_URL y sin fuente de captura USB/pipeline.",
                0.0,
            )
        return (
            "unavailable",
            (reasons_not_ready[0] if reasons_not_ready else "Fuente Insta360 no operativa."),
            0.0,
        )

    if "no responde" in joined or "hardware" in joined or "no disponible" in joined:
        return "unavailable", (reasons_not_ready[0] if reasons_not_ready else "Hardware o servicio no accesible."), 0.0

    return (
        "unavailable",
        (reasons_not_ready[0] if reasons_not_ready else "Proveedor de visión no listo."),
        0.0,
    )


def build_quant_camera_health_payload(vision: Any) -> dict[str, Any]:
    """Construye dict alineado con el contrato radar (sin depender de FastAPI)."""
    enable_camera = bool(getattr(settings, "enable_camera", False))
    diagnose: dict[str, Any]
    try:
        diagnose = vision.diagnose()
    except Exception as exc:
        return {
            "provider": "error",
            "state": "unavailable",
            "mode_detected": None,
            "backend": None,
            "device_index": None,
            "pnp_hints": [],
            "last_capture_ts": None,
            "presence": None,
            "activity": None,
            "availability_pct": 0.0,
            "degradation_reason": f"diagnose_error:{exc}",
            "notes": str(exc),
        }

    provider = str(diagnose.get("provider") or "unknown").strip().lower()
    provider_label = _PROVIDER_LABELS.get(provider, provider or "desconocido")
    if provider == "insta360" and (os.getenv("INSTA360_RTMP_URL") or "").strip():
        provider_label = "Insta360 RTMP"

    st_fast: dict[str, Any] = {}
    try:
        st_fast = vision.status(fast=True)
    except Exception:
        pass

    provider_ready = bool(diagnose.get("provider_ready"))
    screen_ok = bool(st_fast.get("screen_integrity_ok", True))
    reasons = list(diagnose.get("reasons_not_ready") or [])
    if not isinstance(reasons, list):
        reasons = []

    insta_detail = _insta360_check_detail(diagnose)
    insta_src = insta_detail.get("source")

    state, deg_reason, availability_pct = _classify_operational_state(
        enable_camera=enable_camera,
        provider=provider,
        provider_ready=provider_ready,
        screen_integrity_ok=screen_ok,
        reasons_not_ready=reasons,
        insta360_source=str(insta_src) if insta_src is not None else None,
    )

    last_cap = diagnose.get("last_capture_at") or st_fast.get("last_capture_at")
    if last_cap is not None:
        last_cap = str(last_cap)

    presence = st_fast.get("operator_present")
    if presence is None:
        presence = None
    else:
        presence = bool(presence)

    notes = str(st_fast.get("notes") or "").strip()
    activity = notes[:240] if notes else None
    src = insta_detail.get("source")
    if provider == "insta360" and src and src != "none":
        activity = (activity + " · " if activity else "") + f"fuente={src}"

    physical: dict[str, Any] = {
        "mode_detected": None,
        "backend": None,
        "device_index": None,
        "pnp_hints": [],
        "probe_notes": "",
    }
    if provider == "insta360" and enable_camera:
        try:
            physical = InstaCapture().build_health_snapshot(pnp_timeout_sec=2.5)
        except Exception as exc:
            logger.debug("InstaCapture health snapshot omitido: %s", exc)
            physical["probe_notes"] = f"snapshot_error:{exc}"

    notes_parts = [p for p in (notes, physical.get("probe_notes") or "") if p]
    merged_notes = " | ".join(notes_parts) if notes_parts else None

    return {
        "provider": provider_label,
        "state": state,
        "mode_detected": physical.get("mode_detected"),
        "backend": physical.get("backend"),
        "device_index": physical.get("device_index"),
        "pnp_hints": physical.get("pnp_hints") or [],
        "last_capture_ts": last_cap,
        "presence": presence,
        "activity": activity,
        "availability_pct": round(float(availability_pct), 2),
        "degradation_reason": deg_reason,
        "vision_provider": provider,
        "provider_ready": provider_ready,
        "notes": merged_notes,
    }
