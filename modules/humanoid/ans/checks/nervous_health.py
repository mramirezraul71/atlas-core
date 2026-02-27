"""Nervous system health check: score 0-100 basado en sensores + bitácora.

Este check conecta el Sistema Nervioso con ANS:
- Si score cae, crea incidente y puede sugerir heals.
"""
from __future__ import annotations

import os


def run() -> dict:
    try:
        from modules.humanoid.nervous.engine import get_nervous_status
        data = get_nervous_status(limit=80) or {}
        score = int(data.get("score", 100) or 0)
        ok_thr = int(os.getenv("NERVOUS_OK_THRESHOLD", "75") or 75)
        ok = score >= ok_thr
        # severidad por envergadura
        if score < 40:
            sev = "critical"
        elif score < 55:
            sev = "high"
        elif score < ok_thr:
            sev = "med"
        else:
            sev = "low"

        signals = data.get("signals") or []
        # Señales de la DB: no tienen "ok", son penalizaciones. Derivamos failing por severidad/puntos.
        failing = [s for s in signals if int(s.get("points", 0) or 0) > 0]
        top = failing[:5]
        heals = []
        # heurística: si nexus_services_health falla -> restart_nexus_services
        if any(s.get("sensor_id") == "nexus_services_health" for s in failing):
            heals.append("restart_nexus_services")
        if any(s.get("sensor_id") == "api_health" for s in failing):
            heals.extend(["clear_stale_locks", "restart_scheduler"])
        if any(str(s.get("sensor_id") or "").startswith("metrics_latency:") for s in failing):
            heals.append("rotate_logs")

        return {
            "ok": ok,
            "check_id": "nervous_health",
            "message": f"nervous_score={score} thr={ok_thr}",
            "details": {"score": score, "threshold": ok_thr, "top": top},
            "severity": sev,
            "suggested_heals": list(dict.fromkeys(heals)) if not ok else [],
        }
    except Exception as e:
        return {"ok": False, "check_id": "nervous_health", "message": str(e), "details": {"error": str(e)}, "severity": "med", "suggested_heals": []}

