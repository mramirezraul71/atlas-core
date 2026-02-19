"""Evolution health: ATLAS_EVOLUTION como proceso crítico del ANS. Lee último ciclo desde logs/evolution_last_cycle.json."""
from __future__ import annotations

import os
from pathlib import Path
from datetime import datetime, timezone, timedelta


def run() -> dict:
    """OK si el daemon reportó un ciclo exitoso reciente (< 13h)."""
    try:
        base = Path(os.environ.get("ATLAS_BASE", os.getcwd()))
        if not base.is_dir():
            base = Path(__file__).resolve().parent.parent.parent.parent.parent
        state_file = base / "logs" / "evolution_last_cycle.json"
        if not state_file.exists():
            return {
                "ok": True,
                "check_id": "evolution_health",
                "message": "Daemon Evolution no ha reportado aún (ejecutar atlas_evolution.py)",
                "details": {"state_file": str(state_file), "status": "pending"},
                "severity": "low",
                "suggested_heals": [],
            }
        import json
        data = json.loads(state_file.read_text(encoding="utf-8"))
        ts_str = data.get("ts", "")
        cycle_ok = data.get("ok", False)
        try:
            ts = datetime.fromisoformat(ts_str.replace("Z", "+00:00"))
        except Exception:
            ts = None
        now = datetime.now(timezone.utc)
        # Umbral 24h: el daemon de evolución corre tipicamente 1x/día; 13h generaba falsos positivos
        stale = ts is None or (now - ts) > timedelta(hours=24)
        ok = cycle_ok and not stale
        message = data.get("message", "Asimilación Exitosa" if cycle_ok else "Ciclo con errores")
        if stale:
            message = "Último ciclo Evolution hace >24h. Comprobar daemon atlas_evolution.py"
        return {
            "ok": ok,
            "check_id": "evolution_health",
            "message": message,
            "details": {"ts": ts_str, "cycle_ok": cycle_ok, "stale": stale},
            "severity": "low" if ok else "med",
            "suggested_heals": [],
        }
    except Exception as e:
        return {"ok": False, "check_id": "evolution_health", "message": str(e)[:200], "details": {"error": str(e)}, "severity": "low", "suggested_heals": []}
