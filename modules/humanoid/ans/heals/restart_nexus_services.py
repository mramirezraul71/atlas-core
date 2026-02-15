"""Reinicia NEXUS (8000) y Robot backend (8002) cuando no responden."""
from __future__ import annotations

import os
import sys
import subprocess
from pathlib import Path
from .base import heal_result

_BASE = Path(__file__).resolve().parent.parent.parent.parent.parent
SCRIPT_PATH = _BASE / "scripts" / "start_nexus_services.py"


def run(**kwargs) -> dict:
    if not SCRIPT_PATH.exists():
        return heal_result(False, "restart_nexus_services", f"Script no encontrado: {SCRIPT_PATH}", {})
    try:
        from modules.humanoid.ans.live_stream import emit
        emit("heal_attempt", heal_id="restart_nexus_services", message="Iniciando NEXUS y Robot")
    except Exception:
        pass
    try:
        py = os.getenv("PYTHON", "python")
        flags = subprocess.CREATE_NO_WINDOW if sys.platform == "win32" and hasattr(subprocess, "CREATE_NO_WINDOW") else 0
        r = subprocess.run(
            [py, str(SCRIPT_PATH)],
            capture_output=True,
            timeout=30,
            text=True,
            encoding="utf-8",
            errors="replace",
            creationflags=flags,
        )
        out = (r.stdout or "").strip() or (r.stderr or "")[:150]
        ok = "NEXUS" in out or "Robot" in out
        msg = f"Iniciados: {out}" if ok else f"No se pudo iniciar: {out or r.stderr or 'sin salida'}"
        try:
            from modules.humanoid.ans.live_stream import emit
            emit("heal_end", heal_id="restart_nexus_services", ok=ok, message=msg[:80])
        except Exception:
            pass
        return heal_result(ok, "restart_nexus_services", msg, {"output": out})
    except subprocess.TimeoutExpired:
        return heal_result(False, "restart_nexus_services", "Timeout 30s", {})
    except Exception as e:
        return heal_result(False, "restart_nexus_services", str(e), {}, str(e))
