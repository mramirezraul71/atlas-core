"""Autonomous Reactor: detects recurring failures and takes corrective action.

Unlike the ANS cycle (which runs checks and heals known issues), the Reactor
monitors the Lifelog and Selfcheck for PATTERNS of repeated failures and
executes targeted repairs, then reports results to the Bitácora.
"""
from __future__ import annotations

import logging
import os
import threading
import time
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional

log = logging.getLogger("atlas.reactor")

_REACTOR_RUNNING = False
_REACTOR_THREAD: Optional[threading.Thread] = None

REACTION_REGISTRY: Dict[str, dict] = {
    "fix_logs_dir": {
        "pattern": "Cannot create logs dir",
        "description": "Crear directorio de logs faltante",
        "severity": "critical",
    },
    "fix_config_missing": {
        "pattern": "config/atlas.env not found",
        "description": "Crear config/atlas.env desde template",
        "severity": "warning",
    },
    "fix_approvals_404": {
        "pattern": "HTTP 404",
        "source_match": "check_approvals",
        "description": "Endpoint de aprobaciones no disponible",
        "severity": "warning",
    },
    "fix_service_down": {
        "pattern": "Connection refused",
        "description": "Reiniciar servicio caído",
        "severity": "critical",
    },
}


def _emit_bitacora(message: str, ok: bool = True, source: str = "reactor"):
    """Report to both evolution log and lifelog."""
    try:
        from modules.humanoid.ans.live_stream import emit
        emit("reactor", message=message, ok=ok)
    except Exception:
        pass
    try:
        from modules.humanoid.memory_engine.lifelog import get_lifelog
        ll = get_lifelog()
        ll.log(
            event_type="reactor_action", source=source,
            perception=message,
            outcome="fixed" if ok else "failed",
            success=ok, importance=0.8 if not ok else 0.5,
        )
    except Exception:
        pass


def _fix_logs_dir() -> dict:
    """Create missing logs directory."""
    repo = Path(os.getenv("ATLAS_REPO_PATH", "C:\\ATLAS_PUSH"))
    logs_dir = repo / "logs"
    try:
        logs_dir.mkdir(parents=True, exist_ok=True)
        msg = f"Directorio de logs creado: {logs_dir}"
        _emit_bitacora(msg, ok=True)
        return {"ok": True, "action": "mkdir_logs", "message": msg}
    except Exception as e:
        msg = f"No pude crear directorio de logs: {e}"
        _emit_bitacora(msg, ok=False)
        return {"ok": False, "action": "mkdir_logs", "message": msg}


def _fix_config_missing() -> dict:
    """Create atlas.env from example or with minimal content."""
    repo = Path(os.getenv("ATLAS_REPO_PATH", "C:\\ATLAS_PUSH"))
    config_dir = repo / "config"
    env_file = config_dir / "atlas.env"
    example = config_dir / "atlas.env.example"
    try:
        config_dir.mkdir(parents=True, exist_ok=True)
        if example.exists() and not env_file.exists():
            env_file.write_text(example.read_text(encoding="utf-8"), encoding="utf-8")
            msg = "config/atlas.env creado desde template"
        elif not env_file.exists():
            env_file.write_text("# ATLAS config - auto-generated\nATLAS_REPO_PATH=C:\\ATLAS_PUSH\n", encoding="utf-8")
            msg = "config/atlas.env creado con defaults"
        else:
            msg = "config/atlas.env ya existe"
        _emit_bitacora(msg, ok=True)
        return {"ok": True, "action": "create_config", "message": msg}
    except Exception as e:
        msg = f"No pude crear config: {e}"
        _emit_bitacora(msg, ok=False)
        return {"ok": False, "action": "create_config", "message": msg}


def _fix_service_restart(service_port: int = 8791) -> dict:
    """Attempt to verify and report service status."""
    try:
        import requests
        r = requests.get(f"http://127.0.0.1:{service_port}/health", timeout=5)
        if r.status_code == 200:
            msg = f"Servicio en puerto {service_port} verificado: operativo"
            _emit_bitacora(msg, ok=True)
            return {"ok": True, "action": "verify_service", "message": msg}
    except Exception:
        pass
    msg = f"Servicio en puerto {service_port} no responde"
    _emit_bitacora(msg, ok=False)
    return {"ok": False, "action": "verify_service", "message": msg}


def _get_recent_failures() -> List[dict]:
    """Get failures from the last hour."""
    failures = []
    try:
        from modules.humanoid.memory_engine.lifelog import get_lifelog
        ll = get_lifelog()
        recent = ll.get_failures(limit=50)
        failures.extend(recent)
    except Exception:
        pass
    return failures


def _get_selfcheck_problems() -> List[dict]:
    """Get current selfcheck problems."""
    try:
        from modules.humanoid.product.selfcheck import run_selfcheck
        result = run_selfcheck()
        return result.get("problems", [])
    except Exception:
        return []


def run_reaction_cycle() -> Dict[str, Any]:
    """Execute one reaction cycle: detect patterns -> diagnose -> fix -> report."""
    t0 = time.perf_counter()
    actions_taken = []
    issues_found = []

    problems = _get_selfcheck_problems()
    for p in problems:
        pid = p.get("id", "")
        msg = p.get("message", "")
        severity = p.get("severity", "info")
        issues_found.append({"source": "selfcheck", "id": pid, "message": msg, "severity": severity})

        if pid == "logs_dir" or "logs dir" in msg.lower():
            result = _fix_logs_dir()
            actions_taken.append(result)
        elif pid == "config_missing":
            result = _fix_config_missing()
            actions_taken.append(result)

    failures = _get_recent_failures()
    failure_counts: Dict[str, int] = {}
    for f in failures:
        key = f.get("source", "") + ":" + (f.get("perception", "") or "")[:60]
        failure_counts[key] = failure_counts.get(key, 0) + 1

    for key, count in failure_counts.items():
        if count >= 3:
            issues_found.append({
                "source": "lifelog_pattern",
                "id": f"recurring_{key.split(':')[0]}",
                "message": f"Fallo recurrente ({count}x): {key}",
                "severity": "high",
            })

    successful_fixes = sum(1 for a in actions_taken if a.get("ok"))
    failed_fixes = sum(1 for a in actions_taken if not a.get("ok"))

    ms = int((time.perf_counter() - t0) * 1000)

    if actions_taken:
        summary = f"Reactor: {len(issues_found)} issues, {successful_fixes} reparados, {failed_fixes} fallidos ({ms}ms)"
        _emit_bitacora(summary, ok=failed_fixes == 0, source="reactor")
        log.info(summary)

    return {
        "ok": True,
        "issues_found": len(issues_found),
        "issues": issues_found,
        "actions_taken": actions_taken,
        "fixes_ok": successful_fixes,
        "fixes_failed": failed_fixes,
        "ms": ms,
    }


def _reactor_loop():
    """Background loop that runs reaction cycles periodically."""
    global _REACTOR_RUNNING
    interval = int(os.getenv("REACTOR_INTERVAL_SECONDS", "120"))
    log.info("Reactor autonomo iniciado (intervalo: %ds)", interval)
    _emit_bitacora(f"Reactor autonomo activado (cada {interval}s)", ok=True)

    time.sleep(10)

    while _REACTOR_RUNNING:
        try:
            result = run_reaction_cycle()
            if result.get("issues_found", 0) > 0:
                log.info("Reactor cycle: %d issues, %d fixed", result["issues_found"], result["fixes_ok"])
        except Exception as e:
            log.error("Reactor cycle error: %s", e)
        time.sleep(interval)


def start_reactor():
    """Start the autonomous reactor in a background thread."""
    global _REACTOR_RUNNING, _REACTOR_THREAD
    if _REACTOR_RUNNING:
        return {"ok": True, "message": "Reactor ya corriendo"}
    _REACTOR_RUNNING = True
    _REACTOR_THREAD = threading.Thread(target=_reactor_loop, daemon=True, name="atlas-reactor")
    _REACTOR_THREAD.start()
    return {"ok": True, "message": "Reactor iniciado"}


def stop_reactor():
    """Stop the autonomous reactor."""
    global _REACTOR_RUNNING
    _REACTOR_RUNNING = False
    return {"ok": True, "message": "Reactor detenido"}
