"""Switch active port: persist state, restart service. Policy + audit."""
from __future__ import annotations

import json
import os
import subprocess
import time
from pathlib import Path
from typing import Any, Dict, Optional


def _state_path() -> Path:
    base = Path(os.getenv("ATLAS_PUSH_ROOT", os.getcwd()))
    return base / "logs" / "deploy_state.json"


def _env_int(name: str, default: int) -> int:
    try:
        return int(os.getenv(name, str(default)) or default)
    except (TypeError, ValueError):
        return default


def get_deploy_state() -> Dict[str, Any]:
    """Read deploy_state.json. Returns mode, active_port, staging_port, last_deploy_ts, etc."""
    path = _state_path()
    default = {
        "mode": os.getenv("DEPLOY_MODE", "single").strip().lower(),
        "active_port": _env_int("ACTIVE_PORT", 8791),
        "staging_port": _env_int("STAGING_PORT", 8792),
        "last_deploy_ts": None,
        "last_switch_ts": None,
        "auto_switch_on_health": os.getenv("AUTO_SWITCH_ON_HEALTH", "true").strip().lower() in ("1", "true", "yes"),
    }
    if not path.exists():
        return default
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
        default.update(data)
        return default
    except Exception:
        return default


def persist_state(state: Dict[str, Any]) -> None:
    """Write deploy_state.json."""
    path = _state_path()
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(state, indent=2), encoding="utf-8")


def switch_active_port(new_active_port: int, service_name: Optional[str] = None) -> Dict[str, Any]:
    """
    Update deploy state so active_port = new_active_port, then restart Windows service.
    Returns {ok, message, error}. Audit logged.
    """
    t0 = time.perf_counter()
    try:
        from modules.humanoid.audit import get_audit_logger
        audit = get_audit_logger()
    except Exception:
        audit = None

    state = get_deploy_state()
    old_port = state.get("active_port", 8791)
    staging = state.get("staging_port", 8792)
    # After switch: active becomes new (was staging), staging becomes old
    state["active_port"] = new_active_port
    state["staging_port"] = old_port
    state["last_switch_ts"] = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
    persist_state(state)

    svc = service_name or os.getenv("SERVICE_NAME", "ATLAS_PUSH")
    if os.getenv("SERVICE_ENABLED", "").strip().lower() in ("1", "true", "yes"):
        try:
            subprocess.run(["net", "stop", svc], capture_output=True, timeout=15)
            time.sleep(2)
            subprocess.run(["net", "start", svc], capture_output=True, timeout=15)
        except Exception as e:
            ms = int((time.perf_counter() - t0) * 1000)
            if audit:
                audit.log_event("deploy", "switcher", "switch", False, ms, str(e), {"old": old_port, "new": new_active_port}, None)
            return {"ok": False, "message": "service restart failed", "error": str(e)}
    ms = int((time.perf_counter() - t0) * 1000)
    if audit:
        audit.log_event("deploy", "switcher", "switch", True, ms, None, {"old": old_port, "new": new_active_port}, None)
    return {"ok": True, "message": f"active_port={new_active_port}", "error": None}
