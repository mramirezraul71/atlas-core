"""Blue-green local: launch staging, smoke, healthcheck x3, switch or rollback. Policy + audit."""
from __future__ import annotations

import os
import subprocess
import time
from pathlib import Path
from typing import Any, Dict, Optional

from .healthcheck import run_health
from .switcher import get_deploy_state, switch_active_port, persist_state

_log = __import__("logging").getLogger("humanoid.deploy.bluegreen")


def _env_bool(name: str, default: bool) -> bool:
    v = os.getenv(name, "true" if default else "false").strip().lower()
    return v in ("1", "true", "yes", "y", "on")


def _env_int(name: str, default: int) -> int:
    try:
        return int(os.getenv(name, str(default)) or default)
    except (TypeError, ValueError):
        return default


def _audit(action: str, ok: bool, payload: Optional[Dict] = None, error: Optional[str] = None, ms: int = 0) -> None:
    try:
        from modules.humanoid.audit import get_audit_logger
        get_audit_logger().log_event("deploy", "bluegreen", action, ok, ms, error, payload, None)
    except Exception:
        pass


def _launch_staging_instance(port: int, timeout_sec: int = 30) -> Dict[str, Any]:
    """Start uvicorn in subprocess on port. Returns {ok, process, error}."""
    repo = Path(os.getenv("ATLAS_PUSH_ROOT", os.getcwd()))
    venv_python = repo / ".venv" / "Scripts" / "python.exe"
    if not venv_python.exists():
        venv_python = repo / ".venv" / "bin" / "python"
    if not venv_python.exists():
        return {"ok": False, "process": None, "error": "venv not found"}
    try:
        proc = subprocess.Popen(
            [str(venv_python), "-m", "uvicorn", "atlas_adapter.atlas_http_api:app", "--host", "127.0.0.1", "--port", str(port)],
            cwd=str(repo),
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
            env={**os.environ, "ATLAS_DEPLOY_STAGING": "1"},
        )
        # Wait for port to respond
        base = f"http://127.0.0.1:{port}"
        for _ in range(timeout_sec):
            time.sleep(1)
            h = run_health(base)
            if h.get("ok") or h.get("score", 0) > 0:
                return {"ok": True, "process": proc, "error": None}
        proc.terminate()
        proc.wait(timeout=5)
        return {"ok": False, "process": None, "error": "staging did not become healthy in time"}
    except Exception as e:
        return {"ok": False, "process": None, "error": str(e)}


def run_bluegreen_flow(actor: Any = None) -> Dict[str, Any]:
    """
    1) Launch staging on STAGING_PORT
    2) Smoke tests against staging
    3) Healthcheck 3 times
    4) If all OK: switch active port, stop old instance
    5) If fail: keep active, log rollback
    """
    t0 = time.perf_counter()
    if os.getenv("DEPLOY_MODE", "").strip().lower() != "bluegreen":
        return {"ok": False, "data": None, "ms": int((time.perf_counter() - t0) * 1000), "error": "DEPLOY_MODE != bluegreen"}

    state = get_deploy_state()
    active = state.get("active_port", _env_int("ACTIVE_PORT", 8791))
    staging_port = state.get("staging_port", _env_int("STAGING_PORT", 8792))
    auto_switch = state.get("auto_switch_on_health", _env_bool("AUTO_SWITCH_ON_HEALTH", True))
    steps = []

    # 1) Launch staging
    launch = _launch_staging_instance(staging_port)
    steps.append({"step": "launch_staging", "ok": launch.get("ok"), "error": launch.get("error")})
    if not launch.get("ok"):
        _audit("bluegreen_flow", False, {"steps": steps}, launch.get("error"), int((time.perf_counter() - t0) * 1000))
        return {"ok": False, "data": {"steps": steps}, "ms": int((time.perf_counter() - t0) * 1000), "error": launch.get("error")}

    proc = launch.get("process")
    try:
        base = f"http://127.0.0.1:{staging_port}"

        # 2) Smoke
        try:
            from modules.humanoid.update.smoke_runner import run_smoke
            smoke_result = run_smoke(port=staging_port, timeout_sec=120)
            smoke_ok = smoke_result.get("ok", False)
        except Exception as e:
            smoke_ok = False
            smoke_result = {"error": str(e)}
        steps.append({"step": "smoke", "ok": smoke_ok, "error": smoke_result.get("error")})
        if not smoke_ok:
            if proc:
                proc.terminate()
                try:
                    proc.wait(timeout=5)
                except Exception:
                    proc.kill()
            state["mode"] = "single"
            persist_state(state)
            _audit("deploy", "bluegreen_flow", False, {"steps": steps}, smoke_result.get("error"), int((time.perf_counter() - t0) * 1000))
            return {"ok": False, "data": {"steps": steps, "rollback": "smoke failed", "fallback": "single-port"}, "ms": int((time.perf_counter() - t0) * 1000), "error": "smoke failed"}

        # 3) Healthcheck 3x
        health_ok = True
        for i in range(3):
            h = run_health(base)
            if not h.get("ok") or h.get("score", 0) < 60:
                health_ok = False
                steps.append({"step": f"health_{i+1}", "ok": False, "score": h.get("score")})
                break
            steps.append({"step": f"health_{i+1}", "ok": True, "score": h.get("score")})
            time.sleep(1)
        if not health_ok:
            if proc:
                proc.terminate()
                try:
                    proc.wait(timeout=5)
                except Exception:
                    proc.kill()
            state["mode"] = "single"
            persist_state(state)
            _audit("deploy", "bluegreen_flow", False, {"steps": steps}, "health < 60", int((time.perf_counter() - t0) * 1000))
            return {"ok": False, "data": {"steps": steps, "rollback": "health failed", "fallback": "single-port"}, "ms": int((time.perf_counter() - t0) * 1000), "error": "health check failed"}

        # 4) Switch
        if not auto_switch:
            if proc:
                proc.terminate()
                try:
                    proc.wait(timeout=5)
                except Exception:
                    proc.kill()
            return {"ok": True, "data": {"steps": steps, "switched": False, "message": "AUTO_SWITCH_ON_HEALTH=false"}, "ms": int((time.perf_counter() - t0) * 1000), "error": None}

        switch_result = switch_active_port(staging_port)
        steps.append({"step": "switch", "ok": switch_result.get("ok"), "error": switch_result.get("error")})
        state = get_deploy_state()
        state["last_deploy_ts"] = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
        persist_state(state)

        if proc:
            proc.terminate()
            try:
                proc.wait(timeout=5)
            except Exception:
                proc.kill()

        ms = int((time.perf_counter() - t0) * 1000)
        _audit("deploy", "bluegreen_flow", switch_result.get("ok", False), {"steps": steps}, switch_result.get("error"), ms)
        return {"ok": switch_result.get("ok", False), "data": {"steps": steps, "switched": True}, "ms": ms, "error": switch_result.get("error")}
    except Exception as e:
        if proc:
            try:
                proc.terminate()
                proc.wait(timeout=5)
            except Exception:
                pass
        _audit("deploy", "bluegreen_flow", False, {"steps": steps}, str(e), int((time.perf_counter() - t0) * 1000))
        return {"ok": False, "data": {"steps": steps}, "ms": int((time.perf_counter() - t0) * 1000), "error": str(e)}
