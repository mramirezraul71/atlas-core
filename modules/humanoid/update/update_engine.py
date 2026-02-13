"""Git-based update engine: fetch, staging, smoke, promote or rollback. Policy + audit. Windows-style update window."""
from __future__ import annotations

import logging
import os
import time
from datetime import datetime, timezone
from typing import Any, Dict, Optional

from .git_manager import (
    checkout_branch,
    create_staging_branch,
    delete_branch,
    fetch,
    get_current_branch,
    get_diff,
    get_head_commit,
    get_remote_commit,
    get_status,
    merge_staging,
)
from .rollback import rollback as do_rollback
from .smoke_runner import run_smoke

_log = logging.getLogger("humanoid.update_engine")


def _env_bool(name: str, default: bool) -> bool:
    v = os.getenv(name, "true" if default else "false").strip().lower()
    return v in ("1", "true", "yes", "y", "on")


def _env_str(name: str, default: str) -> str:
    return (os.getenv(name) or default).strip()


def _audit(module: str, action: str, ok: bool, payload: Optional[Dict] = None, error: Optional[str] = None, ms: int = 0) -> None:
    try:
        from modules.humanoid.audit import get_audit_logger
        get_audit_logger().log_event("update_engine", "system", module, action, ok, ms, error, payload, None)
    except Exception:
        pass


def update_enabled() -> bool:
    return _env_bool("UPDATE_ENABLED", True)


def _in_update_window() -> bool:
    """Windows-style: solo permite apply dentro de la ventana configurada."""
    start = _env_str("UPDATE_WINDOW_START", "01:00")
    end = _env_str("UPDATE_WINDOW_END", "04:00")
    now = datetime.now(timezone.utc)
    try:
        h, m = map(int, start.split(":"))
        t_start = h * 60 + m
        h2, m2 = map(int, end.split(":"))
        t_end = h2 * 60 + m2
        t_now = now.hour * 60 + now.minute
        if t_start <= t_end:
            return t_start <= t_now <= t_end
        return t_now >= t_start or t_now <= t_end
    except Exception:
        return True


def status() -> Dict[str, Any]:
    """Current update status: branch, head, remote, has_update, config."""
    t0 = time.perf_counter()
    out: Dict[str, Any] = {
        "ok": True,
        "enabled": update_enabled(),
        "branch": None,
        "head_commit": None,
        "remote_commit": None,
        "has_update": False,
        "config": {
            "remote": _env_str("UPDATE_REMOTE", "origin"),
            "branch": _env_str("UPDATE_BRANCH", "main"),
            "staging_branch": _env_str("UPDATE_STAGING_BRANCH", "staging"),
            "require_smoke": _env_bool("UPDATE_REQUIRE_SMOKE", True),
            "auto_promote": _env_bool("UPDATE_AUTO_PROMOTE", False),
            "allow_rollback": _env_bool("UPDATE_ALLOW_ROLLBACK", True),
            "update_window_start": _env_str("UPDATE_WINDOW_START", "01:00"),
            "update_window_end": _env_str("UPDATE_WINDOW_END", "04:00"),
            "in_update_window": _in_update_window(),
        },
        "ms": 0,
        "error": None,
    }
    if not update_enabled():
        out["ms"] = int((time.perf_counter() - t0) * 1000)
        return out
    remote = out["config"]["remote"]
    branch = out["config"]["branch"]
    r_fetch = fetch(remote)
    if not r_fetch.get("ok"):
        out["ok"] = False
        out["error"] = r_fetch.get("error")
        out["ms"] = int((time.perf_counter() - t0) * 1000)
        return out
    cur = get_current_branch()
    head = get_head_commit()
    rem = get_remote_commit(remote=remote, branch=branch)
    out["branch"] = cur.get("branch")
    out["head_commit"] = head.get("commit")
    out["remote_commit"] = rem.get("commit")
    if head.get("commit") and rem.get("commit"):
        out["has_update"] = head["commit"] != rem["commit"]
    out["ms"] = int((time.perf_counter() - t0) * 1000)
    _audit("update", "status", out["ok"], {"has_update": out["has_update"]}, out.get("error"), out["ms"])
    return out


def check(actor: Any = None) -> Dict[str, Any]:
    """Update check: fetch, snapshot (status+diff), return status + plan. No apply."""
    t0 = time.perf_counter()
    if not update_enabled():
        return {"ok": False, "data": None, "ms": int((time.perf_counter() - t0) * 1000), "error": "UPDATE_ENABLED=false"}
    st = status()
    if not st.get("ok"):
        return {"ok": False, "data": st, "ms": st.get("ms", 0), "error": st.get("error")}
    snapshot = {}
    r_status = get_status()
    r_diff = get_diff()
    snapshot["status_output"] = r_status.get("output", "")
    snapshot["diff_output"] = (r_diff.get("output") or "")[:4096]
    data = {
        **st,
        "snapshot": snapshot,
    }
    ms = int((time.perf_counter() - t0) * 1000)
    _audit("update", "check", True, {"has_update": st.get("has_update")}, None, ms)
    return {"ok": True, "data": data, "ms": ms, "error": None}


def apply(
    require_smoke: Optional[bool] = None,
    auto_promote: Optional[bool] = None,
    actor: Any = None,
) -> Dict[str, Any]:
    """
    Full update flow: fetch -> checkout staging -> run smoke -> promote or rollback.
    Does not install packages (use updater.apply for that). Only git-based update.
    """
    t0 = time.perf_counter()
    if not update_enabled():
        return {"ok": False, "data": None, "ms": int((time.perf_counter() - t0) * 1000), "error": "UPDATE_ENABLED=false"}
    policy_allow = os.getenv("POLICY_ALLOW_UPDATE_APPLY", "false").strip().lower() in ("1", "true", "yes")
    if not policy_allow:
        return {"ok": False, "data": None, "ms": int((time.perf_counter() - t0) * 1000), "error": "POLICY_ALLOW_UPDATE_APPLY=false"}
    update_window_required = _env_bool("UPDATE_REQUIRE_WINDOW", False)
    if update_window_required and not _in_update_window():
        return {"ok": False, "data": None, "ms": int((time.perf_counter() - t0) * 1000), "error": "outside update window"}
    remote = _env_str("UPDATE_REMOTE", "origin")
    branch = _env_str("UPDATE_BRANCH", "main")
    staging_name = _env_str("UPDATE_STAGING_BRANCH", "staging")
    require_smoke = require_smoke if require_smoke is not None else _env_bool("UPDATE_REQUIRE_SMOKE", True)
    allow_rollback = _env_bool("UPDATE_ALLOW_ROLLBACK", True)

    steps = []
    # 1) Fetch
    r_fetch = fetch(remote)
    steps.append({"step": "fetch", "ok": r_fetch.get("ok"), "error": r_fetch.get("error")})
    if not r_fetch.get("ok"):
        _audit("update", "apply", False, {"steps": steps}, r_fetch.get("error"))
        return {"ok": False, "data": {"steps": steps}, "ms": int((time.perf_counter() - t0) * 1000), "error": r_fetch.get("error")}

    # 2) Create staging from origin/main
    r_staging = create_staging_branch(remote=remote, branch=branch, staging_name=staging_name)
    steps.append({"step": "create_staging", "ok": r_staging.get("ok"), "error": r_staging.get("error")})
    if not r_staging.get("ok"):
        _audit("update", "apply", False, {"steps": steps}, r_staging.get("error"))
        return {"ok": False, "data": {"steps": steps}, "ms": int((time.perf_counter() - t0) * 1000), "error": r_staging.get("error")}

    # 3) Smoke
    smoke_ok = True
    if require_smoke:
        smoke_result = run_smoke(timeout_sec=120)
        smoke_ok = smoke_result.get("ok", False)
        steps.append({"step": "smoke", "ok": smoke_ok, "returncode": smoke_result.get("returncode"), "error": smoke_result.get("error")})
        _audit("update", "smoke_run", smoke_ok, {"returncode": smoke_result.get("returncode")}, smoke_result.get("error"), smoke_result.get("ms", 0))

    if smoke_ok:
        # 4a) Promote: checkout main, merge staging
        r_main = checkout_branch(branch)
        steps.append({"step": "checkout_main", "ok": r_main.get("ok"), "error": r_main.get("error")})
        if not r_main.get("ok"):
            _audit("update", "apply", False, {"steps": steps}, r_main.get("error"))
            return {"ok": False, "data": {"steps": steps}, "ms": int((time.perf_counter() - t0) * 1000), "error": r_main.get("error")}
        r_merge = merge_staging(staging_name=staging_name)
        steps.append({"step": "merge_staging", "ok": r_merge.get("ok"), "error": r_merge.get("error")})
        if not r_merge.get("ok"):
            _audit("update", "apply", False, {"steps": steps}, r_merge.get("error"))
            return {"ok": False, "data": {"steps": steps}, "ms": int((time.perf_counter() - t0) * 1000), "error": r_merge.get("error")}
        delete_branch(staging_name, force=True)
        _audit("update", "apply", True, {"steps": steps}, None, int((time.perf_counter() - t0) * 1000))
        return {"ok": True, "data": {"steps": steps, "promoted": True}, "ms": int((time.perf_counter() - t0) * 1000), "error": None}
    else:
        # 4b) Rollback
        if allow_rollback:
            r_rollback = do_rollback(main_branch=branch, staging_branch=staging_name, remote=remote, reset_to_remote=True)
            steps.append({"step": "rollback", "ok": r_rollback.get("ok"), "message": r_rollback.get("message")})
            _audit("update", "apply", False, {"steps": steps, "rollback": True}, r_rollback.get("message"))
            return {"ok": False, "data": {"steps": steps, "rollback": True}, "ms": int((time.perf_counter() - t0) * 1000), "error": "smoke failed; rollback executed"}
        _audit("update", "apply", False, {"steps": steps}, "smoke failed")
        return {"ok": False, "data": {"steps": steps}, "ms": int((time.perf_counter() - t0) * 1000), "error": "smoke failed"}
