"""Rollback: checkout main, delete staging, optionally reset to previous commit."""
from __future__ import annotations

import logging
from typing import Any, Dict, Optional

from .git_manager import (
    checkout_branch,
    delete_branch,
    get_head_commit,
    get_remote_commit,
    reset_hard,
)

_log = logging.getLogger("humanoid.update.rollback")


def _env_str(name: str, default: str) -> str:
    import os
    return (os.getenv(name) or default).strip()


def _audit(module: str, action: str, ok: bool, payload: Optional[Dict] = None, error: Optional[str] = None) -> None:
    try:
        from modules.humanoid.audit import get_audit_logger
        get_audit_logger().log_event("update_engine", "system", module, action, ok, 0, error, payload, None)
    except Exception:
        pass


def rollback(
    main_branch: str = "main",
    staging_branch: str = "staging",
    remote: str = "origin",
    reset_to_remote: bool = True,
) -> Dict[str, Any]:
    """
    Checkout main, delete staging branch. If reset_to_remote, reset main to origin/main.
    Returns {ok, message, steps}.
    """
    steps = []
    try:
        r = checkout_branch(main_branch)
        steps.append({"step": "checkout_main", "ok": r.get("ok"), "error": r.get("error")})
        if not r.get("ok"):
            _audit("update", "rollback", False, {"steps": steps}, r.get("error"))
            return {"ok": False, "message": r.get("error"), "steps": steps}

        if reset_to_remote:
            rc = get_remote_commit(remote=remote, branch=main_branch)
            if rc.get("ok") and rc.get("commit"):
                r2 = reset_hard(rc["commit"])
                steps.append({"step": "reset_hard", "commit": rc["commit"], "ok": r2.get("ok"), "error": r2.get("error")})
                if not r2.get("ok"):
                    _audit("update", "rollback", False, {"steps": steps}, r2.get("error"))
                    return {"ok": False, "message": r2.get("error"), "steps": steps}

        r3 = delete_branch(staging_branch, force=True)
        steps.append({"step": "delete_staging", "ok": r3.get("ok"), "error": r3.get("error")})
        if not r3.get("ok"):
            _audit("update", "rollback", False, {"steps": steps}, r3.get("error"))
            return {"ok": False, "message": r3.get("error"), "steps": steps}

        _audit("update", "rollback", True, {"steps": steps})
        return {"ok": True, "message": "rollback completed", "steps": steps}
    except Exception as e:
        _log.exception("Rollback failed: %s", e)
        steps.append({"step": "exception", "ok": False, "error": str(e)})
        _audit("update", "rollback", False, {"steps": steps}, str(e))
        return {"ok": False, "message": str(e), "steps": steps}
