"""Run quick diagnostics; return problems and suggestions. Used by GET /support/selfcheck and service startup."""
from __future__ import annotations

import os
from pathlib import Path
from typing import Any, Dict, List


def _repo_root() -> Path:
    p = os.getenv("ATLAS_REPO_PATH") or os.getenv("POLICY_ALLOWED_PATHS") or "C:\\ATLAS_PUSH"
    return Path(p).resolve()


def run_selfcheck() -> Dict[str, Any]:
    """
    Run fast diagnostics. Returns {ok, problems: [{id, severity, message, suggestion}], suggestions: [], ms}.
    Severity: critical | warning | info.
    """
    import time
    t0 = time.perf_counter()
    problems: List[Dict[str, Any]] = []
    suggestions: List[str] = []

    root = _repo_root()
    env_file = root / "config" / "atlas.env"
    if not env_file.exists():
        problems.append({"id": "config_missing", "severity": "warning", "message": "config/atlas.env not found", "suggestion": "Copy config/atlas.env.example to config/atlas.env"})
    else:
        suggestions.append("Config file present")

    logs_dir = root / "logs"
    if not logs_dir.exists():
        try:
            logs_dir.mkdir(parents=True, exist_ok=True)
        except Exception as e:
            problems.append({"id": "logs_dir", "severity": "critical", "message": f"Cannot create logs dir: {e}", "suggestion": "Create C:\\ATLAS_PUSH\\logs with write permissions"})

    try:
        from modules.humanoid.audit import get_audit_logger
        logger = get_audit_logger()
        if getattr(logger, "_db", None) is None:
            problems.append({"id": "audit_db", "severity": "warning", "message": "Audit DB not configured", "suggestion": "Set AUDIT_DB_PATH in atlas.env"})
    except Exception as e:
        problems.append({"id": "audit", "severity": "warning", "message": str(e), "suggestion": "Check audit module"})

    try:
        if os.getenv("SCHED_ENABLED", "true").strip().lower() in ("1", "true", "yes"):
            from modules.humanoid.scheduler import get_scheduler_db
            db = get_scheduler_db()
            _ = db.list_jobs()
    except Exception as e:
        problems.append({"id": "scheduler_db", "severity": "critical", "message": str(e), "suggestion": "Set SCHED_DB_PATH and ensure logs dir is writable"})

    try:
        if os.getenv("MEMORY_ENABLED", "true").strip().lower() in ("1", "true", "yes"):
            from modules.humanoid.memory_engine import db as mem_db
            conn = mem_db._ensure()
            conn.execute("SELECT 1")
    except Exception as e:
        problems.append({"id": "memory_db", "severity": "warning", "message": str(e), "suggestion": "Set ATLAS_MEMORY_DB_PATH or disable MEMORY_ENABLED"})

    critical = [p for p in problems if p.get("severity") == "critical"]
    ok = len(critical) == 0
    ms = int((time.perf_counter() - t0) * 1000)
    return {"ok": ok, "problems": problems, "suggestions": suggestions, "ms": ms}
