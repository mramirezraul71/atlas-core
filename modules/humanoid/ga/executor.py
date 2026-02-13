"""Execute safe actions via hands/dispatcher with timeouts and evidence."""
from __future__ import annotations

import os
import subprocess
import time
from typing import Any, Dict, List, Optional

from .models import ActionCandidate, ExecutionResult


def _policy_allows_ga_autorun() -> bool:
    try:
        from modules.humanoid.policy import get_policy_engine
        from modules.humanoid.policy.models import ActorContext
        ctx = ActorContext(actor="ga_system", role="system")
        return get_policy_engine().can(ctx, "ga", "ga_autorun", None).allow
    except Exception:
        return False


def _audit(module: str, action: str, ok: bool, payload: Optional[Dict] = None, error: Optional[str] = None, ms: int = 0) -> None:
    try:
        from modules.humanoid.audit import get_audit_logger
        get_audit_logger().log_event("ga", "system", module, action, ok, ms, error, payload, None)
    except Exception:
        pass


def _write_memory(summary: str, payload: Dict[str, Any]) -> None:
    try:
        from modules.humanoid.memory_engine import ensure_thread, memory_write
        tid = ensure_thread(None, "GA")
        memory_write(tid, "summary", {"content": summary, **payload})
    except Exception:
        pass


def _dispatch_add_timeout(cand: ActionCandidate) -> ExecutionResult:
    """Add timeout: typically to pytest or CI - no-op placeholder (CI handles)."""
    t0 = time.perf_counter()
    meta = cand.finding.meta
    path = meta.get("path", cand.payload.get("path", ""))
    if not path:
        return ExecutionResult(action_type="add_timeout", ok=False, error="no path", ms=int((time.perf_counter() - t0) * 1000))
    # Deterministic: create placeholder evidence (real impl would patch file)
    return ExecutionResult(
        action_type="add_timeout",
        ok=True,
        exit_code=0,
        stdout_snip="timeout placeholder",
        paths=[path],
        evidence={"path": path, "patched": False},
        ms=int((time.perf_counter() - t0) * 1000),
    )


def _dispatch_add_smoke_test(cand: ActionCandidate) -> ExecutionResult:
    """Add smoke test - no-op placeholder."""
    t0 = time.perf_counter()
    return ExecutionResult(
        action_type="add_smoke_test",
        ok=True,
        exit_code=0,
        stdout_snip="smoke placeholder",
        evidence={"patched": False},
        ms=int((time.perf_counter() - t0) * 1000),
    )


def _dispatch_log_improvement(cand: ActionCandidate) -> ExecutionResult:
    """Log improvement - no-op placeholder."""
    t0 = time.perf_counter()
    return ExecutionResult(
        action_type="log_improvement",
        ok=True,
        exit_code=0,
        evidence={},
        ms=int((time.perf_counter() - t0) * 1000),
    )


def _dispatch_notify_owner(cand: ActionCandidate) -> ExecutionResult:
    """Notify owner - audit only (no external notify in safe mode)."""
    t0 = time.perf_counter()
    _audit("ga", "notify_owner", True, {"finding": cand.finding.detail, "source": cand.finding.source})
    return ExecutionResult(
        action_type="notify_owner",
        ok=True,
        exit_code=0,
        evidence={"audited": True},
        ms=int((time.perf_counter() - t0) * 1000),
    )


def _dispatch_autofix(cand: ActionCandidate) -> ExecutionResult:
    """Autofix: delegate to CI apply_autofix if available."""
    t0 = time.perf_counter()
    path = cand.payload.get("path", "")
    action = "add_param_defaults" if cand.finding.kind == "ps1_no_param" else "autofix"
    try:
        from modules.humanoid.ci.executor import apply_autofix
        item = {"action": action, "path": path}
        r = apply_autofix(item)
    except ImportError:
        r = {"ok": False, "path": path, "error": "CI executor not available"}
    ms = int((time.perf_counter() - t0) * 1000)
    return ExecutionResult(
        action_type="autofix",
        ok=bool(r.get("ok")),
        exit_code=0 if r.get("ok") else 1,
        stdout_snip=str(r.get("error", ""))[:500],
        changed_files=[path] if r.get("ok") and path else [],
        paths=[path] if path else [],
        evidence=r,
        error=r.get("error"),
        ms=ms,
    )


def _dispatch_update_check(cand: ActionCandidate) -> ExecutionResult:
    """Run update check."""
    t0 = time.perf_counter()
    try:
        from modules.humanoid.update.update_engine import check
        r = check()
        ok = r.get("ok", False)
    except Exception as e:
        r = {"error": str(e)}
        ok = False
    ms = int((time.perf_counter() - t0) * 1000)
    return ExecutionResult(
        action_type="update_check",
        ok=ok,
        exit_code=0 if ok else 1,
        stdout_snip=str(r.get("data", r))[:500],
        evidence=r,
        error=r.get("error"),
        ms=ms,
    )


def _dispatch_restart_internal_loop(cand: ActionCandidate) -> ExecutionResult:
    """Restart scheduler loop if stopped."""
    t0 = time.perf_counter()
    try:
        from modules.humanoid.healing import restart_scheduler
        r = restart_scheduler()
        ok = r.get("ok", False)
    except Exception as e:
        r = {"error": str(e)}
        ok = False
    ms = int((time.perf_counter() - t0) * 1000)
    return ExecutionResult(
        action_type="restart_internal_loop",
        ok=ok,
        exit_code=0 if ok else 1,
        evidence=r,
        error=r.get("error"),
        ms=ms,
    )


DISPATCH = {
    "add_timeout": _dispatch_add_timeout,
    "add_smoke_test": _dispatch_add_smoke_test,
    "log_improvement": _dispatch_log_improvement,
    "notify_owner": _dispatch_notify_owner,
    "autofix": _dispatch_autofix,
    "update_check": _dispatch_update_check,
    "restart_internal_loop": _dispatch_restart_internal_loop,
}


def execute_safe(
    candidates: List[ActionCandidate],
    limit: int = 2,
    strict_evidence: bool = True,
) -> List[ExecutionResult]:
    """Execute safe actions; respect limit and policy. Capture evidence."""
    if not _policy_allows_ga_autorun():
        return []
    enabled = os.getenv("GA_SAFE_AUTORUN_ENABLED", "true").strip().lower() in ("1", "true", "yes")
    if not enabled:
        return []
    results: List[ExecutionResult] = []
    for c in candidates[:limit]:
        fn = DISPATCH.get(c.action_type)
        if not fn:
            continue
        t0 = time.perf_counter()
        try:
            res = fn(c)
        except Exception as e:
            res = ExecutionResult(action_type=c.action_type, ok=False, error=str(e), ms=int((time.perf_counter() - t0) * 1000))
        if strict_evidence and res.ok:
            req = c.evidence_required
            if "exit_code" in req and res.exit_code is None:
                res = ExecutionResult(
                    action_type=res.action_type,
                    ok=False,
                    error="strict_evidence: exit_code required",
                    ms=res.ms,
                )
        if res.ok:
            _audit("ga", "safe_execute", True, {"action": c.action_type, "finding": c.finding.detail}, None, res.ms)
            _write_memory(f"GA safe: {c.action_type} - {c.finding.detail}", {"action": c.action_type, "evidence": res.evidence})
        results.append(res)
    return results
