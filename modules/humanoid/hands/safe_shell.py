"""Safe shell command execution (Windows-aware). Policy + audit integrated."""
from __future__ import annotations

import os
import shlex
import subprocess
import time
from typing import Any, Dict, List, Optional

BLOCKED_PATTERNS = ("rm -rf /", "format ", "del /f /s", "rd /s /q", "mkfs", "> /dev/sd")


def _env_list(name: str, default: str) -> List[str]:
    v = os.getenv(name, default)
    return [x.strip() for x in (v or "").split(",") if x.strip()]


def _default_actor():
    from modules.humanoid.policy import ActorContext
    role = os.getenv("POLICY_DEFAULT_ROLE", "owner")
    return ActorContext(actor="api", role=role)


class SafeShellExecutor:
    """Run shell commands with allowlist/blocklist. PolicyEngine + AuditLogger."""

    def __init__(self, allowed_cmds: Optional[List[str]] = None, blocklist: Optional[List[str]] = None) -> None:
        self.safe_mode = os.getenv("HANDS_SAFE_MODE", "true").strip().lower() in ("1", "true", "yes")
        if allowed_cmds is not None:
            self.allowed_cmds = allowed_cmds
        elif self.safe_mode:
            policy_prefixes = os.getenv("POLICY_ALLOWED_COMMAND_PREFIXES")
            if policy_prefixes:
                self.allowed_cmds = _env_list("POLICY_ALLOWED_COMMAND_PREFIXES", "")
            else:
                self.allowed_cmds = _env_list(
                    "HANDS_ALLOWED_PREFIXES",
                    "pip,python,git,uvicorn,pytest,Invoke-RestMethod,where,Get-Command",
                )
        else:
            self.allowed_cmds = []
        self.blocklist = blocklist or list(BLOCKED_PATTERNS)

    def is_safe(self, cmd: str) -> bool:
        raw = cmd.strip().lower()
        for b in self.blocklist:
            if b.lower() in raw:
                return False
        if self.allowed_cmds:
            parts = shlex.split(cmd) or [cmd]
            first_raw = (parts[0] or "")
            first = first_raw.lower()
            allowed = [a.lower() for a in self.allowed_cmds]
            if first in allowed:
                return True
            # Permitir ejecutables de venv (aislamiento por app) sin abrir la allowlist global.
            # Ejemplos Windows: .venv\Scripts\python.exe, C:\...\apps\X\backend\.venv\Scripts\pip.exe
            try:
                from pathlib import Path

                p = Path(first_raw.strip('"').strip("'"))
                name = p.name.lower()
                if name in ("python", "python.exe", "pip", "pip.exe"):
                    s = str(p).replace("/", "\\").lower()
                    if "\\.venv\\" in s or "\\venv\\" in s:
                        return True
            except Exception:
                pass
            return False
        return True

    def run(self, cmd: str, cwd: Optional[str] = None, timeout_sec: int = 60, actor: Optional[Any] = None) -> Dict[str, Any]:
        """Run cmd. Policy check first; then execute; then audit log. Returns {ok, stdout, stderr, returncode, error}."""
        t0 = time.perf_counter()
        actor_ctx = actor or _default_actor()
        # Gobernanza dinámica: si el comando es de alto riesgo, encolar aprobación y no ejecutar.
        try:
            from modules.humanoid.governance.gates import decide
            d = decide("shell_exec", context={"command": cmd, "cwd": cwd or ""})
            if d.needs_approval and not d.allow:
                try:
                    from modules.humanoid.approvals.service import create as create_approval
                    from modules.humanoid.governance.dynamic_risk import assess_shell_command
                    a = assess_shell_command(cmd, cwd=cwd or "")
                    cr = create_approval(
                        "shell_exec",
                        {"command": cmd, "cwd": cwd or "", "risk": a.risk, "reason": a.reason, "signature": a.signature},
                    )
                    aid = cr.get("approval_id")
                    return {"ok": False, "stdout": "", "stderr": "", "returncode": -1, "error": "approval_required", "approval_id": aid}
                except Exception:
                    return {"ok": False, "stdout": "", "stderr": "", "returncode": -1, "error": "approval_required"}
        except Exception:
            pass
        try:
            from modules.humanoid.policy import get_policy_engine
            decision = get_policy_engine().can(actor_ctx, "hands", "exec_command", target=cmd)
            if not decision.allow:
                out = {"ok": False, "stdout": "", "stderr": "", "returncode": -1, "error": decision.reason}
                ms = int((time.perf_counter() - t0) * 1000)
                try:
                    from modules.humanoid.audit import get_audit_logger
                    get_audit_logger().log_event(actor_ctx.actor, actor_ctx.role, "hands", "exec_command", False, ms, decision.reason, {"command": cmd, "cwd": cwd}, None)
                except Exception:
                    pass
                return out
        except Exception:
            pass
        if not self.is_safe(cmd):
            out = {"ok": False, "stdout": "", "stderr": "", "returncode": -1, "error": "command not allowed"}
            ms = int((time.perf_counter() - t0) * 1000)
            try:
                from modules.humanoid.audit import get_audit_logger
                get_audit_logger().log_event(actor_ctx.actor, actor_ctx.role, "hands", "exec_command", False, ms, "command not allowed", {"command": cmd, "cwd": cwd}, None)
            except Exception:
                pass
            return out
        try:
            r = subprocess.run(
                cmd,
                shell=True,
                capture_output=True,
                text=True,
                encoding="utf-8",
                errors="replace",
                cwd=cwd,
                timeout=timeout_sec,
            )
            out = {
                "ok": r.returncode == 0,
                "stdout": r.stdout or "",
                "stderr": r.stderr or "",
                "returncode": r.returncode,
                "error": None,
            }
            ms = int((time.perf_counter() - t0) * 1000)
            try:
                from modules.humanoid.audit import get_audit_logger
                get_audit_logger().log_event(actor_ctx.actor, actor_ctx.role, "hands", "exec_command", out["ok"], ms, out.get("error"), {"command": cmd, "cwd": cwd}, {"returncode": r.returncode, "stdout_len": len(r.stdout or ""), "stderr_len": len(r.stderr or "")})
            except Exception:
                pass
            return out
        except subprocess.TimeoutExpired:
            out = {"ok": False, "stdout": "", "stderr": "", "returncode": -1, "error": "timeout"}
            ms = int((time.perf_counter() - t0) * 1000)
            try:
                from modules.humanoid.audit import get_audit_logger
                get_audit_logger().log_event(actor_ctx.actor, actor_ctx.role, "hands", "exec_command", False, ms, "timeout", {"command": cmd, "cwd": cwd}, None)
            except Exception:
                pass
            return out
        except Exception as e:
            out = {"ok": False, "stdout": "", "stderr": "", "returncode": -1, "error": str(e)}
            ms = int((time.perf_counter() - t0) * 1000)
            try:
                from modules.humanoid.audit import get_audit_logger
                get_audit_logger().log_event(actor_ctx.actor, actor_ctx.role, "hands", "exec_command", False, ms, str(e), {"command": cmd, "cwd": cwd}, None)
            except Exception:
                pass
            return out
