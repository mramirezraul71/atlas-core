"""Safe updater: plan only by default. Policy + audit for update-check and update-apply."""
from __future__ import annotations

import os
import subprocess
import sys
import time
from typing import Any, Dict, List

from .env_scanner import EnvScanner
from .dep_resolver import DepResolver


def _default_actor():
    from modules.humanoid.policy import ActorContext
    return ActorContext(actor="api", role=os.getenv("POLICY_DEFAULT_ROLE", "owner"))


def _env_bool(name: str, default: bool) -> bool:
    v = os.getenv(name, "true" if default else "false").strip().lower()
    return v in ("1", "true", "yes", "y", "on")


def _env_str(name: str, default: str) -> str:
    return (os.getenv(name) or default).strip()


class Updater:
    """Plan → Verify → Apply. Apply runs only if UPDATE_APPLY=true or force=true."""

    def __init__(self) -> None:
        self.scanner = EnvScanner()
        self.resolver = DepResolver()
        self.update_mode = _env_str("UPDATE_MODE", "plan_only")
        self.update_apply = _env_bool("UPDATE_APPLY", False)
        self.snapshot_before = _env_bool("UPDATE_SNAPSHOT_BEFORE", True)
        self.allow_pip_upgrade = _env_bool("UPDATE_ALLOW_PIP_UPGRADE", False)

    def snapshot_before_apply(self, cwd: str | None = None) -> Dict[str, Any]:
        """Run git status + git diff (plan only, no commit). Returns {ok, status_output, diff_output}."""
        cwd = cwd or os.getcwd()
        out: Dict[str, Any] = {"ok": True, "status_output": "", "diff_output": "", "error": None}
        try:
            r = subprocess.run(
                ["git", "status", "--short"],
                cwd=cwd,
                capture_output=True,
                text=True,
                timeout=10,
            )
            out["status_output"] = (r.stdout or r.stderr or "").strip()
            if r.returncode != 0:
                out["ok"] = False
                out["error"] = out["status_output"]
                return out
            r2 = subprocess.run(
                ["git", "diff", "--no-color"],
                cwd=cwd,
                capture_output=True,
                text=True,
                timeout=10,
            )
            out["diff_output"] = (r2.stdout or "").strip()[:8192]
        except Exception as e:
            out["ok"] = False
            out["error"] = str(e)
        return out

    def plan(self, required_packages: List[str], actor: Any = None) -> Dict[str, Any]:
        """Update-check: always allowed for owner. Returns plan + audit log."""
        t0 = time.perf_counter()
        actor_ctx = actor or _default_actor()
        out: Dict[str, Any] = {
            "ok": True,
            "python_version": None,
            "pip_list_ok": False,
            "missing": [],
            "install_plan": [],
            "plan_commands": [],
            "update_config": {
                "UPDATE_MODE": self.update_mode,
                "UPDATE_APPLY": self.update_apply,
                "UPDATE_SNAPSHOT_BEFORE": self.snapshot_before,
                "UPDATE_ALLOW_PIP_UPGRADE": self.allow_pip_upgrade,
            },
            "error": None,
        }
        pv = self.scanner.python_version()
        out["python_version"] = pv.get("version")
        if not pv.get("ok"):
            out["ok"] = False
            out["error"] = pv.get("error")
            return out
        pl = self.scanner.pip_list()
        out["pip_list_ok"] = pl.get("ok")
        if pl.get("ok"):
            self.resolver.set_packages(pl.get("packages", []))
            ch = self.resolver.check(required_packages)
            out["missing"] = ch.get("missing", [])
            for p in out["missing"]:
                cmd = f"pip install {p}"
                out["install_plan"].append(cmd)
                out["plan_commands"].append(cmd)
        else:
            out["error"] = pl.get("error")
            out["ok"] = False
        ms = round((time.perf_counter() - t0) * 1000)
        try:
            from modules.humanoid.audit import get_audit_logger
            get_audit_logger().log_event(actor_ctx.actor, actor_ctx.role, "update", "check", out["ok"], ms, out.get("error"), {"required_packages": required_packages}, {"missing": out.get("missing", []), "plan_commands": out.get("plan_commands", [])})
        except Exception:
            pass
        return out

    def apply(self, required_packages: List[str], force: bool = False, actor: Any = None) -> Dict[str, Any]:
        """Run installs only if UPDATE_APPLY=true or force=true. Returns {ok, ms, error, applied_commands, snapshot}."""
        t0 = time.perf_counter()
        out: Dict[str, Any] = {
            "ok": False,
            "ms": 0,
            "error": None,
            "applied_commands": [],
            "snapshot": None,
        }
        if not (self.update_apply or force):
            out["error"] = "UPDATE_APPLY=false and force not set; refusing to run installs"
            out["ms"] = round((time.perf_counter() - t0) * 1000)
            return out
        policy_allow = os.getenv("POLICY_ALLOW_UPDATE_APPLY", "false").strip().lower() in ("1", "true", "yes")
        if not policy_allow and not force:
            out["error"] = "POLICY_ALLOW_UPDATE_APPLY=false; refusing to run installs"
            out["ms"] = round((time.perf_counter() - t0) * 1000)
            return out
        actor_ctx = actor or _default_actor()
        try:
            from modules.humanoid.policy import get_policy_engine
            decision = get_policy_engine().can(actor_ctx, "update", "apply")
            if not decision.allow and not force:
                out["error"] = decision.reason
                out["ms"] = round((time.perf_counter() - t0) * 1000)
                return out
        except Exception:
            pass
        plan_result = self.plan(required_packages, actor=actor_ctx)
        if not plan_result.get("ok"):
            out["error"] = plan_result.get("error", "plan failed")
            out["ms"] = round((time.perf_counter() - t0) * 1000)
            return out
        commands = plan_result.get("plan_commands", plan_result.get("install_plan", []))
        if not commands:
            out["ok"] = True
            out["ms"] = round((time.perf_counter() - t0) * 1000)
            return out
        if self.snapshot_before:
            out["snapshot"] = self.snapshot_before_apply()
        for cmd in commands:
            try:
                r = subprocess.run(
                    [sys.executable, "-m", "pip", "install", cmd.replace("pip install ", "").strip()],
                    capture_output=True,
                    text=True,
                    timeout=300,
                )
                out["applied_commands"].append({"cmd": cmd, "returncode": r.returncode, "stderr": (r.stderr or "")[:500]})
                if r.returncode != 0:
                    out["error"] = out["error"] or (r.stderr or str(r.returncode))
            except Exception as e:
                out["applied_commands"].append({"cmd": cmd, "error": str(e)})
                out["error"] = out["error"] or str(e)
        out["ok"] = out["error"] is None
        out["ms"] = round((time.perf_counter() - t0) * 1000)
        try:
            from modules.humanoid.audit import get_audit_logger
            get_audit_logger().log_event(actor_ctx.actor, actor_ctx.role, "update", "apply", out["ok"], out["ms"], out.get("error"), {"required_packages": required_packages, "force": force}, {"applied_commands": len(out.get("applied_commands", []))})
        except Exception:
            pass
        return out
