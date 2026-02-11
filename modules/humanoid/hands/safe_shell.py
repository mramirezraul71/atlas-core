"""Safe shell command execution (Windows-aware)."""
from __future__ import annotations

import os
import shlex
import subprocess
from typing import Any, Dict, List, Optional

BLOCKED_PATTERNS = ("rm -rf /", "format ", "del /f /s", "rd /s /q", "mkfs", "> /dev/sd")


def _env_list(name: str, default: str) -> List[str]:
    v = os.getenv(name, default)
    return [x.strip() for x in (v or "").split(",") if x.strip()]


class SafeShellExecutor:
    """Run shell commands with allowlist/blocklist. Returns stdout, stderr, returncode."""

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
            first = (parts[0] or "").lower()
            return first in [a.lower() for a in self.allowed_cmds]
        return True

    def run(self, cmd: str, cwd: Optional[str] = None, timeout_sec: int = 60) -> Dict[str, Any]:
        """Run cmd. Returns {ok, stdout, stderr, returncode, error}."""
        if not self.is_safe(cmd):
            return {"ok": False, "stdout": "", "stderr": "", "returncode": -1, "error": "command not allowed"}
        try:
            r = subprocess.run(
                cmd, shell=True, capture_output=True, text=True, cwd=cwd, timeout=timeout_sec,
            )
            return {
                "ok": r.returncode == 0,
                "stdout": r.stdout or "",
                "stderr": r.stderr or "",
                "returncode": r.returncode,
                "error": None,
            }
        except subprocess.TimeoutExpired:
            return {"ok": False, "stdout": "", "stderr": "", "returncode": -1, "error": "timeout"}
        except Exception as e:
            return {"ok": False, "stdout": "", "stderr": "", "returncode": -1, "error": str(e)}
