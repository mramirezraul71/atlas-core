"""Safe shell command execution (Windows-aware)."""
from __future__ import annotations

import shlex
import subprocess
from typing import Any, Dict, List, Optional

BLOCKED_PATTERNS = ("rm -rf /", "format ", "del /f /s", "rd /s /q", "mkfs", "> /dev/sd")


class SafeShellExecutor:
    """Run shell commands with allowlist/blocklist. Returns stdout, stderr, returncode."""

    def __init__(self, allowed_cmds: Optional[List[str]] = None, blocklist: Optional[List[str]] = None) -> None:
        self.allowed_cmds = allowed_cmds or []
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
