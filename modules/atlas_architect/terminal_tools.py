from __future__ import annotations

import re
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional

from modules.humanoid.hands.safe_shell import SafeShellExecutor


@dataclass
class CommandResult:
    ok: bool
    cmd: str
    returncode: int
    stdout: str
    stderr: str
    error: Optional[str] = None


def _parse_pytest_failures(text: str) -> List[Dict[str, Any]]:
    """Parse simple de fallos pytest (archivos/lineas) desde stdout/stderr."""
    t = (text or "").replace("\\", "/")
    issues: List[Dict[str, Any]] = []
    # match: path:line: in tracebacks
    for m in re.finditer(r"([A-Za-z]:/[^:\n]+\.py):(\d+):", t):
        issues.append({"file": m.group(1), "line": int(m.group(2)), "kind": "trace"})
    # match: FAILED path::test_name
    for m in re.finditer(r"FAILED\s+([^\s:]+\.py)::([^\s]+)", t):
        issues.append({"file": m.group(1), "test": m.group(2), "kind": "failed_test"})
    return issues[:80]


class TerminalTools:
    """Autonomous terminal execution + análisis de errores para refactor basado en stdout/stderr."""

    def __init__(self, repo_root: Path) -> None:
        self.repo_root = Path(repo_root).resolve()
        self.shell = SafeShellExecutor()

    def run(self, cmd: str, timeout_s: int = 180, cwd: Optional[Path] = None) -> CommandResult:
        res = self.shell.run(cmd, cwd=str((cwd or self.repo_root).resolve()), timeout_sec=int(timeout_s or 180))
        return CommandResult(
            ok=bool(res.get("ok")),
            cmd=cmd,
            returncode=int(res.get("returncode") or -1),
            stdout=str(res.get("stdout") or ""),
            stderr=str(res.get("stderr") or ""),
            error=res.get("error"),
        )

    def run_pytest(self, nodeid: Optional[str] = None, timeout_s: int = 300, cwd: Optional[Path] = None) -> Dict[str, Any]:
        cmd = "python -m pytest -q"
        if nodeid:
            cmd += " " + nodeid
        r = self.run(cmd, timeout_s=timeout_s, cwd=cwd)
        text = (r.stdout or "") + "\n" + (r.stderr or "")
        issues = _parse_pytest_failures(text)
        return {
            "ok": r.ok,
            "cmd": r.cmd,
            "returncode": r.returncode,
            "issues": issues,
            "stdout": r.stdout[-20000:],
            "stderr": r.stderr[-20000:],
            "error": r.error,
        }

    # Alias explícito para el agente (API consistente)
    def run_command(self, cmd: str, timeout_s: int = 180, cwd: Optional[Path] = None) -> Dict[str, Any]:
        r = self.run(cmd, timeout_s=timeout_s, cwd=cwd)
        return {
            "ok": r.ok,
            "cmd": r.cmd,
            "returncode": r.returncode,
            "stdout": r.stdout[-20000:],
            "stderr": r.stderr[-20000:],
            "error": r.error,
            "issues": _parse_pytest_failures((r.stdout or "") + "\n" + (r.stderr or "")),
        }

