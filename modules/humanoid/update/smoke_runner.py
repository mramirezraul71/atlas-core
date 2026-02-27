"""Run smoke tests (04_smoke_tests.ps1) via SafeShell. Policy + audit."""
from __future__ import annotations

import os
import subprocess
import time
from pathlib import Path
from typing import Any, Dict

from .git_manager import _repo_path


def _smoke_script_path() -> Path:
    p = os.getenv("UPDATE_SMOKE_SCRIPT") or str(_repo_path() / "04_smoke_tests.ps1")
    return Path(p)


def _env_int(name: str, default: int) -> int:
    try:
        return int(os.getenv(name, str(default)) or default)
    except (TypeError, ValueError):
        return default


def run_smoke(repo_path: str | None = None, port: int = 8791, timeout_sec: int = 120) -> Dict[str, Any]:
    """
    Execute 04_smoke_tests.ps1. Returns {ok, stdout, stderr, returncode, error, ms}.
    Uses subprocess (script is trusted); policy still applies if we run via Hands.
    """
    t0 = time.perf_counter()
    repo = repo_path or str(_repo_path())
    script = _smoke_script_path()
    if not script.exists():
        ms = int((time.perf_counter() - t0) * 1000)
        return {"ok": False, "stdout": "", "stderr": "", "returncode": -1, "error": f"Script not found: {script}", "ms": ms}
    try:
        cmd = ["powershell", "-ExecutionPolicy", "Bypass", "-File", str(script), "-RepoPath", repo, "-AtlasPort", str(port)]
        r = subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            encoding="utf-8",
            errors="replace",
            timeout=timeout_sec,
            cwd=repo,
        )
        ms = int((time.perf_counter() - t0) * 1000)
        return {
            "ok": r.returncode == 0,
            "stdout": (r.stdout or "")[-4096:],
            "stderr": (r.stderr or "")[-2048:],
            "returncode": r.returncode,
            "error": None if r.returncode == 0 else (r.stderr or f"exit {r.returncode}"),
            "ms": ms,
        }
    except subprocess.TimeoutExpired:
        ms = int((time.perf_counter() - t0) * 1000)
        return {"ok": False, "stdout": "", "stderr": "timeout", "returncode": -1, "error": "smoke timeout", "ms": ms}
    except Exception as e:
        ms = int((time.perf_counter() - t0) * 1000)
        return {"ok": False, "stdout": "", "stderr": str(e), "returncode": -1, "error": str(e), "ms": ms}
