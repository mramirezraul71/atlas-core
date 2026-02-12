"""Start/stop uvicorn process on a port. PID tracking for Windows."""
from __future__ import annotations

import os
import subprocess
import time
from pathlib import Path
from typing import Any, Dict, Optional

TIMEOUT_START_SEC = 30
TIMEOUT_STOP_SEC = 10


def _repo_path() -> Path:
    return Path(os.getenv("ATLAS_PUSH_ROOT", os.getcwd())).resolve()


def start_instance(port: int, tag: str = "staging") -> Dict[str, Any]:
    """
    Start uvicorn on port. Returns {ok, pid, port, error}.
    tag used for env ATLAS_DEPLOY_TAG (staging/active).
    """
    repo = _repo_path()
    venv_python = repo / ".venv" / "Scripts" / "python.exe"
    if not venv_python.exists():
        venv_python = repo / ".venv" / "bin" / "python"
    if not venv_python.exists():
        return {"ok": False, "pid": None, "port": port, "error": "venv not found"}
    try:
        proc = subprocess.Popen(
            [
                str(venv_python),
                "-m", "uvicorn",
                "atlas_adapter.atlas_http_api:app",
                "--host", "127.0.0.1",
                "--port", str(port),
            ],
            cwd=str(repo),
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
            env={**os.environ, "ATLAS_DEPLOY_TAG": tag},
        )
        pid = proc.pid
        return {"ok": True, "pid": pid, "port": port, "error": None}
    except Exception as e:
        return {"ok": False, "pid": None, "port": port, "error": str(e)}


def stop_by_pid(pid: Optional[int]) -> Dict[str, Any]:
    """Stop process by PID. Returns {ok, error}."""
    if pid is None:
        return {"ok": True, "error": None}
    try:
        if os.name == "nt":
            subprocess.run(["taskkill", "/PID", str(pid), "/F"], capture_output=True, timeout=TIMEOUT_STOP_SEC)
        else:
            import signal
            os.kill(pid, signal.SIGTERM)
            time.sleep(1)
    except subprocess.TimeoutExpired:
        return {"ok": False, "error": "timeout killing process"}
    except ProcessLookupError:
        return {"ok": True, "error": None}
    except Exception as e:
        return {"ok": False, "error": str(e)}
    return {"ok": True, "error": None}


def wait_until_healthy(base_url: str, timeout_sec: int = TIMEOUT_START_SEC) -> bool:
    """Poll GET /health until ok or timeout."""
    import urllib.request
    url = f"{base_url.rstrip('/')}/health"
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        try:
            req = urllib.request.Request(url, method="GET")
            with urllib.request.urlopen(req, timeout=5) as r:
                import json
                data = json.loads(r.read().decode())
                if data.get("ok") or data.get("score", 0) >= 60:
                    return True
        except Exception:
            pass
        time.sleep(1)
    return False