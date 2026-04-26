#!/usr/bin/env python3
"""Arranca NEXUS (8000) y Robot backend (8002) en segundo plano.
Usado por heal restart_nexus_services.
"""

import os
import subprocess
import sys
import time
import urllib.request
from datetime import datetime
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
_LOCAL_NEXUS = REPO_ROOT / "nexus" / "atlas_nexus"
_LOCAL_ROBOT = REPO_ROOT / "nexus" / "atlas_nexus_robot" / "backend"
NEXUS_DIR = Path(
    os.getenv("NEXUS_ATLAS_PATH")
    or (str(_LOCAL_NEXUS) if _LOCAL_NEXUS.exists() else r"C:\ATLAS_NEXUS\atlas_nexus")
)
ROBOT_DIR = Path(
    os.getenv("NEXUS_ROBOT_PATH")
    or (
        str(_LOCAL_ROBOT)
        if _LOCAL_ROBOT.exists()
        else r"C:\ATLAS_NEXUS\atlas_nexus_robot\backend"
    )
)
LOG_DIR = REPO_ROOT / "logs"
LOG_DIR.mkdir(parents=True, exist_ok=True)
NEXUS_HEALTH_URLS = ("http://127.0.0.1:8000/health", "http://127.0.0.1:8000/status")
ROBOT_HEALTH_URLS = (
    "http://127.0.0.1:8002/health",
    "http://127.0.0.1:8002/api/health",
    "http://127.0.0.1:8002/status",
)


def _resolve_python(service: str) -> str:
    service = (service or "").strip().lower()
    if service == "nexus":
        override_var = "ATLAS_NEXUS_PYTHON"
        service_candidates = [
            REPO_ROOT / ".venv_nexus" / "Scripts" / "python.exe",
            NEXUS_DIR / "venv" / "Scripts" / "python.exe",
        ]
    elif service == "robot":
        override_var = "ATLAS_ROBOT_PYTHON"
        service_candidates = [
            REPO_ROOT / ".venv_robot" / "Scripts" / "python.exe",
            ROBOT_DIR / "venv" / "Scripts" / "python.exe",
        ]
    else:
        override_var = "ATLAS_PYTHON"
        service_candidates = []

    for env_var in (override_var, "ATLAS_PYTHON"):
        env_py = (os.getenv(env_var) or "").strip()
        if env_py and Path(env_py).exists():
            return env_py

    candidates = service_candidates + [
        REPO_ROOT / ".venv" / "Scripts" / "python.exe",
        REPO_ROOT / "venv" / "Scripts" / "python.exe",
    ]
    for c in candidates:
        if c.exists():
            return str(c)

    env_py = (os.getenv("PYTHON") or "").strip()
    if env_py and Path(env_py).exists():
        return env_py
    return "python"


NEXUS_PYTHON = _resolve_python("nexus")
ROBOT_PYTHON = _resolve_python("robot")


def _start(cmd: list[str], cwd: Path, name: str) -> bool:
    try:
        log_path = LOG_DIR / (
            ("robot_backend.log" if name.lower() == "robot" else "nexus_api.log")
            if name
            else "nexus_services.log"
        )
        try:
            with open(log_path, "a", encoding="utf-8") as lf:
                lf.write(
                    "\n--- %s START %s ---\ncmd=%s\ncwd=%s\n"
                    % (datetime.now().isoformat(), name, " ".join(cmd), str(cwd))
                )
        except Exception:
            pass
        log_f = open(log_path, "ab", buffering=0)
        kwargs = {
            "cwd": str(cwd),
            "stdout": log_f,
            "stderr": log_f,
            "stdin": subprocess.DEVNULL,
        }
        env = os.environ.copy()
        env.setdefault("PYTHONIOENCODING", "utf-8")
        env.setdefault("PYTHONUTF8", "1")
        kwargs["env"] = env
        if sys.platform == "win32":
            # Detach so the child keeps running when this script exits.
            create_no_window = getattr(subprocess, "CREATE_NO_WINDOW", 0)
            detached_process = getattr(subprocess, "DETACHED_PROCESS", 0x00000008)
            kwargs["creationflags"] = create_no_window | detached_process
        else:
            kwargs["start_new_session"] = True
        subprocess.Popen(cmd, **kwargs)
        try:
            log_f.close()
        except Exception:
            pass
        time.sleep(1)
        return True
    except Exception:
        return False


def _http_ok(url: str, timeout: float = 2.5) -> bool:
    try:
        req = urllib.request.Request(
            url, method="GET", headers={"Accept": "application/json"}
        )
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            return 200 <= int(resp.status) < 300
    except Exception:
        return False


def _service_is_healthy(urls: tuple[str, ...]) -> bool:
    return any(_http_ok(url) for url in urls)


def _port_listening(port: int) -> bool:
    """Fast local port guard to avoid duplicate launches."""
    try:
        import socket

        with socket.create_connection(("127.0.0.1", int(port)), timeout=0.8):
            return True
    except Exception:
        return False


def start_robot_only() -> str:
    """Arranca solo el backend del Robot. Devuelve 'Robot' o 'none'."""
    if not ROBOT_DIR.exists():
        return "none"
    if _port_listening(8002) or _service_is_healthy(ROBOT_HEALTH_URLS):
        return "Robot(already)"
    main_py = ROBOT_DIR / "main.py"
    if main_py.exists():
        return (
            "Robot"
            if _start([ROBOT_PYTHON, "main.py"], ROBOT_DIR, "Robot")
            else "none"
        )
    return (
        "Robot"
        if _start(
            [
                ROBOT_PYTHON,
                "-m",
                "uvicorn",
                "main:app",
                "--host",
                "0.0.0.0",
                "--port",
                "8002",
            ],
            ROBOT_DIR,
            "Robot",
        )
        else "none"
    )


def main(robot_only: bool = False, include_nexus: bool = False):
    started: list[str] = []
    if robot_only:
        return start_robot_only()
    if include_nexus and NEXUS_DIR.exists():
        if _port_listening(8000) or _service_is_healthy(NEXUS_HEALTH_URLS):
            started.append("NEXUS(already)")
        elif _start(
            [NEXUS_PYTHON, "nexus.py", "--mode", "api"], NEXUS_DIR, "NEXUS"
        ):
            started.append("NEXUS")

    if ROBOT_DIR.exists():
        if _port_listening(8002) or _service_is_healthy(ROBOT_HEALTH_URLS):
            started.append("Robot(already)")
        else:
            main_py = ROBOT_DIR / "main.py"
            if main_py.exists():
                if _start([ROBOT_PYTHON, "main.py"], ROBOT_DIR, "Robot"):
                    started.append("Robot")
            else:
                if _start(
                    [
                        ROBOT_PYTHON,
                        "-m",
                        "uvicorn",
                        "main:app",
                        "--host",
                        "0.0.0.0",
                        "--port",
                        "8002",
                    ],
                    ROBOT_DIR,
                    "Robot",
                ):
                    started.append("Robot")
    return ",".join(started) if started else "none"


if __name__ == "__main__":
    robot_only = "--robot-only" in sys.argv
    include_nexus = "--include-nexus" in sys.argv
    print(main(robot_only=robot_only, include_nexus=include_nexus))
