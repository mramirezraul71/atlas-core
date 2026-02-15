#!/usr/bin/env python3
"""Arranca NEXUS (8000) y Robot backend (8002) en segundo plano. Usado por heal restart_nexus_services."""
import os
import sys
import subprocess
import time
from pathlib import Path
from datetime import datetime

NEXUS_DIR = Path(os.getenv("NEXUS_ATLAS_PATH") or r"C:\ATLAS_NEXUS\atlas_nexus")
ROBOT_DIR = Path(os.getenv("NEXUS_ROBOT_PATH") or r"C:\ATLAS_NEXUS\atlas_nexus_robot\backend")
PYTHON = os.getenv("PYTHON", "python")
REPO_ROOT = Path(__file__).resolve().parent.parent
LOG_DIR = REPO_ROOT / "logs"
LOG_DIR.mkdir(parents=True, exist_ok=True)


def _start(cmd: list, cwd: Path, name: str) -> bool:
    try:
        log_path = LOG_DIR / (("robot_backend.log" if name.lower() == "robot" else "nexus_api.log") if name else "nexus_services.log")
        try:
            with open(log_path, "a", encoding="utf-8") as lf:
                lf.write("\n--- %s START %s ---\ncmd=%s\ncwd=%s\n" % (datetime.now().isoformat(), name, " ".join(cmd), str(cwd)))
        except Exception:
            pass
        log_f = open(log_path, "ab", buffering=0)
        kwargs = {"cwd": str(cwd), "stdout": log_f, "stderr": log_f, "stdin": subprocess.DEVNULL}
        if sys.platform == "win32":
            # Detach so the child keeps running when this script exits (e.g. when called from API)
            CREATE_NO_WINDOW = getattr(subprocess, "CREATE_NO_WINDOW", 0)
            DETACHED_PROCESS = getattr(subprocess, "DETACHED_PROCESS", 0x00000008)
            kwargs["creationflags"] = CREATE_NO_WINDOW | DETACHED_PROCESS
        else:
            kwargs["start_new_session"] = True
        p = subprocess.Popen(cmd, **kwargs)
        try:
            log_f.close()
        except Exception:
            pass
        time.sleep(1)
        return True
    except Exception:
        return False


def start_robot_only() -> str:
    """Arranca solo el backend del Robot (cámaras, visión). Devuelve 'Robot' o 'none'."""
    if not ROBOT_DIR.exists():
        return "none"
    main_py = ROBOT_DIR / "main.py"
    if main_py.exists():
        return "Robot" if _start([PYTHON, "main.py"], ROBOT_DIR, "Robot") else "none"
    return "Robot" if _start([PYTHON, "-m", "uvicorn", "main:app", "--host", "0.0.0.0", "--port", "8002"], ROBOT_DIR, "Robot") else "none"


def main(robot_only: bool = False):
    started = []
    if robot_only:
        return start_robot_only()
    if NEXUS_DIR.exists():
        if _start([PYTHON, "nexus.py", "--mode", "api"], NEXUS_DIR, "NEXUS"):
            started.append("NEXUS")
    if ROBOT_DIR.exists():
        main_py = ROBOT_DIR / "main.py"
        if main_py.exists():
            if _start([PYTHON, "main.py"], ROBOT_DIR, "Robot"):
                started.append("Robot")
        else:
            if _start([PYTHON, "-m", "uvicorn", "main:app", "--host", "0.0.0.0", "--port", "8002"], ROBOT_DIR, "Robot"):
                started.append("Robot")
    return ",".join(started) if started else "none"


if __name__ == "__main__":
    robot_only = "--robot-only" in sys.argv
    print(main(robot_only=robot_only))
