#!/usr/bin/env python3
"""Arranca NEXUS (8000) y Robot backend (8002) en segundo plano. Usado por heal restart_nexus_services."""
import os
import sys
import subprocess
import time
from pathlib import Path

NEXUS_DIR = Path(os.getenv("NEXUS_ATLAS_PATH") or r"C:\ATLAS_NEXUS\atlas_nexus")
ROBOT_DIR = Path(os.getenv("NEXUS_ROBOT_PATH") or r"C:\ATLAS_NEXUS\atlas_nexus_robot\backend")
PYTHON = os.getenv("PYTHON", "python")


def _start(cmd: list, cwd: Path, name: str) -> bool:
    try:
        flags = subprocess.CREATE_NO_WINDOW if sys.platform == "win32" and hasattr(subprocess, "CREATE_NO_WINDOW") else 0
        kwargs = {"cwd": str(cwd), "stdout": subprocess.DEVNULL, "stderr": subprocess.DEVNULL}
        if sys.platform == "win32" and flags:
            kwargs["creationflags"] = flags
        else:
            kwargs["start_new_session"] = True
        subprocess.Popen(cmd, **kwargs)
        time.sleep(1)
        return True
    except Exception:
        return False


def main():
    started = []
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
    print(main())
