"""Launcher for ATLAS as Windows service: load venv, run uvicorn, rotating log."""
from __future__ import annotations

import atexit
import logging
import os
import sys
import urllib.request
from logging.handlers import RotatingFileHandler
from pathlib import Path

# Repo root: parent of tools/
REPO_ROOT = Path(__file__).resolve().parent.parent
os.chdir(REPO_ROOT)
sys.path.insert(0, str(REPO_ROOT))

# Load env
env_file = REPO_ROOT / "config" / "atlas.env"
if env_file.exists():
    from dotenv import load_dotenv

    load_dotenv(env_file)

LOG_PATH = os.getenv("SERVICE_LOG_PATH", str(REPO_ROOT / "logs" / "service.log"))
Path(LOG_PATH).parent.mkdir(parents=True, exist_ok=True)
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    handlers=[
        RotatingFileHandler(
            LOG_PATH, maxBytes=2 * 1024 * 1024, backupCount=3, encoding="utf-8"
        ),
        logging.StreamHandler(),
    ],
)
log = logging.getLogger("atlas.service")
_LOCK_PATH: Path | None = None
_LOCK_FD: int | None = None


def _api_already_up(host: str, port: int) -> bool:
    """Sonda HTTP antes de lanzar uvicorn. Evita doble instancia en el mismo puerto."""
    try:
        url = f"http://{host}:{port}/health"
        with urllib.request.urlopen(url, timeout=3) as r:
            return r.status == 200
    except Exception:
        return False


def _pid_alive(pid: int) -> bool:
    try:
        if pid <= 0:
            return False
        os.kill(pid, 0)
        return True
    except Exception:
        return False


def _read_lock_pid(path: Path) -> int | None:
    try:
        raw = path.read_text(encoding="utf-8").strip()
        if not raw:
            return None
        return int(raw.splitlines()[0].strip())
    except Exception:
        return None


def _release_singleton_lock() -> None:
    global _LOCK_FD, _LOCK_PATH
    try:
        if _LOCK_FD is not None:
            os.close(_LOCK_FD)
            _LOCK_FD = None
    except Exception:
        pass
    try:
        if _LOCK_PATH and _LOCK_PATH.exists():
            _LOCK_PATH.unlink()
    except Exception:
        pass
    _LOCK_PATH = None


def _acquire_singleton_lock(port: int) -> bool:
    """Acquire singleton lock for this launcher instance."""
    global _LOCK_FD, _LOCK_PATH
    lock_path = REPO_ROOT / "logs" / f"service_launcher_{port}.lock"
    lock_path.parent.mkdir(parents=True, exist_ok=True)

    if lock_path.exists():
        existing_pid = _read_lock_pid(lock_path)
        if existing_pid and _pid_alive(existing_pid):
            log.info("Launcher lock busy: pid=%s (port=%s).", existing_pid, port)
            return False
        try:
            lock_path.unlink()
            log.warning("Removed stale launcher lock: %s", lock_path)
        except Exception as e:
            log.error("Cannot remove stale lock %s: %s", lock_path, e)
            return False

    try:
        flags = os.O_CREAT | os.O_EXCL | os.O_WRONLY
        fd = os.open(str(lock_path), flags)
        os.write(fd, str(os.getpid()).encode("utf-8"))
        _LOCK_FD = fd
        _LOCK_PATH = lock_path
        atexit.register(_release_singleton_lock)
        return True
    except FileExistsError:
        return False
    except Exception as e:
        log.error("Unable to create launcher lock: %s", e)
        return False


def main():
    host = os.getenv("SERVICE_BIND", "127.0.0.1")
    port = int(os.getenv("SERVICE_PORT", "8791") or 8791)
    if os.getenv("DEPLOY_MODE", "").strip().lower() == "bluegreen":
        state_file = REPO_ROOT / "logs" / "deploy_state.json"
        if state_file.exists():
            try:
                import json

                state = json.loads(state_file.read_text(encoding="utf-8"))
                port = int(state.get("active_port", port))
            except Exception:
                pass
    if _api_already_up(host, port):
        log.info("API ya activa en %s:%s. service_launcher saliendo sin duplicar.", host, port)
        return 0
    if not _acquire_singleton_lock(port):
        log.info("No lock acquired for %s:%s. service_launcher exiting.", host, port)
        return 0

    app_import = os.getenv("SERVICE_APP_IMPORT", "atlas_adapter.atlas_http_api:app")
    log.info("Starting ATLAS service on %s:%s (app=%s)", host, port, app_import)
    import uvicorn

    uvicorn.run(app_import, host=host, port=port, log_level="info")


def _run_selfcheck() -> None:
    """Run selfcheck after env load; log critical issues, do not crash."""
    try:
        import os

        if os.getenv("WORKER_ONLY", "").strip().lower() in ("1", "true", "yes"):
            return
        from modules.humanoid.product.selfcheck import run_selfcheck

        result = run_selfcheck()
        problems = result.get("problems") or []
        critical = [p for p in problems if p.get("severity") == "critical"]
        if critical:
            for p in critical:
                log.warning(
                    "Selfcheck critical: %s - %s", p.get("id"), p.get("message")
                )
    except Exception as e:
        log.debug("Selfcheck skipped: %s", e)


if __name__ == "__main__":
    _run_selfcheck()
    main()
