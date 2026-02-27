"""Launcher for ATLAS as Windows service: load venv, run uvicorn, rotating log."""
from __future__ import annotations

import os
import sys
import logging
from pathlib import Path
from logging.handlers import RotatingFileHandler

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
        RotatingFileHandler(LOG_PATH, maxBytes=2 * 1024 * 1024, backupCount=3, encoding="utf-8"),
        logging.StreamHandler(),
    ],
)
log = logging.getLogger("atlas.service")

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
                log.warning("Selfcheck critical: %s - %s", p.get("id"), p.get("message"))
    except Exception as e:
        log.debug("Selfcheck skipped: %s", e)


if __name__ == "__main__":
    _run_selfcheck()
    main()
