"""Shared filesystem and runtime settings for PUSH startup."""
from __future__ import annotations

import importlib.util
import os
from pathlib import Path

BASE_DIR = Path(__file__).resolve().parents[2]
ENV_PATH = BASE_DIR / "config" / "atlas.env"
LOGS_DIR = BASE_DIR / "logs"

ATLAS_ROOT = Path(r"C:\ATLAS")
ROUTER_PATH = ATLAS_ROOT / "modules" / "command_router.py"
LOCAL_ROUTER = BASE_DIR / "modules" / "command_router.py"

_DB_PATH_DEFAULTS = (
    ("SCHED_DB_PATH", "atlas_sched.sqlite"),
    ("ATLAS_MEMORY_DB_PATH", "atlas_memory.sqlite"),
    ("MEMORY_DB_PATH", "atlas_memory.sqlite"),
    ("AUDIT_DB_PATH", "atlas_audit.sqlite"),
    ("LEARNING_EPISODIC_DB_PATH", "learning_episodic.sqlite"),
    ("NERVOUS_DB_PATH", "atlas_nervous.sqlite"),
    ("NERVOUS_DB_PATH", "nervous_system.sqlite"),
    ("NERVOUS_DB_PATH", "nervous_system.sqlite"),
)


def ensure_db_path(env_var: str, filename: str) -> None:
    """Force env var to a writable path under the project logs dir."""
    path = Path(os.getenv(env_var) or str(LOGS_DIR / filename))
    try:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.touch()
    except Exception:
        path = LOGS_DIR / filename
        path.parent.mkdir(parents=True, exist_ok=True)
        try:
            path.touch()
        except Exception:
            pass
    os.environ[env_var] = str(path)


def initialize_runtime_paths() -> None:
    """Prepare writable runtime paths before importing app modules."""
    LOGS_DIR.mkdir(parents=True, exist_ok=True)
    for env_var, filename in _DB_PATH_DEFAULTS:
        ensure_db_path(env_var, filename)


def load_handle():
    """Load the command router handle from the operator install or repo copy."""
    router_path = ROUTER_PATH if ROUTER_PATH.exists() else LOCAL_ROUTER
    spec = importlib.util.spec_from_file_location("command_router", str(router_path))
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Unable to load command router from {router_path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)  # type: ignore[attr-defined]
    return module.handle  # type: ignore[attr-defined]
