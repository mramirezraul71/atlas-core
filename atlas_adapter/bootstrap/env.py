"""Environment loading helpers used before importing env-sensitive modules."""
from __future__ import annotations

import logging
import os
from pathlib import Path


def load_project_env(env_path: Path) -> Path:
    """Load the repo env file early so modules see the expected os.environ."""
    if env_path.exists():
        logging.getLogger("dotenv").setLevel(logging.ERROR)
        from dotenv import load_dotenv

        load_dotenv(env_path, override=True)
    else:
        logging.warning("atlas.env not found at %s", env_path)
    return env_path


def load_vault_env(vault_path: str | None = None) -> None:
    """Best-effort load of the credentials vault used by local operators."""
    try:
        from dotenv import load_dotenv

        primary_path = (
            vault_path
            or os.getenv("ATLAS_VAULT_PATH")
            or r"C:\Users\Raul\OneDrive\RAUL - Personal\Escritorio\credenciales.txt"
        ).strip()
        for candidate in (primary_path, r"C:\dev\credenciales.txt"):
            path = Path(candidate)
            if path.is_file():
                load_dotenv(str(path), override=True)
                break
    except Exception:
        pass
