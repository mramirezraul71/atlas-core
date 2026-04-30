"""Policy config from env: POLICY_* and AUDIT_*."""
from __future__ import annotations

import os
from pathlib import Path
from typing import List


def _env_bool(name: str, default: bool) -> bool:
    v = os.getenv(name, "true" if default else "false").strip().lower()
    return v in ("1", "true", "yes", "y", "on")


def _env_list(name: str, default: str = "") -> List[str]:
    v = os.getenv(name, default)
    return [x.strip() for x in (v or "").split(",") if x.strip()]


def _env_str(name: str, default: str) -> str:
    return (os.getenv(name) or default).strip()


# Policy
POLICY_MODE = _env_str("POLICY_MODE", "strict")
POLICY_DEFAULT_ROLE = _env_str("POLICY_DEFAULT_ROLE", "owner")
POLICY_ALLOWED_PATHS: List[Path] = [
    Path(p) for p in _env_list("POLICY_ALLOWED_PATHS", "C:\\ATLAS_PUSH")
]
POLICY_ALLOWED_COMMAND_PREFIXES: List[str] = _env_list(
    "POLICY_ALLOWED_COMMAND_PREFIXES",
    "pip,python,git,uvicorn,pytest,where,Get-Command,Invoke-RestMethod,curl.exe",
)
POLICY_ALLOW_KILL_PROCESS = _env_bool("POLICY_ALLOW_KILL_PROCESS", False)
POLICY_ALLOW_UPDATE_APPLY = _env_bool("POLICY_ALLOW_UPDATE_APPLY", False)

# Audit
AUDIT_DB_PATH: str | None = os.getenv("AUDIT_DB_PATH")
AUDIT_ENABLED = _env_bool("AUDIT_ENABLED", True)
