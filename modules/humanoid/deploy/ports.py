"""Resolve active/staging ports from env and persisted state."""
from __future__ import annotations

import os
from typing import Tuple

from .switcher import get_deploy_state


def _env_int(name: str, default: int) -> int:
    try:
        return int(os.getenv(name, str(default)) or default)
    except (TypeError, ValueError):
        return default


def get_ports() -> Tuple[int, int]:
    """(active_port, staging_port) from state or env."""
    state = get_deploy_state()
    active = state.get("active_port") or _env_int("ACTIVE_PORT", 8791)
    staging = state.get("staging_port") or _env_int("STAGING_PORT", 8792)
    return int(active), int(staging)