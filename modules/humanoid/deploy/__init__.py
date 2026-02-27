"""Deploy: blue-green local switching, health-based switch, canary ramp-up."""
from __future__ import annotations

from .bluegreen import run_bluegreen_flow
from .healthcheck import run_health, health_score
from .switcher import get_deploy_state, switch_active_port

try:
    from . import canary  # noqa: F401
except ImportError:
    canary = None  # type: ignore

__all__ = [
    "run_bluegreen_flow",
    "get_deploy_state",
    "run_health",
    "health_score",
    "switch_active_port",
    "canary",
]
