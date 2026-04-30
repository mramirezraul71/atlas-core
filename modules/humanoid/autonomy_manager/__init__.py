"""ATLAS Autonomy Manager.

Runtime knowledge model + policy engine + planner + bounded execution loop
for internal self-detection and self-correction.
"""

from .daemon import get_autonomy_manager_daemon
from .manager import get_latest_status, run_cycle

__all__ = ["get_autonomy_manager_daemon", "get_latest_status", "run_cycle"]
