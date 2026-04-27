"""Estados base para el orquestador autónomo (F1)."""
from __future__ import annotations

from enum import Enum


class QuantAutonomyState(str, Enum):
    BOOTING = "BOOTING"
    SCANNING = "SCANNING"
    DEGRADED = "DEGRADED"
    KILL_SWITCH = "KILL_SWITCH"
