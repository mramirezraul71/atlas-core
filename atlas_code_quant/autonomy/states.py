"""FSM 14 estados — F8.

Compatibilidad F1: ``QuantAutonomyState`` mantiene su nombre y los 4 valores
originales (BOOTING, SCANNING, DEGRADED, KILL_SWITCH) y añade los 10 nuevos.
"""
from __future__ import annotations

from enum import Enum


class QuantAutonomyState(str, Enum):
    # F1 originales
    BOOTING = "BOOTING"
    DEGRADED = "DEGRADED"
    SCANNING = "SCANNING"
    KILL_SWITCH = "KILL_SWITCH"
    # F8 nuevos
    OPPORTUNITY_DETECTED = "OPPORTUNITY_DETECTED"
    STRATEGY_BUILDING = "STRATEGY_BUILDING"
    BACKTESTING = "BACKTESTING"
    PAPER_READY = "PAPER_READY"
    PAPER_EXECUTING = "PAPER_EXECUTING"
    LIVE_ARMED = "LIVE_ARMED"
    LIVE_EXECUTING = "LIVE_EXECUTING"
    MONITORING = "MONITORING"
    EXITING = "EXITING"
    ERROR_RECOVERY = "ERROR_RECOVERY"


# Diagrama de transiciones permitidas. Mapa adyacencia: estado -> set de
# estados a los que puede ir directamente. KILL_SWITCH es absorbente salvo
# por reset operacional explícito (recuperación a BOOTING).
ALLOWED_TRANSITIONS: dict[QuantAutonomyState, set[QuantAutonomyState]] = {
    QuantAutonomyState.BOOTING: {
        QuantAutonomyState.DEGRADED,
        QuantAutonomyState.SCANNING,
        QuantAutonomyState.KILL_SWITCH,
        QuantAutonomyState.ERROR_RECOVERY,
    },
    QuantAutonomyState.DEGRADED: {
        QuantAutonomyState.SCANNING,
        QuantAutonomyState.BOOTING,
        QuantAutonomyState.KILL_SWITCH,
    },
    QuantAutonomyState.SCANNING: {
        QuantAutonomyState.OPPORTUNITY_DETECTED,
        QuantAutonomyState.DEGRADED,
        QuantAutonomyState.MONITORING,
        QuantAutonomyState.KILL_SWITCH,
    },
    QuantAutonomyState.OPPORTUNITY_DETECTED: {
        QuantAutonomyState.STRATEGY_BUILDING,
        QuantAutonomyState.SCANNING,
        QuantAutonomyState.KILL_SWITCH,
    },
    QuantAutonomyState.STRATEGY_BUILDING: {
        QuantAutonomyState.BACKTESTING,
        QuantAutonomyState.SCANNING,
        QuantAutonomyState.KILL_SWITCH,
        QuantAutonomyState.ERROR_RECOVERY,
    },
    QuantAutonomyState.BACKTESTING: {
        QuantAutonomyState.PAPER_READY,
        QuantAutonomyState.SCANNING,
        QuantAutonomyState.KILL_SWITCH,
        QuantAutonomyState.ERROR_RECOVERY,
    },
    QuantAutonomyState.PAPER_READY: {
        QuantAutonomyState.PAPER_EXECUTING,
        QuantAutonomyState.SCANNING,
        QuantAutonomyState.KILL_SWITCH,
    },
    QuantAutonomyState.PAPER_EXECUTING: {
        QuantAutonomyState.MONITORING,
        QuantAutonomyState.LIVE_ARMED,
        QuantAutonomyState.EXITING,
        QuantAutonomyState.KILL_SWITCH,
        QuantAutonomyState.ERROR_RECOVERY,
    },
    QuantAutonomyState.LIVE_ARMED: {
        QuantAutonomyState.LIVE_EXECUTING,
        QuantAutonomyState.PAPER_EXECUTING,
        QuantAutonomyState.KILL_SWITCH,
    },
    QuantAutonomyState.LIVE_EXECUTING: {
        QuantAutonomyState.MONITORING,
        QuantAutonomyState.EXITING,
        QuantAutonomyState.KILL_SWITCH,
        QuantAutonomyState.ERROR_RECOVERY,
    },
    QuantAutonomyState.MONITORING: {
        QuantAutonomyState.EXITING,
        QuantAutonomyState.SCANNING,
        QuantAutonomyState.LIVE_EXECUTING,
        QuantAutonomyState.PAPER_EXECUTING,
        QuantAutonomyState.KILL_SWITCH,
    },
    QuantAutonomyState.EXITING: {
        QuantAutonomyState.SCANNING,
        QuantAutonomyState.MONITORING,
        QuantAutonomyState.KILL_SWITCH,
        QuantAutonomyState.ERROR_RECOVERY,
    },
    QuantAutonomyState.ERROR_RECOVERY: {
        QuantAutonomyState.SCANNING,
        QuantAutonomyState.DEGRADED,
        QuantAutonomyState.KILL_SWITCH,
        QuantAutonomyState.BOOTING,
    },
    QuantAutonomyState.KILL_SWITCH: {
        # Solo reset operacional manual (vuelve a boot). Se valida con flag.
        QuantAutonomyState.BOOTING,
    },
}


def is_allowed(src: QuantAutonomyState, dst: QuantAutonomyState) -> bool:
    return dst in ALLOWED_TRANSITIONS.get(src, set())
