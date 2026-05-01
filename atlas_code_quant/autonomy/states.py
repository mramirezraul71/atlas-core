"""Atlas Code Quant — Autonomy FSM states (F17).

Define los 14 estados canónicos del orquestador autónomo descritos en
la sección 11 del estudio maestro (`docs/Atlas plan maestro 2.txt`,
v9). Esta enum es **intencionalmente exhaustiva** — incluye los
estados ``LIVE_ARMED`` y ``LIVE_EXECUTING`` que NO son alcanzables
en F17–F20: la FSM los conoce nominalmente pero no permite ninguna
transición hacia ellos en este bloque.

Reglas duras (F17):

* La FSM opera **únicamente sobre el pipeline paper (F16)**.
* No se permiten transiciones a ``LIVE_ARMED`` ni
  ``LIVE_EXECUTING`` en este bloque (F17–F20). Cualquier intento se
  rechaza explícitamente con un ``IllegalTransition`` y se queda en
  el estado actual.
* No tocar locks globales (``paper_only``,
  ``full_live_globally_locked``).
"""

from __future__ import annotations

from enum import Enum


class AutonomyState(str, Enum):
    """Estados canónicos del orquestador autónomo Atlas (F17)."""

    BOOTING = "BOOTING"
    DEGRADED = "DEGRADED"
    SCANNING = "SCANNING"
    OPPORTUNITY_DETECTED = "OPPORTUNITY_DETECTED"
    STRATEGY_BUILDING = "STRATEGY_BUILDING"
    BACKTESTING = "BACKTESTING"
    PAPER_READY = "PAPER_READY"
    PAPER_EXECUTING = "PAPER_EXECUTING"
    MONITORING = "MONITORING"
    EXITING = "EXITING"
    LIVE_ARMED = "LIVE_ARMED"  # NO alcanzable en F17–F20
    LIVE_EXECUTING = "LIVE_EXECUTING"  # NO alcanzable en F17–F20
    KILL_SWITCH = "KILL_SWITCH"
    ERROR_RECOVERY = "ERROR_RECOVERY"


# Estados que F17–F20 NO permite alcanzar bajo ninguna circunstancia.
LIVE_FORBIDDEN_STATES: frozenset[AutonomyState] = frozenset(
    {AutonomyState.LIVE_ARMED, AutonomyState.LIVE_EXECUTING}
)


# Transiciones canónicas del estudio maestro v9 §11. Cada estado
# tiene un conjunto cerrado de destinos legales. Cualquier transición
# fuera de este mapa se considera ``IllegalTransition``.
ALLOWED_TRANSITIONS: dict[AutonomyState, frozenset[AutonomyState]] = {
    AutonomyState.BOOTING: frozenset(
        {
            AutonomyState.SCANNING,
            AutonomyState.DEGRADED,
            AutonomyState.KILL_SWITCH,
            AutonomyState.ERROR_RECOVERY,
        }
    ),
    AutonomyState.DEGRADED: frozenset(
        {
            AutonomyState.SCANNING,
            AutonomyState.BOOTING,
            AutonomyState.KILL_SWITCH,
            AutonomyState.ERROR_RECOVERY,
        }
    ),
    AutonomyState.SCANNING: frozenset(
        {
            AutonomyState.OPPORTUNITY_DETECTED,
            AutonomyState.DEGRADED,
            AutonomyState.KILL_SWITCH,
            AutonomyState.ERROR_RECOVERY,
            AutonomyState.SCANNING,  # auto-loop sin oportunidad
        }
    ),
    AutonomyState.OPPORTUNITY_DETECTED: frozenset(
        {
            AutonomyState.STRATEGY_BUILDING,
            AutonomyState.SCANNING,
            AutonomyState.DEGRADED,
            AutonomyState.KILL_SWITCH,
            AutonomyState.ERROR_RECOVERY,
        }
    ),
    AutonomyState.STRATEGY_BUILDING: frozenset(
        {
            AutonomyState.BACKTESTING,
            AutonomyState.SCANNING,
            AutonomyState.DEGRADED,
            AutonomyState.KILL_SWITCH,
            AutonomyState.ERROR_RECOVERY,
        }
    ),
    AutonomyState.BACKTESTING: frozenset(
        {
            AutonomyState.PAPER_READY,
            AutonomyState.SCANNING,
            AutonomyState.DEGRADED,
            AutonomyState.KILL_SWITCH,
            AutonomyState.ERROR_RECOVERY,
        }
    ),
    AutonomyState.PAPER_READY: frozenset(
        {
            AutonomyState.PAPER_EXECUTING,
            AutonomyState.STRATEGY_BUILDING,  # vision_delay
            AutonomyState.EXITING,  # vision force_exit / risk fail
            AutonomyState.SCANNING,
            AutonomyState.DEGRADED,
            AutonomyState.KILL_SWITCH,
            AutonomyState.ERROR_RECOVERY,
            # NOTA: LIVE_ARMED es destino legal del estudio pero
            # F17–F20 lo bloquea desde el orquestador.
        }
    ),
    AutonomyState.PAPER_EXECUTING: frozenset(
        {
            AutonomyState.MONITORING,
            AutonomyState.EXITING,
            AutonomyState.DEGRADED,
            AutonomyState.KILL_SWITCH,
            AutonomyState.ERROR_RECOVERY,
        }
    ),
    AutonomyState.MONITORING: frozenset(
        {
            AutonomyState.EXITING,
            AutonomyState.MONITORING,
            AutonomyState.DEGRADED,
            AutonomyState.KILL_SWITCH,
            AutonomyState.ERROR_RECOVERY,
        }
    ),
    AutonomyState.EXITING: frozenset(
        {
            AutonomyState.SCANNING,
            AutonomyState.DEGRADED,
            AutonomyState.KILL_SWITCH,
            AutonomyState.ERROR_RECOVERY,
        }
    ),
    AutonomyState.LIVE_ARMED: frozenset(
        {
            AutonomyState.LIVE_EXECUTING,
            AutonomyState.PAPER_READY,
            AutonomyState.EXITING,
            AutonomyState.KILL_SWITCH,
            AutonomyState.ERROR_RECOVERY,
        }
    ),
    AutonomyState.LIVE_EXECUTING: frozenset(
        {
            AutonomyState.MONITORING,
            AutonomyState.EXITING,
            AutonomyState.KILL_SWITCH,
            AutonomyState.ERROR_RECOVERY,
        }
    ),
    AutonomyState.KILL_SWITCH: frozenset(
        {AutonomyState.ERROR_RECOVERY}
    ),
    AutonomyState.ERROR_RECOVERY: frozenset(
        {AutonomyState.BOOTING, AutonomyState.KILL_SWITCH}
    ),
}


class IllegalTransition(Exception):
    """Se lanza cuando se solicita una transición no permitida.

    El orquestador F17 captura esta excepción internamente para
    mantener la garantía defensiva (``step()`` nunca lanza), pero
    los tests pueden detectarla cuando llaman ``assert_transition``
    directamente.
    """


def assert_transition(
    src: AutonomyState, dst: AutonomyState, *, allow_live: bool = False
) -> None:
    """Valida que ``src → dst`` sea legal.

    Si ``allow_live`` es ``False`` (default en F17–F20), las
    transiciones a ``LIVE_ARMED`` o ``LIVE_EXECUTING`` se rechazan
    aun cuando el estudio las contemple.
    """
    if not allow_live and dst in LIVE_FORBIDDEN_STATES:
        raise IllegalTransition(
            f"transición a estado live bloqueada en F17–F20: {src} → {dst}"
        )
    legal = ALLOWED_TRANSITIONS.get(src, frozenset())
    if dst not in legal:
        raise IllegalTransition(
            f"transición ilegal según estudio maestro §11: {src} → {dst}"
        )


__all__ = [
    "AutonomyState",
    "LIVE_FORBIDDEN_STATES",
    "ALLOWED_TRANSITIONS",
    "IllegalTransition",
    "assert_transition",
]
