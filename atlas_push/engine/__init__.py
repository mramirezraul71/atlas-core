"""atlas_push.engine — motor de decisión de Atlas Push.

Contiene :class:`DecisionEngine` y los Protocols :class:`Strategy` y
:class:`RiskManager` que definen sus puntos de extensión.

Ver ``docs/atlas_push/ARCHITECTURE.md`` §3.3 y §4. En el paso D el
motor es un **scaffold pass-through**: ``decide(state)`` devuelve
siempre ``DecisionOutput.empty()``. Las estrategias y la capa de
riesgo reales se introducen en pasos posteriores.

Regla de oro: este módulo no importa símbolos del brain core mayor de
ATLAS (``brain_core``, ``mission_manager``, ``safety_kernel``,
``state_bus``, ``arbitration``, ``policy_store``,
``modules.command_router``).

Invariante de hermanos: este módulo no importa ``atlas_push.intents``
(y viceversa). :class:`DecisionEngine` e :class:`IntentRouter` son
piezas hermanas, no anidadas.
"""

from atlas_push.engine.decision_engine import (
    DecisionEngine,
    RiskManager,
    Strategy,
    StrategyProposal,
)

__all__ = [
    "DecisionEngine",
    "RiskManager",
    "Strategy",
    "StrategyProposal",
]
