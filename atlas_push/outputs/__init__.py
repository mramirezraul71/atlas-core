"""atlas_push.outputs — tipos de salida del cerebro de Atlas Push.

Contiene :class:`DecisionOutput` y las value types que lo componen
(:class:`LogicalOrder`, :class:`TargetWeight`, :class:`RiskVeto`).

Todos los tipos son dataclasses inmutables (``frozen=True``) y forman
el contrato de salida del :class:`atlas_push.engine.DecisionEngine`.
Ver ``docs/atlas_push/ARCHITECTURE.md`` §3.2.

Regla de oro: este módulo no importa símbolos del brain core mayor de
ATLAS (``brain_core``, ``mission_manager``, ``safety_kernel``,
``state_bus``, ``arbitration``, ``policy_store``,
``modules.command_router``).
"""

from atlas_push.outputs.decision_output import (
    DecisionOutput,
    LogicalOrder,
    RiskVeto,
    TargetWeight,
)

__all__ = [
    "DecisionOutput",
    "LogicalOrder",
    "RiskVeto",
    "TargetWeight",
]
