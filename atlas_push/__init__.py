"""atlas_push — núcleo de Atlas Push.

Paquete que alberga el cerebro de Atlas Push. Contenido actual
(tras el Paso D):

- ``atlas_push.intents``: ``IntentRouter`` y ``IntentResult``.
- ``atlas_push.engine``: ``DecisionEngine`` (scaffold pass-through) y
  Protocols ``Strategy`` / ``RiskManager``.
- ``atlas_push.state``: ``MarketState`` y value types asociados
  (``AccountSnapshot``, ``Position``, ``Quote``, ``RiskContext``).
- ``atlas_push.outputs``: ``DecisionOutput`` y value types asociados
  (``LogicalOrder``, ``TargetWeight``, ``RiskVeto``).

Próximos pasos (no implementados aún):

- ``atlas_push.strategies``: ``Strategy.propose`` (más adelante).
- ``atlas_push.risk``: ``RiskManager.apply`` (más adelante).
- ``atlas_push.ports``: Protocols hacia sistemas externos (Paso E).
- ``atlas_push.config``: carga de configuración (Paso E).

Invariantes:

- Regla de oro: ``atlas_push`` no importa símbolos del brain core
  mayor de ATLAS (``brain_core``, ``mission_manager``,
  ``safety_kernel``, ``state_bus``, ``arbitration``,
  ``policy_store``, ``modules.command_router``). Los contratos viajan
  vía ``typing.Protocol``.
- Hermanos, no anidados: ``atlas_push.intents`` y ``atlas_push.engine``
  son piezas hermanas. ``IntentRouter`` no importa ``DecisionEngine``
  y viceversa.
"""

from atlas_push.engine import DecisionEngine, RiskManager, Strategy
from atlas_push.intents import IntentResult, IntentRouter, Kind
from atlas_push.outputs import (
    DecisionOutput,
    LogicalOrder,
    RiskVeto,
    TargetWeight,
)
from atlas_push.state import (
    AccountSnapshot,
    MarketState,
    Position,
    Quote,
    RiskContext,
)

__all__ = [
    # intents
    "IntentResult",
    "IntentRouter",
    "Kind",
    # engine
    "DecisionEngine",
    "RiskManager",
    "Strategy",
    # state
    "AccountSnapshot",
    "MarketState",
    "Position",
    "Quote",
    "RiskContext",
    # outputs
    "DecisionOutput",
    "LogicalOrder",
    "RiskVeto",
    "TargetWeight",
]
