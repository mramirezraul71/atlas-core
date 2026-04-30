"""atlas_push.state — tipos de entrada al cerebro de Atlas Push.

Contiene :class:`MarketState` y las value types que lo componen
(:class:`AccountSnapshot`, :class:`Position`, :class:`Quote`,
:class:`RiskContext`).

Todos los tipos son dataclasses inmutables (``frozen=True``) y forman el
contrato de entrada al :class:`atlas_push.engine.DecisionEngine`. Ver
``docs/atlas_push/ARCHITECTURE.md`` §3.1.

Regla de oro: este módulo no importa símbolos del brain core mayor de
ATLAS (``brain_core``, ``mission_manager``, ``safety_kernel``,
``state_bus``, ``arbitration``, ``policy_store``,
``modules.command_router``).
"""

from atlas_push.state.market_state import (
    AccountSnapshot,
    MarketState,
    Position,
    Quote,
    RiskContext,
)

__all__ = [
    "AccountSnapshot",
    "MarketState",
    "Position",
    "Quote",
    "RiskContext",
]
