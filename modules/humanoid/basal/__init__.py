"""
Basal Ganglia Module: Seleccion e inhibicion de acciones.

Implementa el proceso Go/NoGo de los ganglios basales:
- ActionSelector: Seleccion entre candidatos de accion
- Inhibitor: Control inhibitorio de acciones
"""
from .action_selector import (
    ActionSelector,
    ActionCandidate,
    ActionType,
    SelectionContext,
    SelectionResult,
    SelectionStrategy,
)
from .inhibitor import (
    Inhibitor,
    InhibitionRule,
    InhibitionVerdict,
    InhibitionLevel,
    InhibitionSource,
    Override,
)

__all__ = [
    # Action Selector
    "ActionSelector",
    "ActionCandidate",
    "ActionType",
    "SelectionContext",
    "SelectionResult",
    "SelectionStrategy",
    # Inhibitor
    "Inhibitor",
    "InhibitionRule",
    "InhibitionVerdict",
    "InhibitionLevel",
    "InhibitionSource",
    "Override",
]
