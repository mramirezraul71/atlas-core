"""Estrategias de opciones — contratos esqueleto F5.

Cada estrategia implementa ``build_plan(opportunity, config) -> StrategyPlan``
sobre un mismo contrato Pydantic ``StrategyPlan`` con ``legs`` tipadas. Las
estrategias devuelven planes coherentes pero **no** seleccionan strikes reales
(se hará en F5 fitness/F8 orchestrator).
"""

from .iron_butterfly import IronButterflyStrategy
from .iron_condor import IronCondorStrategy
from .straddle_strangle import StraddleStrangleStrategy
from .vertical_spread import VerticalSpreadStrategy

__all__ = [
    "IronButterflyStrategy",
    "IronCondorStrategy",
    "StraddleStrangleStrategy",
    "VerticalSpreadStrategy",
]
