"""Estrategias de opciones (F1 scaffold)."""

from .vertical_spread import VerticalSpreadStrategy
from .iron_condor import IronCondorStrategy
from .iron_butterfly import IronButterflyStrategy
from .straddle_strangle import StraddleStrangleStrategy

__all__ = [
    "VerticalSpreadStrategy",
    "IronCondorStrategy",
    "IronButterflyStrategy",
    "StraddleStrangleStrategy",
]
