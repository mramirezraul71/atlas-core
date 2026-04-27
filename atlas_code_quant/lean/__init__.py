"""Integración LEAN external mode (F4 esqueleto).

Defaults seguros: ``ATLAS_LEAN_ENABLED=false``. Sin subprocess sin opt-in.
"""

from .config import LeanConfig, LeanMode
from .launcher import LeanLauncher, LeanRunResult, LeanBacktestParams
from .parser import (
    EquityPoint,
    LeanOrder,
    fitness_from_statistics,
    parse_equity_curve,
    parse_orders,
    parse_statistics,
)

__all__ = [
    "LeanConfig",
    "LeanMode",
    "LeanLauncher",
    "LeanRunResult",
    "LeanBacktestParams",
    "EquityPoint",
    "LeanOrder",
    "parse_statistics",
    "parse_orders",
    "parse_equity_curve",
    "fitness_from_statistics",
]
