"""
Atlas Options Brain – Módulo de opciones para Atlas (Atlas Code Quant extension).
Fase 1: Modelo de datos + DSL de estrategias + Proveedores de datos.
Fase 2 (slice): simulador paper mínimo en `simulator`.
Fase 3 (slice): adapter Atlas en `integration`.
"""
from .models import OptionContract, OptionsChain, OptionType, OptionRight, Greeks, Leg
from .dsl import (
    OptionsStrategy, IronCondor,
    BullPutSpread, BearCallSpread,
    BullCallSpread, BearPutSpread,
    CoveredCall,
)
from .providers import (
    OptionsDataProvider,
    TradierProvider,
    PolygonProvider,
    YFinanceProvider,
)
from .simulator import PaperSimulator, Position, PositionSnapshot
from .integration import AtlasOptionsClient, StrategyType
from .broker import (
    LiveOrder,
    LiveOrderLeg,
    TradierLiveExecutionSink,
    TradierMultilegType,
    TradierOrderBuilder,
    TradierOrderExecutor,
)

__version__ = "0.1.0"
__all__ = [
    "OptionContract", "OptionsChain", "OptionType", "OptionRight", "Greeks", "Leg",
    "OptionsStrategy", "IronCondor",
    "BullPutSpread", "BearCallSpread",
    "BullCallSpread", "BearPutSpread",
    "CoveredCall",
    "OptionsDataProvider", "TradierProvider", "PolygonProvider", "YFinanceProvider",
    "PaperSimulator", "Position", "PositionSnapshot",
    "AtlasOptionsClient", "StrategyType",
    "LiveOrder",
    "LiveOrderLeg",
    "TradierLiveExecutionSink",
    "TradierMultilegType",
    "TradierOrderBuilder",
    "TradierOrderExecutor",
]
