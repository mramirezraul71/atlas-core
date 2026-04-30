"""Atlas Code-Quant — Strategy Layer.

Módulo 4: Signal generation, visual triggers, entry/exit logic.
"""
from .signal_generator import SignalGenerator, TradeSignal, SignalType
from .visual_triggers import VisualTriggerValidator

__all__ = [
    "SignalGenerator",
    "TradeSignal",
    "SignalType",
    "VisualTriggerValidator",
]
