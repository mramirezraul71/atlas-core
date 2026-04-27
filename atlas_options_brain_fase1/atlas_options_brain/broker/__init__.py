"""Broker adapters (live / Tradier) — estructuras y ejecutor sandbox.

.. legacy:: F4 PHASE1
    Paquete legacy/PHASE1 conservado para tests históricos. NO es la ruta
    de órdenes canónica de Atlas Code Quant. Stack canónico:
    ``atlas_code_quant.execution.tradier_execution`` (+ ``tradier_controls`` /
    ``tradier_pdt_ledger``).

    Ver ``docs/ATLAS_CODE_QUANT_F4_TRADIER_CANONICALIZATION.md`` y
    ``atlas_code_quant/execution/README_TRADIER.md``.
"""

from .tradier_executor import TradierMultilegType, TradierOrderExecutor
from .tradier_live import (
    LiveOrder,
    LiveOrderLeg,
    TradierLiveExecutionSink,
    TradierOrderBuilder,
)

__all__ = [
    "LiveOrder",
    "LiveOrderLeg",
    "TradierLiveExecutionSink",
    "TradierOrderBuilder",
    "TradierMultilegType",
    "TradierOrderExecutor",
]
