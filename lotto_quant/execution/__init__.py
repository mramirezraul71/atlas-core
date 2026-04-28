"""
Execution subsystem for Atlas Lotto-Quant.

Two modes:
  - PAPER: simulated broker; positions persisted to DuckDB; P&L from prize draws.
  - LIVE:  manual-confirmation broker (a real human still buys lottery tickets);
           the system records intent and outcome but never auto-executes.

Both modes feed the same metrics layer so the dashboard renders identically.
"""

from .modes import OperatingMode, get_active_mode, set_active_mode
from .broker import PaperBroker, LiveBroker, BrokerBase, Order, Fill
from .pnl import PnLTracker, PnLSnapshot

__all__ = [
    "OperatingMode",
    "get_active_mode",
    "set_active_mode",
    "BrokerBase",
    "PaperBroker",
    "LiveBroker",
    "Order",
    "Fill",
    "PnLTracker",
    "PnLSnapshot",
]
