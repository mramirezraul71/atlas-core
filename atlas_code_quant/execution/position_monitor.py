"""Position Monitor — F9.

Evalúa posiciones :class:`PaperOpenPosition` contra ticks de precio y devuelve
una decisión estructurada de cierre:

- ``take_profit``: precio actual >= TP
- ``stop_loss``: precio actual <= SL
- ``time_stop``: ``now >= time_stop_at``
- ``hold``: ninguna condición se cumple

El monitor es síncrono y stateless: no conoce el broker. Quien lo usa decide
cuándo llamar :meth:`PaperBroker.close_position`.
"""
from __future__ import annotations

import time
from dataclasses import dataclass

from atlas_code_quant.execution.paper_broker import PaperOpenPosition


@dataclass(slots=True)
class MonitorDecision:
    """Decisión de monitoreo para una posición tras un tick."""

    position_id: str
    action: str  # hold | close
    reason: str  # take_profit | stop_loss | time_stop | hold
    last_price: float
    suggested_exit_price: float
    elapsed_seconds: float


class PositionMonitor:
    """Evalúa una posición open contra el último precio observado."""

    def evaluate(
        self,
        position: PaperOpenPosition,
        *,
        last_price: float,
        now: float | None = None,
    ) -> MonitorDecision:
        if position.status != "open":
            return MonitorDecision(
                position_id=position.position_id,
                action="hold",
                reason="already_closed",
                last_price=last_price,
                suggested_exit_price=last_price,
                elapsed_seconds=0.0,
            )

        ts_now = now if now is not None else time.time()
        elapsed = ts_now - position.opened_at

        # Take Profit
        if last_price >= position.take_profit_price:
            return MonitorDecision(
                position_id=position.position_id,
                action="close",
                reason="take_profit",
                last_price=last_price,
                suggested_exit_price=position.take_profit_price,
                elapsed_seconds=elapsed,
            )

        # Stop Loss
        if last_price <= position.stop_loss_price:
            return MonitorDecision(
                position_id=position.position_id,
                action="close",
                reason="stop_loss",
                last_price=last_price,
                suggested_exit_price=position.stop_loss_price,
                elapsed_seconds=elapsed,
            )

        # Time stop
        if ts_now >= position.time_stop_at:
            return MonitorDecision(
                position_id=position.position_id,
                action="close",
                reason="time_stop",
                last_price=last_price,
                suggested_exit_price=last_price,
                elapsed_seconds=elapsed,
            )

        return MonitorDecision(
            position_id=position.position_id,
            action="hold",
            reason="hold",
            last_price=last_price,
            suggested_exit_price=last_price,
            elapsed_seconds=elapsed,
        )
