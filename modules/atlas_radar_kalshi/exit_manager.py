"""
exit_manager.py — Gestión disciplinada de salidas.

Reglas:

- **Take-profit** cuando el precio alcanza el objetivo (captura del
  edge, configurable ``tp_capture_pct`` del edge inicial).
- **Stop-loss** si el ensemble cambia de lado o el edge se invierte
  (``sl_edge_revert``) o si el precio se mueve N¢ en contra
  (``sl_ticks``).
- **Time-stop** si transcurren ``time_stop_seconds`` sin alcanzar TP.
- **Forced-exit** si:
    * datos stale / latencia degradada (señal del scanner),
    * circuit breaker activo,
    * kill-switch.
"""
from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Optional

from pydantic import BaseModel, Field


class ExitConfig(BaseModel):
    tp_capture_pct: float = 0.6      # capturar 60% del edge
    sl_ticks: int = 4
    sl_edge_revert: float = -0.02    # edge inverso al inicial
    time_stop_seconds: int = 1800    # 30 min default


class Position(BaseModel):
    ticker: str
    side: str
    entry_price: int
    size: int
    entry_ts: float
    edge_at_entry: float
    target_price: int
    stop_price: int
    order_id: Optional[str] = None
    closed: bool = False


class ExitSignal(BaseModel):
    should_exit: bool
    reason: str = ""
    target_price: Optional[int] = None  # precio sugerido para limit close


# ===========================================================================
class ExitManager:
    """Mantiene posiciones abiertas y produce señales de salida."""

    def __init__(self, cfg: Optional[ExitConfig] = None) -> None:
        self.cfg = cfg or ExitConfig()
        self.positions: dict[str, Position] = {}

    # ------------------------------------------------------------------
    def open(self, pos: Position) -> None:
        self.positions[pos.ticker] = pos

    def close(self, ticker: str) -> Optional[Position]:
        p = self.positions.pop(ticker, None)
        if p:
            p.closed = True
        return p

    def list_open(self) -> list[Position]:
        return [p for p in self.positions.values() if not p.closed]

    # ------------------------------------------------------------------
    def evaluate(
        self,
        ticker: str,
        current_price: int,
        current_edge: float,
        data_degraded: bool = False,
        forced: bool = False,
    ) -> ExitSignal:
        pos = self.positions.get(ticker)
        if not pos or pos.closed:
            return ExitSignal(should_exit=False)
        cfg = self.cfg

        # 1) Forced exits
        if forced:
            return ExitSignal(should_exit=True, reason="forced",
                              target_price=current_price)
        if data_degraded:
            return ExitSignal(should_exit=True, reason="data_degraded",
                              target_price=current_price)

        # 2) Take-profit (precio cruza target en favor)
        gain = (current_price - pos.entry_price) if pos.side == "YES" \
            else (pos.entry_price - current_price)
        if pos.side == "YES" and current_price >= pos.target_price:
            return ExitSignal(should_exit=True, reason="take_profit",
                              target_price=pos.target_price)
        if pos.side == "NO" and current_price <= pos.target_price:
            return ExitSignal(should_exit=True, reason="take_profit",
                              target_price=pos.target_price)

        # 3) Stop-loss por ticks
        if -gain >= cfg.sl_ticks:
            return ExitSignal(should_exit=True, reason="stop_loss_ticks",
                              target_price=current_price)

        # 4) Edge revertido
        if pos.side == "YES" and current_edge <= cfg.sl_edge_revert:
            return ExitSignal(should_exit=True, reason="edge_revert",
                              target_price=current_price)
        if pos.side == "NO" and -current_edge <= cfg.sl_edge_revert:
            return ExitSignal(should_exit=True, reason="edge_revert",
                              target_price=current_price)

        # 5) Time-stop
        if time.time() - pos.entry_ts >= cfg.time_stop_seconds:
            return ExitSignal(should_exit=True, reason="time_stop",
                              target_price=current_price)

        return ExitSignal(should_exit=False)

    # ------------------------------------------------------------------
    @staticmethod
    def build_targets(side: str, entry_price: int, p_fair: float,
                      tp_capture_pct: float, sl_ticks: int) -> tuple[int, int]:
        """Calcula target y stop al abrir."""
        fair_cents = int(round(p_fair * 100))
        if side == "YES":
            tp = entry_price + max(1, int((fair_cents - entry_price) * tp_capture_pct))
            tp = min(99, max(entry_price + 1, tp))
            sl = max(1, entry_price - sl_ticks)
        else:
            tp = entry_price - max(1, int((entry_price - (100 - fair_cents)) * tp_capture_pct))
            tp = max(1, min(entry_price - 1, tp))
            sl = min(99, entry_price + sl_ticks)
        return tp, sl
