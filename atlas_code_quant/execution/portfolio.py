"""Atlas Code-Quant — Gestión de portfolio y posiciones."""
from __future__ import annotations

import logging
from dataclasses import dataclass, field
from datetime import datetime
from typing import Dict

logger = logging.getLogger("quant.execution.portfolio")


@dataclass
class Position:
    symbol: str
    side: str           # "long" | "short"
    size: float         # unidades
    entry_price: float
    current_price: float = 0.0
    stop_loss: float | None = None
    take_profit: float | None = None
    opened_at: datetime = field(default_factory=datetime.now)

    @property
    def pnl(self) -> float:
        """PnL no realizado en términos del precio base."""
        if self.side == "long":
            return (self.current_price - self.entry_price) * self.size
        return (self.entry_price - self.current_price) * self.size

    @property
    def pnl_pct(self) -> float:
        if self.entry_price == 0:
            return 0.0
        return (self.pnl / (self.entry_price * self.size)) * 100

    def to_dict(self) -> dict:
        return {
            "symbol": self.symbol,
            "side": self.side,
            "size": self.size,
            "entry_price": self.entry_price,
            "current_price": self.current_price,
            "pnl": round(self.pnl, 4),
            "pnl_pct": round(self.pnl_pct, 2),
            "stop_loss": self.stop_loss,
            "take_profit": self.take_profit,
            "opened_at": self.opened_at.isoformat(),
        }


class Portfolio:
    """Gestión de capital, posiciones y riesgo en tiempo real.

    Args:
        initial_capital: Capital inicial en USD.
        max_position_pct: Porcentaje máximo del capital por posición.
        max_drawdown_pct: Drawdown máximo permitido antes de pausar trading.

    Example::
        p = Portfolio(initial_capital=10_000)
        p.open_position("BTC/USDT", "long", size=0.1, entry_price=65000)
        print(p.total_pnl)
    """

    def __init__(
        self,
        initial_capital: float = 10_000.0,
        max_position_pct: float = 0.05,
        max_drawdown_pct: float = 0.15,
    ) -> None:
        self.initial_capital = initial_capital
        self.capital = initial_capital
        self.max_position_pct = max_position_pct
        self.max_drawdown_pct = max_drawdown_pct
        self.positions: Dict[str, Position] = {}
        self._peak_capital = initial_capital

    def position_size(self, price: float) -> float:
        """Calcula el tamaño máximo de posición en unidades."""
        max_usd = self.capital * self.max_position_pct
        return max_usd / price if price > 0 else 0.0

    def open_position(
        self,
        symbol: str,
        side: str,
        size: float,
        entry_price: float,
        stop_loss: float | None = None,
        take_profit: float | None = None,
    ) -> Position | None:
        """Abre una posición si hay capital y riesgo disponible.

        Returns:
            Position abierta o None si el riesgo es excesivo.
        """
        if self._drawdown_exceeded():
            logger.warning("Drawdown máximo alcanzado — posición bloqueada")
            return None
        if symbol in self.positions:
            logger.warning("Ya existe posición abierta para %s", symbol)
            return None
        cost = size * entry_price
        if cost > self.capital:
            logger.warning("Capital insuficiente para abrir %s", symbol)
            return None
        self.capital -= cost
        pos = Position(symbol, side, size, entry_price, entry_price, stop_loss, take_profit)
        self.positions[symbol] = pos
        logger.info("Posición abierta: %s %s x%.4f @ %.4f", side, symbol, size, entry_price)
        return pos

    def close_position(self, symbol: str, exit_price: float) -> float:
        """Cierra posición y retorna el PnL realizado."""
        pos = self.positions.pop(symbol, None)
        if not pos:
            return 0.0
        pos.current_price = exit_price
        realized = pos.pnl
        self.capital += pos.size * exit_price
        self._peak_capital = max(self._peak_capital, self.capital)
        logger.info("Posición cerrada: %s PnL=%.4f (%.2f%%)", symbol, realized, pos.pnl_pct)
        return realized

    def update_prices(self, prices: dict[str, float]) -> None:
        """Actualiza precios actuales de todas las posiciones."""
        for symbol, price in prices.items():
            if symbol in self.positions:
                self.positions[symbol].current_price = price

    @property
    def total_pnl(self) -> float:
        return sum(p.pnl for p in self.positions.values())

    @property
    def total_equity(self) -> float:
        return self.capital + sum(p.size * p.current_price for p in self.positions.values())

    def _drawdown_exceeded(self) -> bool:
        dd = (self._peak_capital - self.total_equity) / self._peak_capital
        return dd > self.max_drawdown_pct

    def summary(self) -> dict:
        return {
            "initial_capital": self.initial_capital,
            "current_equity": round(self.total_equity, 2),
            "free_capital": round(self.capital, 2),
            "open_positions": len(self.positions),
            "total_pnl": round(self.total_pnl, 4),
            "drawdown_pct": round(
                (self._peak_capital - self.total_equity) / self._peak_capital * 100, 2
            ),
            "positions": [p.to_dict() for p in self.positions.values()],
        }
