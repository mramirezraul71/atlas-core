"""Atlas Code-Quant — Gestión de portfolio y posiciones.

v2 (logarítmico): Integra Kelly Criterion para position sizing óptimo
y circuit breaker mejorado con contador de pérdidas consecutivas.
Drawdown máximo elevado a 20% (recomendación Grok/xAI) con pausa
automática — no cierre forzado.
"""
from __future__ import annotations

import logging
import math
from dataclasses import dataclass, field
from datetime import datetime
from typing import Dict, Sequence

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
        """PnL no realizado."""
        if self.side == "long":
            return (self.current_price - self.entry_price) * self.size
        return (self.entry_price - self.current_price) * self.size

    @property
    def pnl_pct(self) -> float:
        if self.entry_price == 0:
            return 0.0
        return (self.pnl / (self.entry_price * self.size)) * 100

    @property
    def log_return(self) -> float:
        """Retorno logarítmico de la posición — más preciso para compounding."""
        if self.entry_price <= 0 or self.current_price <= 0:
            return 0.0
        if self.side == "long":
            return math.log(self.current_price / self.entry_price)
        return math.log(self.entry_price / self.current_price)

    def to_dict(self) -> dict:
        return {
            "symbol": self.symbol,
            "side": self.side,
            "size": self.size,
            "entry_price": self.entry_price,
            "current_price": self.current_price,
            "pnl": round(self.pnl, 4),
            "pnl_pct": round(self.pnl_pct, 2),
            "log_return": round(self.log_return, 6),
            "stop_loss": self.stop_loss,
            "take_profit": self.take_profit,
            "opened_at": self.opened_at.isoformat(),
        }


class Portfolio:
    """Gestión de capital, posiciones y riesgo en tiempo real.

    v2: Integra Kelly Criterion (opcional) y circuit breaker mejorado.

    Args:
        initial_capital: Capital inicial en USD.
        max_position_pct: Porcentaje máximo del capital por posición (fallback si no hay Kelly).
        max_drawdown_pct: Drawdown máximo (20% recomendado Grok) — pausa trading al superar.
        max_consecutive_losses: Nº de pérdidas consecutivas antes de reducir sizing 50%.
        kelly_sizer: Instancia opcional de KellySizer para sizing dinámico.

    Example::
        p = Portfolio(initial_capital=10_000, max_drawdown_pct=0.20)
        p.open_position("BTC/USDT", "long", size=0.1, entry_price=65000)
        print(p.total_log_return)
    """

    def __init__(
        self,
        initial_capital: float = 10_000.0,
        max_position_pct: float = 0.05,
        max_drawdown_pct: float = 0.20,
        max_consecutive_losses: int = 3,
        kelly_sizer=None,
    ) -> None:
        self.initial_capital = initial_capital
        self.capital = initial_capital
        self.max_position_pct = max_position_pct
        self.max_drawdown_pct = max_drawdown_pct
        self.max_consecutive_losses = max_consecutive_losses
        self.kelly_sizer = kelly_sizer
        self.positions: Dict[str, Position] = {}
        self._peak_capital = initial_capital
        self._consecutive_losses: int = 0
        self._circuit_breaker_active: bool = False
        self._realized_log_returns: list[float] = []

    # ── Position sizing ───────────────────────────────────────────────────────

    def position_size(
        self,
        price: float,
        symbol: str = "",
        strategy: str = "",
        pnl_history: Sequence[float] | None = None,
        atr: float | None = None,
    ) -> float:
        """Calcula el tamaño de posición en unidades.

        Si hay KellySizer configurado y se proveen PnLs históricos,
        usa Kelly risk-constrained. De lo contrario, usa max_position_pct.

        Args:
            price: Precio actual del activo.
            symbol: Ticker (para Kelly).
            strategy: Nombre de estrategia (para Kelly).
            pnl_history: Lista de PnLs realizados previos (para Kelly).
            atr: ATR actual (para stops dinámicos y risk check).
        """
        if price <= 0:
            return 0.0

        # Reducción por pérdidas consecutivas (half-size tras streak de pérdidas)
        size_multiplier = 0.5 if self._consecutive_losses >= self.max_consecutive_losses else 1.0

        if self.kelly_sizer and pnl_history and len(pnl_history) >= self.kelly_sizer.min_samples:
            result = self.kelly_sizer.compute(symbol, strategy, list(pnl_history))
            units = self.kelly_sizer.position_size(self.capital, price, result, atr)
            return units * size_multiplier

        # Fallback: porcentaje fijo del capital
        max_usd = self.capital * self.max_position_pct * size_multiplier
        return max_usd / price

    def atr_stops(
        self,
        price: float,
        atr: float,
        side: str = "long",
    ) -> tuple[float, float]:
        """Delega cálculo de stops ATR al KellySizer si está disponible."""
        if self.kelly_sizer:
            return self.kelly_sizer.atr_stops(price, atr, side)
        # Fallback: 1.5 ATR stop, 3 ATR TP
        sl = price - 1.5 * atr if side == "long" else price + 1.5 * atr
        tp = price + 3.0 * atr if side == "long" else price - 3.0 * atr
        return (sl, tp)

    # ── Lifecycle ─────────────────────────────────────────────────────────────

    def open_position(
        self,
        symbol: str,
        side: str,
        size: float,
        entry_price: float,
        stop_loss: float | None = None,
        take_profit: float | None = None,
    ) -> Position | None:
        """Abre una posición si hay capital y el circuit breaker no está activo."""
        if self._circuit_breaker_active:
            logger.warning("[CircuitBreaker] ACTIVO — posición bloqueada para %s", symbol)
            return None
        if self._drawdown_exceeded():
            self._circuit_breaker_active = True
            logger.warning("[CircuitBreaker] Drawdown %.1f%% superado — trading PAUSADO",
                           self.max_drawdown_pct * 100)
            return None
        if symbol in self.positions:
            logger.warning("Ya existe posición abierta para %s", symbol)
            return None
        cost = size * entry_price
        if cost > self.capital:
            logger.warning("Capital insuficiente para abrir %s (costo=%.2f capital=%.2f)",
                           symbol, cost, self.capital)
            return None
        self.capital -= cost
        pos = Position(symbol, side, size, entry_price, entry_price, stop_loss, take_profit)
        self.positions[symbol] = pos
        logger.info("Posición abierta: %s %s x%.4f @ %.4f", side, symbol, size, entry_price)
        return pos

    def close_position(self, symbol: str, exit_price: float) -> float:
        """Cierra posición, actualiza circuit breaker y retorna el PnL realizado."""
        pos = self.positions.pop(symbol, None)
        if not pos:
            return 0.0
        pos.current_price = exit_price
        realized = pos.pnl

        # Registrar retorno logarítmico realizado
        if pos.entry_price > 0 and exit_price > 0:
            if pos.side == "long":
                lr = math.log(exit_price / pos.entry_price)
            else:
                lr = math.log(pos.entry_price / exit_price)
            self._realized_log_returns.append(lr)

        self.capital += pos.size * exit_price
        self._peak_capital = max(self._peak_capital, self.capital)

        # Actualizar contador de pérdidas consecutivas
        if realized < 0:
            self._consecutive_losses += 1
            if self._consecutive_losses >= self.max_consecutive_losses:
                logger.warning(
                    "[CircuitBreaker] %d pérdidas consecutivas — reduciendo sizing al 50%%",
                    self._consecutive_losses,
                )
        else:
            self._consecutive_losses = 0  # Reset en ganancia

        # Reactivar circuit breaker si drawdown ya no supera límite
        if self._circuit_breaker_active and not self._drawdown_exceeded():
            self._circuit_breaker_active = False
            logger.info("[CircuitBreaker] Desactivado — drawdown normalizado")

        logger.info("Posición cerrada: %s PnL=%.4f (%.2f%%)", symbol, realized, pos.pnl_pct)
        return realized

    def update_prices(self, prices: dict[str, float]) -> None:
        """Actualiza precios actuales de todas las posiciones."""
        for symbol, price in prices.items():
            if symbol in self.positions:
                self.positions[symbol].current_price = price

    def reset_circuit_breaker(self) -> None:
        """Resetea manualmente el circuit breaker (uso operador/supervisor)."""
        self._circuit_breaker_active = False
        self._consecutive_losses = 0
        logger.info("[CircuitBreaker] Reset manual por operador")

    # ── Propiedades ───────────────────────────────────────────────────────────

    @property
    def total_pnl(self) -> float:
        return sum(p.pnl for p in self.positions.values())

    @property
    def total_equity(self) -> float:
        return self.capital + sum(p.size * p.current_price for p in self.positions.values())

    @property
    def total_log_return(self) -> float:
        """Retorno logarítmico total acumulado de trades cerrados.
        Suma de log-retornos = crecimiento logarítmico de la riqueza (Kelly optimal).
        """
        return sum(self._realized_log_returns)

    @property
    def geometric_return(self) -> float:
        """Retorno geométrico total: e^(sum_log_returns) - 1"""
        return math.exp(self.total_log_return) - 1

    @property
    def current_drawdown_pct(self) -> float:
        dd = (self._peak_capital - self.total_equity) / self._peak_capital
        return round(dd * 100, 2)

    def _drawdown_exceeded(self) -> bool:
        dd = (self._peak_capital - self.total_equity) / (self._peak_capital + 1e-10)
        return dd > self.max_drawdown_pct

    def summary(self) -> dict:
        return {
            "initial_capital": self.initial_capital,
            "current_equity": round(self.total_equity, 2),
            "free_capital": round(self.capital, 2),
            "open_positions": len(self.positions),
            "total_pnl": round(self.total_pnl, 4),
            "total_log_return": round(self.total_log_return, 6),
            "geometric_return_pct": round(self.geometric_return * 100, 4),
            "drawdown_pct": round(self.current_drawdown_pct, 2),
            "circuit_breaker_active": self._circuit_breaker_active,
            "consecutive_losses": self._consecutive_losses,
            "positions": [p.to_dict() for p in self.positions.values()],
        }
