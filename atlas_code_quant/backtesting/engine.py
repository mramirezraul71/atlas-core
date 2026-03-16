"""Atlas Code-Quant — Motor de backtesting event-driven.

Simula estrategias sobre datos históricos OHLCV vela a vela,
gestionando posiciones, comisiones y slippage de forma realista.
"""
from __future__ import annotations

import logging
from dataclasses import dataclass, field
from datetime import datetime
from typing import Any

import pandas as pd

from strategies.base import BaseStrategy, Signal, TradeSignal

logger = logging.getLogger("quant.backtest")


@dataclass
class Trade:
    symbol: str
    side: str
    entry_price: float
    exit_price: float
    size: float
    entry_time: datetime
    exit_time: datetime
    pnl: float
    pnl_pct: float
    exit_reason: str        # "signal" | "stop_loss" | "take_profit" | "end"

    def to_dict(self) -> dict:
        return {
            "symbol": self.symbol,
            "side": self.side,
            "entry_price": self.entry_price,
            "exit_price": self.exit_price,
            "size": self.size,
            "entry_time": self.entry_time.isoformat(),
            "exit_time": self.exit_time.isoformat(),
            "pnl": round(self.pnl, 4),
            "pnl_pct": round(self.pnl_pct, 4),
            "exit_reason": self.exit_reason,
        }


@dataclass
class BacktestConfig:
    initial_capital: float = 10_000.0
    commission_pct: float = 0.001       # 0.1% por operación (Binance estándar)
    slippage_pct: float = 0.0005        # 0.05% slippage
    position_size_pct: float = 0.05     # 5% del capital por trade
    max_open_trades: int = 3


class BacktestEngine:
    """Motor de backtesting vela a vela con gestión de riesgo.

    Args:
        strategy: Estrategia a evaluar (hereda BaseStrategy).
        config: Parámetros de simulación.

    Example::
        engine = BacktestEngine(strategy=MACrossStrategy(...))
        results = engine.run(df, symbol="BTC/USDT")
        print(results.metrics)
    """

    def __init__(self, strategy: BaseStrategy, config: BacktestConfig | None = None) -> None:
        self.strategy = strategy
        self.cfg = config or BacktestConfig()

    def run(self, df: pd.DataFrame, symbol: str) -> "BacktestResult":
        """Ejecuta el backtest vela a vela.

        Args:
            df: OHLCV DataFrame con index timestamp, mínimo slow_period*2 filas.
            symbol: Par/ticker simulado.

        Returns:
            BacktestResult con trades, equity curve y métricas.
        """
        capital = self.cfg.initial_capital
        equity_curve: list[dict] = []
        trades: list[Trade] = []
        open_pos: dict[str, Any] = {}   # symbol → posición activa
        peak = capital

        for i in range(1, len(df)):
            bar = df.iloc[i]
            ts  = df.index[i]
            price = float(bar["close"])
            high  = float(bar["high"])
            low   = float(bar["low"])

            # ── Chequear SL / TP en posición abierta ───────────────────────
            if symbol in open_pos:
                pos = open_pos[symbol]
                closed = False
                exit_price = price
                exit_reason = "signal"

                if pos["side"] == "long":
                    if pos["stop_loss"] and low <= pos["stop_loss"]:
                        exit_price = pos["stop_loss"]
                        exit_reason = "stop_loss"
                        closed = True
                    elif pos["take_profit"] and high >= pos["take_profit"]:
                        exit_price = pos["take_profit"]
                        exit_reason = "take_profit"
                        closed = True

                if closed:
                    trade, capital = self._close_position(pos, exit_price, exit_reason, ts, capital)
                    trades.append(trade)
                    del open_pos[symbol]

            # ── Generar señal con ventana hasta la barra actual ─────────────
            window = df.iloc[: i + 1]
            signal: TradeSignal = self.strategy.generate_signal(window, symbol)

            # ── Abrir posición si hay señal y no hay posición abierta ───────
            if (signal.signal == Signal.BUY
                    and symbol not in open_pos
                    and len(open_pos) < self.cfg.max_open_trades):
                entry = self._apply_slippage(price, "buy")
                size  = self._position_size(capital, entry)
                cost  = size * entry * (1 + self.cfg.commission_pct)
                if cost <= capital:
                    capital -= cost
                    open_pos[symbol] = {
                        "symbol": symbol,
                        "side": "long",
                        "entry_price": entry,
                        "size": size,
                        "entry_time": ts,
                        "stop_loss": signal.stop_loss,
                        "take_profit": signal.take_profit,
                    }

            elif signal.signal == Signal.SELL and symbol in open_pos:
                pos = open_pos[symbol]
                exit_p = self._apply_slippage(price, "sell")
                trade, capital = self._close_position(pos, exit_p, "signal", ts, capital)
                trades.append(trade)
                del open_pos[symbol]

            # ── Equity curve ────────────────────────────────────────────────
            unrealized = sum(
                p["size"] * price - p["size"] * p["entry_price"]
                for p in open_pos.values()
            )
            equity = capital + sum(p["size"] * price for p in open_pos.values()) + unrealized
            peak = max(peak, equity)
            equity_curve.append({"timestamp": ts, "equity": round(equity, 4), "price": price})

        # Cierra posiciones restantes al último precio
        last_bar = df.iloc[-1]
        last_ts  = df.index[-1]
        for sym, pos in list(open_pos.items()):
            exit_p = float(last_bar["close"])
            trade, capital = self._close_position(pos, exit_p, "end", last_ts, capital)
            trades.append(trade)

        equity_df = pd.DataFrame(equity_curve).set_index("timestamp")
        return BacktestResult(
            symbol=symbol,
            strategy_name=self.strategy.name,
            trades=trades,
            equity_curve=equity_df,
            final_capital=capital,
            initial_capital=self.cfg.initial_capital,
            config=self.cfg,
        )

    def _position_size(self, capital: float, price: float) -> float:
        usd = capital * self.cfg.position_size_pct
        return usd / price if price > 0 else 0.0

    def _apply_slippage(self, price: float, side: str) -> float:
        mult = 1 + self.cfg.slippage_pct if side == "buy" else 1 - self.cfg.slippage_pct
        return price * mult

    def _close_position(
        self, pos: dict, exit_price: float, reason: str, ts: datetime, capital: float
    ) -> tuple[Trade, float]:
        proceeds = pos["size"] * exit_price * (1 - self.cfg.commission_pct)
        capital += proceeds
        cost = pos["size"] * pos["entry_price"]
        pnl = proceeds - cost * (1 + self.cfg.commission_pct)
        pnl_pct = pnl / (cost * (1 + self.cfg.commission_pct)) * 100
        trade = Trade(
            symbol=pos["symbol"],
            side=pos["side"],
            entry_price=pos["entry_price"],
            exit_price=exit_price,
            size=pos["size"],
            entry_time=pos["entry_time"],
            exit_time=ts,
            pnl=pnl,
            pnl_pct=pnl_pct,
            exit_reason=reason,
        )
        return trade, capital


@dataclass
class BacktestResult:
    symbol: str
    strategy_name: str
    trades: list[Trade]
    equity_curve: pd.DataFrame
    final_capital: float
    initial_capital: float
    config: BacktestConfig
    _metrics: dict = field(default_factory=dict, repr=False)

    def __post_init__(self):
        from backtesting.metrics import compute_metrics
        self._metrics = compute_metrics(self)

    @property
    def metrics(self) -> dict:
        return self._metrics

    def summary(self) -> str:
        m = self._metrics
        lines = [
            f"{'─'*50}",
            f" Backtest: {self.strategy_name} | {self.symbol}",
            f"{'─'*50}",
            f" Capital inicial : ${self.initial_capital:,.2f}",
            f" Capital final   : ${self.final_capital:,.2f}",
            f" Retorno total   : {m.get('total_return_pct', 0):.2f}%",
            f" Sharpe ratio    : {m.get('sharpe_ratio', 0):.3f}",
            f" Max drawdown    : {m.get('max_drawdown_pct', 0):.2f}%",
            f" Trades totales  : {m.get('total_trades', 0)}",
            f" Win rate        : {m.get('win_rate_pct', 0):.1f}%",
            f" Profit factor   : {m.get('profit_factor', 0):.2f}",
            f" Avg PnL/trade   : ${m.get('avg_pnl', 0):.4f}",
            f"{'─'*50}",
        ]
        return "\n".join(lines)
