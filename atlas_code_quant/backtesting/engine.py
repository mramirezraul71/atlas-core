"""Atlas Code-Quant — Motor de backtesting event-driven.

Simula estrategias sobre datos históricos OHLCV vela a vela,
gestionando posiciones, comisiones y slippage de forma realista.

v2 (logarítmico):
- Stop-loss y take-profit dinámicos basados en ATR (Grok recomendación).
- Equity curve incluye log-retorno acumulado por barra.
- Walk-forward validation: método run_walk_forward() para validación
  sin data leakage (train 70% / val 15% / test 15% rolling).
- Slippage variable basado en volatilidad (atr_pct).
"""
from __future__ import annotations

import logging
import math
from dataclasses import dataclass, field
from datetime import datetime
from typing import Any

import numpy as np
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
    log_return: float       # ln(exit/entry) — retorno logarítmico del trade
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
            "log_return": round(self.log_return, 6),
            "exit_reason": self.exit_reason,
        }


@dataclass
class BacktestConfig:
    initial_capital: float = 10_000.0
    commission_pct: float = 0.001       # 0.1% por operación (Binance estándar)
    slippage_pct: float = 0.0005        # 0.05% base; se escala por ATR en v2
    position_size_pct: float = 0.05     # 5% fallback (Kelly reemplaza en producción)
    max_open_trades: int = 3
    use_atr_stops: bool = True          # Usar ATR para SL/TP dinámicos
    atr_sl_multiplier: float = 1.5      # SL = entry ± ATR * 1.5
    atr_tp_multiplier: float = 3.0      # TP = entry ± ATR * 3.0
    atr_period: int = 14
    walk_forward_train_pct: float = 0.70    # 70% train
    walk_forward_val_pct: float = 0.15      # 15% validación
    # Restante 15% = test final


class BacktestEngine:
    """Motor de backtesting vela a vela con gestión de riesgo logarítmica.

    v2: ATR stops dinámicos + log-return tracking + walk-forward.

    Args:
        strategy: Estrategia a evaluar (hereda BaseStrategy).
        config: Parámetros de simulación.

    Example::
        engine = BacktestEngine(strategy=MACrossStrategy(...))
        results = engine.run(df, symbol="BTC/USDT")
        print(results.metrics)
        # Walk-forward:
        wf_results = engine.run_walk_forward(df, symbol="BTC/USDT", folds=5)
    """

    def __init__(self, strategy: BaseStrategy, config: BacktestConfig | None = None) -> None:
        self.strategy = strategy
        self.cfg = config or BacktestConfig()

    def run(self, df: pd.DataFrame, symbol: str) -> "BacktestResult":
        """Ejecuta el backtest vela a vela.

        Args:
            df: OHLCV DataFrame con index timestamp.
            symbol: Par/ticker simulado.

        Returns:
            BacktestResult con trades, equity curve y métricas.
        """
        # Pre-calcular ATR para todo el DataFrame
        atr_series = self._compute_atr(df)

        capital = self.cfg.initial_capital
        equity_curve: list[dict] = []
        trades: list[Trade] = []
        open_pos: dict[str, Any] = {}
        peak = capital
        cum_log_return = 0.0

        for i in range(1, len(df)):
            bar   = df.iloc[i]
            ts    = df.index[i]
            price = float(bar["close"])
            high  = float(bar["high"])
            low   = float(bar["low"])
            atr   = float(atr_series.iloc[i]) if not math.isnan(atr_series.iloc[i]) else price * 0.01

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
                    trade, capital, lr = self._close_position(pos, exit_price, exit_reason, ts, capital)
                    cum_log_return += lr
                    trades.append(trade)
                    del open_pos[symbol]

            # ── Generar señal con ventana hasta la barra actual ─────────────
            window = df.iloc[: i + 1]
            signal: TradeSignal = self.strategy.generate_signal(window, symbol)

            # ── Abrir posición si hay señal y no hay posición abierta ───────
            if (signal.signal == Signal.BUY
                    and symbol not in open_pos
                    and len(open_pos) < self.cfg.max_open_trades):
                entry = self._apply_slippage(price, "buy", atr)
                size  = self._position_size(capital, entry)
                cost  = size * entry * (1 + self.cfg.commission_pct)
                if cost <= capital:
                    # ATR stops dinámicos si la señal no trae SL/TP propios
                    sl, tp = self._resolve_stops(signal, entry, atr, "long")
                    capital -= cost
                    open_pos[symbol] = {
                        "symbol": symbol,
                        "side": "long",
                        "entry_price": entry,
                        "size": size,
                        "entry_time": ts,
                        "stop_loss": sl,
                        "take_profit": tp,
                    }

            elif signal.signal == Signal.SELL and symbol in open_pos:
                pos = open_pos[symbol]
                exit_p = self._apply_slippage(price, "sell", atr)
                trade, capital, lr = self._close_position(pos, exit_p, "signal", ts, capital)
                cum_log_return += lr
                trades.append(trade)
                del open_pos[symbol]

            # ── Equity curve con log-retorno acumulado ───────────────────────
            equity = capital + sum(p["size"] * price for p in open_pos.values())
            peak   = max(peak, equity)
            # Log-retorno instantáneo de la barra
            bar_log_ret = math.log(equity / self.cfg.initial_capital) if equity > 0 else 0.0
            equity_curve.append({
                "timestamp":      ts,
                "equity":         round(equity, 4),
                "price":          price,
                "log_return_cum": round(bar_log_ret, 6),
            })

        # Cierra posiciones restantes al último precio
        last_bar = df.iloc[-1]
        last_ts  = df.index[-1]
        for sym, pos in list(open_pos.items()):
            exit_p = float(last_bar["close"])
            trade, capital, lr = self._close_position(pos, exit_p, "end", last_ts, capital)
            cum_log_return += lr
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
            cumulative_log_return=cum_log_return,
        )

    def run_walk_forward(
        self,
        df: pd.DataFrame,
        symbol: str,
        folds: int = 5,
    ) -> list["BacktestResult"]:
        """Walk-forward validation: divide en folds y evalúa sin data leakage.

        Cada fold: [train | val | test] con porcentajes del config.
        Solo evalúa en el segmento 'test' de cada fold para evitar overfitting.

        Args:
            df: OHLCV DataFrame completo.
            symbol: Par/ticker.
            folds: Número de folds de validación.

        Returns:
            Lista de BacktestResult, uno por fold (segmento test únicamente).
        """
        n = len(df)
        fold_size = n // folds
        results = []

        for fold in range(folds):
            start = fold * fold_size
            end   = start + fold_size if fold < folds - 1 else n
            fold_df = df.iloc[start:end]

            fold_n = len(fold_df)
            train_end = int(fold_n * self.cfg.walk_forward_train_pct)
            val_end   = int(fold_n * (self.cfg.walk_forward_train_pct + self.cfg.walk_forward_val_pct))

            # Solo backtestear en el segmento test
            test_df = fold_df.iloc[val_end:]
            if len(test_df) < 10:
                continue

            # Usar ventana de train+val como warmup (prefijo para calcular señales)
            warmup_df = fold_df.iloc[:val_end]
            full_eval_df = pd.concat([warmup_df, test_df])

            result = self.run(full_eval_df, symbol)
            result.fold = fold + 1
            results.append(result)
            logger.info(
                "[WalkForward] Fold %d/%d — trades=%d retorno=%.2f%% sharpe=%.3f",
                fold + 1, folds,
                result.metrics.get("total_trades", 0),
                result.metrics.get("total_return_pct", 0),
                result.metrics.get("sharpe_ratio", 0),
            )

        return results

    # ── Helpers internos ─────────────────────────────────────────────────────

    def _position_size(self, capital: float, price: float) -> float:
        usd = capital * self.cfg.position_size_pct
        return usd / price if price > 0 else 0.0

    def _apply_slippage(self, price: float, side: str, atr: float = 0.0) -> float:
        """Slippage escalado por volatilidad ATR — más realista en mercados volátiles."""
        base_slip = self.cfg.slippage_pct
        # Escalar slippage por régimen de volatilidad (hasta 3x en alta vol)
        if price > 0 and atr > 0:
            atr_pct = atr / price
            vol_scale = min(3.0, 1.0 + atr_pct * 10)
            base_slip = base_slip * vol_scale
        mult = 1 + base_slip if side == "buy" else 1 - base_slip
        return price * mult

    def _resolve_stops(
        self,
        signal: TradeSignal,
        entry: float,
        atr: float,
        side: str,
    ) -> tuple[float | None, float | None]:
        """Devuelve (stop_loss, take_profit) — ATR si la señal no trae propios."""
        if signal.stop_loss and signal.take_profit:
            return signal.stop_loss, signal.take_profit
        if self.cfg.use_atr_stops and atr > 0:
            sl_dist = atr * self.cfg.atr_sl_multiplier
            tp_dist = atr * self.cfg.atr_tp_multiplier
            if side == "long":
                return (entry - sl_dist, entry + tp_dist)
            return (entry + sl_dist, entry - tp_dist)
        return signal.stop_loss, signal.take_profit

    def _close_position(
        self, pos: dict, exit_price: float, reason: str, ts: datetime, capital: float
    ) -> tuple[Trade, float, float]:
        """Cierra posición y retorna (Trade, capital_actualizado, log_return)."""
        proceeds = pos["size"] * exit_price * (1 - self.cfg.commission_pct)
        capital += proceeds
        cost = pos["size"] * pos["entry_price"]
        pnl = proceeds - cost * (1 + self.cfg.commission_pct)
        pnl_pct = pnl / (cost * (1 + self.cfg.commission_pct)) * 100 if cost > 0 else 0.0

        # Log-return del trade — base para Kelly y métricas logarítmicas
        if pos["entry_price"] > 0 and exit_price > 0:
            log_ret = math.log(exit_price / pos["entry_price"])
            if pos["side"] == "short":
                log_ret = -log_ret
        else:
            log_ret = 0.0

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
            log_return=log_ret,
            exit_reason=reason,
        )
        return trade, capital, log_ret

    def _compute_atr(self, df: pd.DataFrame) -> pd.Series:
        h = df["high"]
        l = df["low"]
        c = df["close"]
        tr = pd.concat([
            h - l,
            (h - c.shift(1)).abs(),
            (l - c.shift(1)).abs(),
        ], axis=1).max(axis=1)
        return tr.rolling(self.cfg.atr_period).mean()


@dataclass
class BacktestResult:
    symbol: str
    strategy_name: str
    trades: list[Trade]
    equity_curve: pd.DataFrame
    final_capital: float
    initial_capital: float
    config: BacktestConfig
    cumulative_log_return: float = 0.0      # Σ ln(exit/entry) de todos los trades
    fold: int = 0                           # Walk-forward fold (0 = full run)
    _metrics: dict = field(default_factory=dict, repr=False)

    def __post_init__(self):
        from backtesting.metrics import compute_metrics
        self._metrics = compute_metrics(self)

    @property
    def metrics(self) -> dict:
        return self._metrics

    def summary(self) -> str:
        m = self._metrics
        fold_str = f" [Fold {self.fold}]" if self.fold else ""
        lines = [
            f"{'─'*55}",
            f" Backtest: {self.strategy_name} | {self.symbol}{fold_str}",
            f"{'─'*55}",
            f" Capital inicial   : ${self.initial_capital:,.2f}",
            f" Capital final     : ${self.final_capital:,.2f}",
            f" Retorno total     : {m.get('total_return_pct', 0):.2f}%",
            f" Log-retorno acum. : {m.get('log_total_return', 0):.4f}",
            f" Retorno geom.     : {m.get('geometric_return_pct', 0):.2f}%",
            f" Sharpe ratio      : {m.get('sharpe_ratio', 0):.3f}",
            f" Sortino ratio     : {m.get('sortino_ratio', 0):.3f}",
            f" Calmar ratio      : {m.get('calmar_ratio', 0):.3f}",
            f" Max drawdown      : {m.get('max_drawdown_pct', 0):.2f}%",
            f" Kelly fraction    : {m.get('kelly_fraction', 0):.4f}",
            f" Trades totales    : {m.get('total_trades', 0)}",
            f" Win rate          : {m.get('win_rate_pct', 0):.1f}%",
            f" Profit factor     : {m.get('profit_factor', 0):.2f}",
            f" MC VaR 95%        : {m.get('mc_var_95', 0):.2f}%",
            f"{'─'*55}",
        ]
        return "\n".join(lines)
