"""Atlas Code-Quant — Métricas de backtesting.

Calcula métricas cuantitativas a partir de un BacktestResult:
Sharpe, Sortino, Calmar, Drawdown, Win Rate, Profit Factor, etc.
"""
from __future__ import annotations

import math
from typing import TYPE_CHECKING

import numpy as np
import pandas as pd

if TYPE_CHECKING:
    from backtesting.engine import BacktestResult

RISK_FREE_RATE = 0.04   # 4% anual, referencia US Treasury
TRADING_DAYS   = 252


def compute_metrics(result: "BacktestResult") -> dict:
    """Calcula todas las métricas de rendimiento del backtest.

    Args:
        result: BacktestResult con trades y equity curve.

    Returns:
        Dict con métricas cuantitativas (floats, 4 decimales).
    """
    trades    = result.trades
    eq_df     = result.equity_curve
    init_cap  = result.initial_capital
    final_cap = result.final_capital

    # ── Retorno total ────────────────────────────────────────────────────────
    total_return_pct = (final_cap - init_cap) / init_cap * 100

    # ── Métricas de trades ───────────────────────────────────────────────────
    total_trades = len(trades)
    wins  = [t for t in trades if t.pnl > 0]
    loses = [t for t in trades if t.pnl <= 0]

    win_rate_pct   = len(wins) / total_trades * 100 if total_trades else 0.0
    avg_pnl        = sum(t.pnl for t in trades) / total_trades if total_trades else 0.0
    avg_win        = sum(t.pnl for t in wins)  / len(wins)  if wins  else 0.0
    avg_loss       = sum(t.pnl for t in loses) / len(loses) if loses else 0.0

    gross_profit = sum(t.pnl for t in wins)
    gross_loss   = abs(sum(t.pnl for t in loses))
    profit_factor = gross_profit / gross_loss if gross_loss > 0 else float("inf")

    expectancy = (win_rate_pct / 100 * avg_win) + ((1 - win_rate_pct / 100) * avg_loss)

    # ── Drawdown ─────────────────────────────────────────────────────────────
    max_drawdown_pct = 0.0
    max_drawdown_usd = 0.0
    if not eq_df.empty:
        equity_series = eq_df["equity"]
        rolling_peak  = equity_series.cummax()
        drawdown      = (equity_series - rolling_peak) / rolling_peak * 100
        max_drawdown_pct = float(drawdown.min())
        max_drawdown_usd = float((equity_series - rolling_peak).min())

    # ── Ratios de riesgo/retorno ─────────────────────────────────────────────
    sharpe_ratio = 0.0
    sortino_ratio = 0.0
    calmar_ratio  = 0.0

    if not eq_df.empty and len(eq_df) > 1:
        returns = eq_df["equity"].pct_change().dropna()
        rf_daily = RISK_FREE_RATE / TRADING_DAYS

        mean_ret = float(returns.mean())
        std_ret  = float(returns.std())

        if std_ret > 0:
            sharpe_ratio = (mean_ret - rf_daily) / std_ret * math.sqrt(TRADING_DAYS)

        downside = returns[returns < rf_daily]
        downside_std = float(downside.std()) if len(downside) > 1 else 0.0
        if downside_std > 0:
            sortino_ratio = (mean_ret - rf_daily) / downside_std * math.sqrt(TRADING_DAYS)

        if max_drawdown_pct < 0:
            ann_return = ((1 + mean_ret) ** TRADING_DAYS - 1) * 100
            calmar_ratio = ann_return / abs(max_drawdown_pct)

    # ── Duración promedio de trades ──────────────────────────────────────────
    avg_duration_h = 0.0
    if trades:
        durations = [(t.exit_time - t.entry_time).total_seconds() / 3600 for t in trades]
        avg_duration_h = sum(durations) / len(durations)

    # ── Racha máxima ─────────────────────────────────────────────────────────
    max_win_streak  = _max_streak(trades, win=True)
    max_loss_streak = _max_streak(trades, win=False)

    # ── Retorno anualizado ───────────────────────────────────────────────────
    ann_return_pct = 0.0
    if not eq_df.empty and len(eq_df) > 1:
        returns = eq_df["equity"].pct_change().dropna()
        mean_ret = float(returns.mean())
        ann_return_pct = ((1 + mean_ret) ** TRADING_DAYS - 1) * 100

    return {
        "total_return_pct":  round(total_return_pct, 4),
        "ann_return_pct":    round(ann_return_pct, 4),
        "total_trades":      total_trades,
        "win_trades":        len(wins),
        "loss_trades":       len(loses),
        "win_rate_pct":      round(win_rate_pct, 2),
        "avg_pnl":           round(avg_pnl, 4),
        "avg_win":           round(avg_win, 4),
        "avg_loss":          round(avg_loss, 4),
        "profit_factor":     round(profit_factor, 4) if math.isfinite(profit_factor) else 999.0,
        "expectancy":        round(expectancy, 4),
        "gross_profit":      round(gross_profit, 4),
        "gross_loss":        round(gross_loss, 4),
        "sharpe_ratio":      round(sharpe_ratio, 4),
        "sortino_ratio":     round(sortino_ratio, 4),
        "calmar_ratio":      round(calmar_ratio, 4),
        "max_drawdown_pct":  round(max_drawdown_pct, 4),
        "max_drawdown_usd":  round(max_drawdown_usd, 4),
        "avg_duration_h":    round(avg_duration_h, 2),
        "max_win_streak":    max_win_streak,
        "max_loss_streak":   max_loss_streak,
        "final_capital":     round(final_cap, 2),
        "initial_capital":   round(init_cap, 2),
    }


def _max_streak(trades: list, win: bool) -> int:
    """Calcula la racha máxima de victorias o derrotas consecutivas."""
    max_s = current = 0
    for t in trades:
        if (t.pnl > 0) == win:
            current += 1
            max_s = max(max_s, current)
        else:
            current = 0
    return max_s
