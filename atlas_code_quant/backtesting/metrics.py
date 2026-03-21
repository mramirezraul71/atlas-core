"""Atlas Code-Quant — Métricas de backtesting.

Calcula métricas cuantitativas a partir de un BacktestResult:
Sharpe, Sortino, Calmar, Drawdown, Win Rate, Profit Factor, etc.

v2 (logarítmico):
- Métricas basadas en retornos logarítmicos (más precisas para compounding).
- Kelly fraction estimada a partir del historial de trades.
- Monte Carlo simulation (N paths por resampling de trades).
  Grok/xAI: walk-forward + Monte Carlo 10.000 sim = validación rigurosa.
"""
from __future__ import annotations

import math
import random
from typing import TYPE_CHECKING

import numpy as np
import pandas as pd

if TYPE_CHECKING:
    from backtesting.engine import BacktestResult

RISK_FREE_RATE = 0.04   # 4% anual, referencia US Treasury
TRADING_DAYS   = 252
_MC_SIMULATIONS = 1_000  # Reducido a 1000 para no bloquear la API (vs 10000 en batch)


def compute_metrics(result: "BacktestResult") -> dict:
    """Calcula todas las métricas de rendimiento del backtest.

    Args:
        result: BacktestResult con trades y equity curve.

    Returns:
        Dict con métricas cuantitativas incluyendo logarítmicas, Kelly y Monte Carlo.
    """
    trades    = result.trades
    eq_df     = result.equity_curve
    init_cap  = result.initial_capital
    final_cap = result.final_capital

    # ── Retorno total (aritmético y logarítmico) ─────────────────────────────
    total_return_pct = (final_cap - init_cap) / init_cap * 100

    # Retorno logarítmico total — suma de log-retornos individuales
    log_returns_trades = [t.log_return for t in trades if hasattr(t, "log_return")]
    log_total_return = sum(log_returns_trades) if log_returns_trades else math.log(final_cap / init_cap) if init_cap > 0 else 0.0
    geometric_return_pct = (math.exp(log_total_return) - 1) * 100

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

    # ── Ratios de riesgo/retorno (base logarítmica) ──────────────────────────
    sharpe_ratio  = 0.0
    sortino_ratio = 0.0
    calmar_ratio  = 0.0
    ann_return_pct = 0.0

    if not eq_df.empty and len(eq_df) > 1:
        # Usar log-retornos de la equity curve — más precisos
        equity_series = eq_df["equity"]
        log_rets = np.log(equity_series / equity_series.shift(1)).dropna()
        rf_daily = RISK_FREE_RATE / TRADING_DAYS

        mean_log = float(log_rets.mean())
        std_log  = float(log_rets.std())

        if std_log > 0:
            sharpe_ratio = (mean_log - rf_daily) / std_log * math.sqrt(TRADING_DAYS)

        downside = log_rets[log_rets < rf_daily]
        downside_std = float(downside.std()) if len(downside) > 1 else 0.0
        if downside_std > 0:
            sortino_ratio = (mean_log - rf_daily) / downside_std * math.sqrt(TRADING_DAYS)

        # Retorno anualizado desde retornos log
        ann_return_pct = (math.exp(mean_log * TRADING_DAYS) - 1) * 100

        if max_drawdown_pct < 0:
            calmar_ratio = ann_return_pct / abs(max_drawdown_pct)

    # ── Kelly Fraction estimada ───────────────────────────────────────────────
    kelly_fraction = _estimate_kelly(trades)

    # ── Monte Carlo — distribución de retornos esperados ─────────────────────
    mc_results = _monte_carlo(trades, init_cap, n_sim=_MC_SIMULATIONS)

    # ── Duración promedio de trades ──────────────────────────────────────────
    avg_duration_h = 0.0
    if trades:
        durations = [(t.exit_time - t.entry_time).total_seconds() / 3600 for t in trades]
        avg_duration_h = sum(durations) / len(durations)

    # ── Racha máxima ─────────────────────────────────────────────────────────
    max_win_streak  = _max_streak(trades, win=True)
    max_loss_streak = _max_streak(trades, win=False)

    return {
        # Retornos
        "total_return_pct":      round(total_return_pct, 4),
        "log_total_return":      round(log_total_return, 6),
        "geometric_return_pct":  round(geometric_return_pct, 4),
        "ann_return_pct":        round(ann_return_pct, 4),
        # Trades
        "total_trades":          total_trades,
        "win_trades":            len(wins),
        "loss_trades":           len(loses),
        "win_rate_pct":          round(win_rate_pct, 2),
        "avg_pnl":               round(avg_pnl, 4),
        "avg_win":               round(avg_win, 4),
        "avg_loss":              round(avg_loss, 4),
        "profit_factor":         round(profit_factor, 4) if math.isfinite(profit_factor) else 999.0,
        "expectancy":            round(expectancy, 4),
        "gross_profit":          round(gross_profit, 4),
        "gross_loss":            round(gross_loss, 4),
        # Ratios
        "sharpe_ratio":          round(sharpe_ratio, 4),
        "sortino_ratio":         round(sortino_ratio, 4),
        "calmar_ratio":          round(calmar_ratio, 4),
        # Drawdown
        "max_drawdown_pct":      round(max_drawdown_pct, 4),
        "max_drawdown_usd":      round(max_drawdown_usd, 4),
        # Kelly
        "kelly_fraction":        round(kelly_fraction, 6),
        "kelly_quarter":         round(kelly_fraction * 0.25, 6),
        "kelly_half":            round(kelly_fraction * 0.50, 6),
        # Monte Carlo
        "mc_median_return_pct":  round(mc_results["median_return_pct"], 2),
        "mc_var_95":             round(mc_results["var_95"], 2),      # VaR 95%
        "mc_cvar_95":            round(mc_results["cvar_95"], 2),     # CVaR / Expected Shortfall
        "mc_prob_profit":        round(mc_results["prob_profit"], 4), # P(retorno > 0)
        "mc_simulations":        _MC_SIMULATIONS,
        # Misc
        "avg_duration_h":        round(avg_duration_h, 2),
        "max_win_streak":        max_win_streak,
        "max_loss_streak":       max_loss_streak,
        "final_capital":         round(final_cap, 2),
        "initial_capital":       round(init_cap, 2),
    }


# ── Kelly estimado ────────────────────────────────────────────────────────────

def _estimate_kelly(trades: list) -> float:
    """Estima la fracción de Kelly óptima a partir de los trades del backtest.

    f* = (p * b - q) / b
    donde p=win_rate, q=1-p, b=avg_win/avg_loss
    """
    if len(trades) < 6:
        return 0.0
    wins  = [t.pnl for t in trades if t.pnl > 0]
    loses = [abs(t.pnl) for t in trades if t.pnl <= 0]
    if not wins or not loses:
        return 0.0
    p = len(wins) / len(trades)
    q = 1.0 - p
    b = (sum(wins) / len(wins)) / (sum(loses) / len(loses))
    if b <= 0:
        return 0.0
    f_star = (p * b - q) / b
    return max(0.0, min(f_star, 1.0))


# ── Monte Carlo ───────────────────────────────────────────────────────────────

def _monte_carlo(
    trades: list,
    initial_capital: float,
    n_sim: int = _MC_SIMULATIONS,
) -> dict:
    """Simulación Monte Carlo por resampling de trades históricos.

    Genera N caminos aleatorios reordenando los trades,
    calcula distribución de retornos finales.

    Args:
        trades: Lista de Trade con pnl.
        initial_capital: Capital inicial.
        n_sim: Número de simulaciones.

    Returns:
        Dict con percentiles, VaR, CVaR y probabilidad de ganancia.
    """
    if len(trades) < 3:
        return {
            "median_return_pct": 0.0,
            "var_95": 0.0,
            "cvar_95": 0.0,
            "prob_profit": 0.5,
        }

    pnls = [t.pnl for t in trades]
    final_returns: list[float] = []

    for _ in range(n_sim):
        # Resamplear trades con reemplazo (bootstrap)
        sampled = random.choices(pnls, k=len(pnls))
        final_equity = initial_capital + sum(sampled)
        ret_pct = (final_equity - initial_capital) / initial_capital * 100
        final_returns.append(ret_pct)

    arr = sorted(final_returns)
    n   = len(arr)

    median_return_pct = arr[n // 2]
    var_95_idx        = int(n * 0.05)   # 5% peor cola
    var_95            = arr[var_95_idx]
    cvar_95           = sum(arr[:var_95_idx]) / max(var_95_idx, 1)
    prob_profit       = sum(1 for r in arr if r > 0) / n

    return {
        "median_return_pct": median_return_pct,
        "var_95": var_95,
        "cvar_95": cvar_95,
        "prob_profit": prob_profit,
    }


# ── Helpers ───────────────────────────────────────────────────────────────────

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
