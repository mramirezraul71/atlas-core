"""Backtest Engine — F9.

Motor mínimo determinista para evaluar una ``StrategyPlan`` sobre una serie
sintética de trades. NO sustituye LEAN. Usa fixtures locales o trades pasados
explícitamente, y devuelve métricas estándar.

Diseño paper-first:
- No realiza llamadas externas.
- Usa una semilla determinista por ``(symbol, strategy_name)`` para que dos
  ejecuciones consecutivas produzcan los mismos resultados.
- Cuando ``ATLAS_LEAN_ENABLED=true`` y un parser LEAN devuelve estadísticas
  reales, el engine prefiere esas estadísticas. En sandbox sin LEAN cae al
  generador determinista.

Métricas devueltas por ``BacktestResult``:
    sharpe, profit_factor, win_rate, expectancy,
    max_drawdown, trades_count, total_pnl, avg_win, avg_loss
"""
from __future__ import annotations

import math
import os
import random
from dataclasses import dataclass, field, asdict
from typing import Any

from atlas_code_quant.strategies.contracts import StrategyPlan


# ---------------------------------------------------------------------------
# Modelos
# ---------------------------------------------------------------------------


@dataclass(slots=True)
class Trade:
    """Un trade individual del backtest sintético."""

    pnl_usd: float
    bars_held: int = 1
    win: bool = False

    def __post_init__(self) -> None:
        self.win = self.pnl_usd > 0.0


@dataclass(slots=True)
class BacktestResult:
    """Resultado consolidado de un backtest sobre una ``StrategyPlan``."""

    strategy: str
    symbol: str
    trades_count: int
    sharpe: float
    profit_factor: float
    win_rate: float
    expectancy: float
    max_drawdown: float
    total_pnl: float
    avg_win: float
    avg_loss: float
    trace_id: str = ""
    source: str = "stub"
    rejected: bool = False
    rejection_reasons: list[str] = field(default_factory=list)

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


# ---------------------------------------------------------------------------
# Engine
# ---------------------------------------------------------------------------


# Perfiles base por estrategia. Reflejan la forma típica del payoff sin
# pretender ser realistas: el ranker los compara entre sí.
_STRATEGY_PROFILES: dict[str, dict[str, float]] = {
    "vertical_spread": {"win_rate": 0.62, "avg_win": 110.0, "avg_loss": -150.0, "vol": 60.0},
    "iron_condor":     {"win_rate": 0.72, "avg_win":  85.0, "avg_loss": -210.0, "vol": 55.0},
    "iron_butterfly":  {"win_rate": 0.55, "avg_win": 145.0, "avg_loss": -180.0, "vol": 80.0},
    "straddle_strangle": {"win_rate": 0.48, "avg_win": 220.0, "avg_loss": -130.0, "vol": 95.0},
}

_DEFAULT_PROFILE = {"win_rate": 0.55, "avg_win": 100.0, "avg_loss": -110.0, "vol": 70.0}


def _seed_for(plan: StrategyPlan) -> int:
    """Semilla determinista por (symbol, strategy)."""
    s = f"{plan.symbol}|{plan.strategy}"
    h = 0
    for ch in s:
        h = (h * 131 + ord(ch)) & 0xFFFFFFFF
    return h or 1


def _generate_trades(
    plan: StrategyPlan,
    n_trades: int,
    *,
    score_modifier: float = 0.0,
) -> list[Trade]:
    """Genera ``n_trades`` deterministas siguiendo el perfil de la estrategia.

    ``score_modifier`` desplaza la win_rate ±0.05 según el score Radar
    (oportunidades >80 mejoran ligeramente el WR; <65 lo reducen).
    """
    profile = _STRATEGY_PROFILES.get(plan.strategy, _DEFAULT_PROFILE)
    rng = random.Random(_seed_for(plan))
    win_rate = max(0.05, min(0.95, profile["win_rate"] + score_modifier))
    avg_win = profile["avg_win"]
    avg_loss = profile["avg_loss"]
    vol = profile["vol"]

    trades: list[Trade] = []
    for _ in range(n_trades):
        is_win = rng.random() < win_rate
        if is_win:
            pnl = rng.gauss(avg_win, vol * 0.30)
            pnl = max(5.0, pnl)  # un win siempre es positivo
        else:
            pnl = rng.gauss(avg_loss, vol * 0.30)
            pnl = min(-5.0, pnl)  # una pérdida siempre es negativa
        bars = rng.randint(1, 10)
        trades.append(Trade(pnl_usd=round(pnl, 2), bars_held=bars))
    return trades


def _compute_metrics(trades: list[Trade]) -> dict[str, float]:
    """Calcula métricas estándar a partir de una lista de trades."""
    if not trades:
        return {
            "sharpe": 0.0,
            "profit_factor": 0.0,
            "win_rate": 0.0,
            "expectancy": 0.0,
            "max_drawdown": 0.0,
            "total_pnl": 0.0,
            "avg_win": 0.0,
            "avg_loss": 0.0,
        }

    pnls = [t.pnl_usd for t in trades]
    wins = [p for p in pnls if p > 0.0]
    losses = [p for p in pnls if p < 0.0]
    n = len(trades)

    total_pnl = sum(pnls)
    win_rate = len(wins) / n
    avg_win = (sum(wins) / len(wins)) if wins else 0.0
    avg_loss = (sum(losses) / len(losses)) if losses else 0.0
    expectancy = total_pnl / n

    gross_profit = sum(wins)
    gross_loss = abs(sum(losses))
    profit_factor = (gross_profit / gross_loss) if gross_loss > 0 else (
        float("inf") if gross_profit > 0 else 0.0
    )
    if math.isinf(profit_factor):
        profit_factor = 99.0  # cap para tablas

    mean = sum(pnls) / n
    var = sum((p - mean) ** 2 for p in pnls) / n
    std = math.sqrt(var) if var > 0 else 0.0
    # Sharpe simple por trade (no anualizado): desviación estándar de PnL.
    sharpe = (mean / std) if std > 0 else 0.0

    # Max drawdown sobre equity curve
    equity = 0.0
    peak = 0.0
    max_dd = 0.0
    for p in pnls:
        equity += p
        peak = max(peak, equity)
        dd = peak - equity
        max_dd = max(max_dd, dd)
    # MaxDD relativo: respecto al pico (capped a 1.0 si no hubo pico positivo)
    if peak > 0:
        max_dd_rel = min(1.0, max_dd / peak)
    else:
        max_dd_rel = min(1.0, max_dd / max(1.0, abs(min(0.0, total_pnl))))

    return {
        "sharpe": round(sharpe, 4),
        "profit_factor": round(profit_factor, 4),
        "win_rate": round(win_rate, 4),
        "expectancy": round(expectancy, 4),
        "max_drawdown": round(max_dd_rel, 4),
        "total_pnl": round(total_pnl, 2),
        "avg_win": round(avg_win, 2),
        "avg_loss": round(avg_loss, 2),
    }


@dataclass(slots=True)
class BacktestEngine:
    """Backtest engine determinista para evaluar StrategyPlans."""

    n_trades: int = 60
    score_threshold_high: float = 80.0
    score_threshold_low: float = 65.0

    def evaluate(
        self,
        plan: StrategyPlan,
        *,
        opportunity_score: float = 70.0,
        trace_id: str = "",
    ) -> BacktestResult:
        """Evalúa un ``StrategyPlan`` y devuelve ``BacktestResult``.

        Si ``plan.status != 'planned'`` o no es accionable, devuelve un
        ``BacktestResult`` con ``rejected=True`` y métricas a cero.
        """
        if not plan.is_actionable():
            return BacktestResult(
                strategy=plan.strategy,
                symbol=plan.symbol,
                trades_count=0,
                sharpe=0.0,
                profit_factor=0.0,
                win_rate=0.0,
                expectancy=0.0,
                max_drawdown=0.0,
                total_pnl=0.0,
                avg_win=0.0,
                avg_loss=0.0,
                trace_id=trace_id or plan.trace_id,
                rejected=True,
                rejection_reasons=[
                    f"plan_not_actionable status={plan.status} legs={len(plan.legs)}"
                ],
                source="stub",
            )

        # Modificador de win_rate por score Radar
        if opportunity_score >= self.score_threshold_high:
            score_mod = 0.05
        elif opportunity_score < self.score_threshold_low:
            score_mod = -0.05
        else:
            score_mod = 0.0

        trades = _generate_trades(plan, self.n_trades, score_modifier=score_mod)
        metrics = _compute_metrics(trades)

        source = "stub"
        if os.environ.get("ATLAS_LEAN_ENABLED", "false").strip().lower() in {"1", "true", "yes", "on"}:
            # Hook para LEAN: si en el futuro hay parser real, reemplazar aquí.
            # En este sandbox se deja el modo stub explícito.
            source = "lean_unavailable_stub"

        return BacktestResult(
            strategy=plan.strategy,
            symbol=plan.symbol,
            trades_count=len(trades),
            sharpe=metrics["sharpe"],
            profit_factor=metrics["profit_factor"],
            win_rate=metrics["win_rate"],
            expectancy=metrics["expectancy"],
            max_drawdown=metrics["max_drawdown"],
            total_pnl=metrics["total_pnl"],
            avg_win=metrics["avg_win"],
            avg_loss=metrics["avg_loss"],
            trace_id=trace_id or plan.trace_id,
            source=source,
        )
