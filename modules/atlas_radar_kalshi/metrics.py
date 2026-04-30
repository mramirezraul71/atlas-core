"""
metrics.py — Métricas de performance y observabilidad.

Calcula desde el journal:

- PnL bruto y neto.
- Hit-rate, profit factor, expectancy.
- Drawdown actual y máximo.
- Exposición total y por mercado.

Adicionalmente expone un ``prometheus_text()`` que emite las series
en formato OpenMetrics para que el dashboard u otro Prometheus las
consuma. No requiere ``prometheus_client``: emitimos texto plano.
"""
from __future__ import annotations

import json
from collections import defaultdict
from typing import Iterable

from pydantic import BaseModel

from .state.journal import Journal


class PerformanceMetrics(BaseModel):
    trades: int = 0
    wins: int = 0
    losses: int = 0
    hit_rate: float = 0.0
    pnl_gross_cents: int = 0
    pnl_net_cents: int = 0
    profit_factor: float = 0.0
    expectancy_cents: float = 0.0
    max_drawdown_cents: int = 0
    current_drawdown_cents: int = 0
    avg_slippage_cents: float = 0.0


def compute(journal: Journal) -> PerformanceMetrics:
    exits = journal.read("exits", limit=10_000)
    if not exits:
        return PerformanceMetrics()
    pnl = []
    slips = []
    for e in exits:
        pnl.append(int(e.get("pnl_cents", 0)))
        if "slippage_cents" in e:
            slips.append(float(e["slippage_cents"]))

    wins = sum(1 for x in pnl if x > 0)
    losses = sum(1 for x in pnl if x < 0)
    total = sum(pnl)
    gross_win = sum(x for x in pnl if x > 0)
    gross_loss = -sum(x for x in pnl if x < 0)
    pf = (gross_win / gross_loss) if gross_loss > 0 else (
        float("inf") if gross_win > 0 else 0.0
    )
    expectancy = (total / len(pnl)) if pnl else 0.0

    # drawdown sobre equity curve
    equity = 0
    peak = 0
    max_dd = 0
    for x in pnl:
        equity += x
        peak = max(peak, equity)
        dd = peak - equity
        max_dd = max(max_dd, dd)
    current_dd = peak - equity

    return PerformanceMetrics(
        trades=len(pnl),
        wins=wins, losses=losses,
        hit_rate=wins / len(pnl) if pnl else 0.0,
        pnl_gross_cents=total + sum(int(e.get("fees_cents", 0)) for e in exits),
        pnl_net_cents=total,
        profit_factor=pf if pf != float("inf") else 999.0,
        expectancy_cents=expectancy,
        max_drawdown_cents=max_dd,
        current_drawdown_cents=current_dd,
        avg_slippage_cents=(sum(slips) / len(slips)) if slips else 0.0,
    )


def exposure_by_market(open_positions: dict[str, int]) -> dict[str, int]:
    return dict(open_positions)


def prometheus_text(perf: PerformanceMetrics, exec_metrics: dict,
                    risk_status: dict) -> str:
    """Emite métricas en formato OpenMetrics."""
    lines = [
        "# HELP radar_pnl_cents Net PnL in cents",
        "# TYPE radar_pnl_cents gauge",
        f"radar_pnl_cents {perf.pnl_net_cents}",
        "# HELP radar_trades_total Total trades",
        "# TYPE radar_trades_total counter",
        f"radar_trades_total {perf.trades}",
        "# HELP radar_hit_rate Hit rate (wins / trades)",
        "# TYPE radar_hit_rate gauge",
        f"radar_hit_rate {perf.hit_rate:.6f}",
        "# HELP radar_profit_factor Profit factor",
        "# TYPE radar_profit_factor gauge",
        f"radar_profit_factor {perf.profit_factor:.6f}",
        "# HELP radar_max_drawdown_cents Max drawdown in cents",
        "# TYPE radar_max_drawdown_cents gauge",
        f"radar_max_drawdown_cents {perf.max_drawdown_cents}",
        "# HELP radar_latency_p95_ms p95 order latency",
        "# TYPE radar_latency_p95_ms gauge",
        f"radar_latency_p95_ms {exec_metrics.get('p95_ms', 0)}",
        "# HELP radar_fill_ratio Fill ratio",
        "# TYPE radar_fill_ratio gauge",
        f"radar_fill_ratio {exec_metrics.get('fill_ratio', 0):.6f}",
        "# HELP radar_safe_mode 1 if safe mode is on",
        "# TYPE radar_safe_mode gauge",
        f"radar_safe_mode {1 if risk_status.get('safe_mode') else 0}",
        "# HELP radar_kill_switch 1 if kill switch is on",
        "# TYPE radar_kill_switch gauge",
        f"radar_kill_switch {1 if risk_status.get('kill_switch') else 0}",
        "# HELP radar_exposure_cents Total exposure in cents",
        "# TYPE radar_exposure_cents gauge",
        f"radar_exposure_cents {risk_status.get('exposure_cents', 0)}",
        "# HELP radar_drawdown_day_cents Intraday drawdown vs peak (cents)",
        "# TYPE radar_drawdown_day_cents gauge",
        f"radar_drawdown_day_cents {risk_status.get('drawdown_day_cents', 0)}",
    ]
    return "\n".join(lines) + "\n"
