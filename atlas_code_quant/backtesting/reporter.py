"""Atlas Code-Quant — Reporter de backtesting.

Genera informes visuales (HTML con Plotly) y JSON a partir de BacktestResult.
"""
from __future__ import annotations

import json
from pathlib import Path
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from backtesting.engine import BacktestResult


def export_json(result: "BacktestResult", path: str | Path) -> Path:
    """Exporta el resultado completo a JSON.

    Args:
        result: BacktestResult a exportar.
        path: Ruta de salida (.json).

    Returns:
        Path del archivo creado.
    """
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)

    payload = {
        "symbol":        result.symbol,
        "strategy_name": result.strategy_name,
        "metrics":       result.metrics,
        "trades":        [t.to_dict() for t in result.trades],
        "config": {
            "initial_capital":   result.config.initial_capital,
            "commission_pct":    result.config.commission_pct,
            "slippage_pct":      result.config.slippage_pct,
            "position_size_pct": result.config.position_size_pct,
            "max_open_trades":   result.config.max_open_trades,
        },
    }
    path.write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")
    return path


def generate_html_report(result: "BacktestResult", path: str | Path) -> Path:
    """Genera un informe HTML interactivo con Plotly.

    Args:
        result: BacktestResult a visualizar.
        path: Ruta de salida (.html).

    Returns:
        Path del archivo HTML generado.
    """
    try:
        import plotly.graph_objects as go
        from plotly.subplots import make_subplots
    except ImportError as e:
        raise ImportError("plotly es necesario: pip install plotly") from e

    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)

    eq_df   = result.equity_curve
    trades  = result.trades
    metrics = result.metrics
    m       = metrics

    fig = make_subplots(
        rows=3, cols=2,
        subplot_titles=(
            "Curva de Equity",
            "Distribución PnL por Trade",
            "Precio vs Operaciones",
            "Drawdown",
            "PnL Acumulado",
            "Duración de Trades (h)",
        ),
        row_heights=[0.4, 0.3, 0.3],
        specs=[
            [{"colspan": 2}, None],
            [{"type": "xy"}, {"type": "xy"}],
            [{"type": "xy"}, {"type": "xy"}],
        ],
    )

    # ── 1. Equity curve ──────────────────────────────────────────────────────
    fig.add_trace(
        go.Scatter(
            x=eq_df.index, y=eq_df["equity"],
            mode="lines", name="Equity",
            line=dict(color="#00d4ff", width=2),
            fill="tozeroy", fillcolor="rgba(0,212,255,0.08)",
        ),
        row=1, col=1,
    )

    # ── 2. Distribución PnL ──────────────────────────────────────────────────
    pnls = [t.pnl for t in trades]
    colors = ["#00e676" if p > 0 else "#ff5252" for p in pnls]
    fig.add_trace(
        go.Bar(
            x=list(range(len(pnls))), y=pnls,
            name="PnL Trades",
            marker_color=colors,
        ),
        row=2, col=1,
    )

    # ── 3. Drawdown ──────────────────────────────────────────────────────────
    if not eq_df.empty:
        rolling_peak = eq_df["equity"].cummax()
        drawdown_pct = (eq_df["equity"] - rolling_peak) / rolling_peak * 100
        fig.add_trace(
            go.Scatter(
                x=eq_df.index, y=drawdown_pct,
                mode="lines", name="Drawdown %",
                line=dict(color="#ff5252", width=1.5),
                fill="tozeroy", fillcolor="rgba(255,82,82,0.12)",
            ),
            row=2, col=2,
        )

    # ── 4. PnL acumulado ─────────────────────────────────────────────────────
    cum_pnl = []
    running = 0.0
    for t in trades:
        running += t.pnl
        cum_pnl.append(running)
    fig.add_trace(
        go.Scatter(
            x=list(range(len(cum_pnl))), y=cum_pnl,
            mode="lines+markers", name="PnL Acumulado",
            line=dict(color="#ffd740", width=2),
            marker=dict(size=5),
        ),
        row=3, col=1,
    )

    # ── 5. Duración trades ───────────────────────────────────────────────────
    durations = [(t.exit_time - t.entry_time).total_seconds() / 3600 for t in trades]
    fig.add_trace(
        go.Histogram(
            x=durations, nbinsx=20,
            name="Duración (h)",
            marker_color="#b388ff",
        ),
        row=3, col=2,
    )

    # ── Layout ───────────────────────────────────────────────────────────────
    fig.update_layout(
        template="plotly_dark",
        title=dict(
            text=f"Atlas Code-Quant — Backtest: {result.strategy_name} | {result.symbol}",
            font=dict(size=18, color="#00d4ff"),
        ),
        paper_bgcolor="#0d1117",
        plot_bgcolor="#0d1117",
        height=900,
        showlegend=False,
        annotations=[
            dict(
                text=(
                    f"Retorno: <b>{m.get('total_return_pct',0):.2f}%</b>  |  "
                    f"Sharpe: <b>{m.get('sharpe_ratio',0):.3f}</b>  |  "
                    f"Drawdown: <b>{m.get('max_drawdown_pct',0):.2f}%</b>  |  "
                    f"Win Rate: <b>{m.get('win_rate_pct',0):.1f}%</b>  |  "
                    f"Trades: <b>{m.get('total_trades',0)}</b>  |  "
                    f"Profit Factor: <b>{m.get('profit_factor',0):.2f}</b>"
                ),
                xref="paper", yref="paper",
                x=0.5, y=-0.06,
                showarrow=False,
                font=dict(size=12, color="#aaa"),
                align="center",
            )
        ],
    )

    fig.write_html(str(path), include_plotlyjs="cdn")
    return path


def print_summary(result: "BacktestResult") -> None:
    """Imprime el resumen de texto del backtest."""
    print(result.summary())
