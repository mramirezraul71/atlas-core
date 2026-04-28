"""
lotto_quant.dashboard.hud
=========================

Atlas Lotto-Quant — Professional HUD.

Streamlit-based, dark-theme, minimalist, high-impact dashboard.

Features
--------
- Operating mode toggle (PAPER ↔ LIVE) persisted across sessions.
- Live KPI cards (bankroll, realized P&L, Sharpe-like, max drawdown,
  win-rate, hit-rate, EV-positive games, alerts last 24h).
- Plotly visualizations:
    • Equity curve with drawdown shading
    • EV scoreboard (top games by EV/$)
    • Anomaly heatmap (treemap of EV × anomaly score)
    • Markov trajectories — tickets-until-EV-positive distribution
    • Alert feed
    • Local-AI status panel (Ollama health + model)
- Auto-refresh every N seconds (sidebar control).
- All data is read through `dashboard.metrics.MetricsService`.

Run with:
    streamlit run lotto_quant/dashboard/hud.py
"""

from __future__ import annotations

import os
import time
from typing import Optional

import pandas as pd
import streamlit as st

try:
    import plotly.express as px
    import plotly.graph_objects as go
    _PLOTLY_OK = True
except Exception:  # pragma: no cover
    _PLOTLY_OK = False

from .. import config as _config
from ..data.database import LottoQuantDB
from ..backtest import BacktestConfig, BacktestEngine, BacktestResult
from ..execution.broker import LiveBroker
from ..signals.alert_engine import make_default_notifier
from ..execution.modes import OperatingMode, get_state, set_active_mode
from .links import atlas_central_url
from .metrics import DashboardMetrics, MetricsService

# ─────────────────────────────────────────────────────────────────
# Page setup
# ─────────────────────────────────────────────────────────────────
st.set_page_config(
    page_title="Atlas Lotto-Quant HUD",
    page_icon="🎯",
    layout="wide",
    initial_sidebar_state="expanded",
)

# ─────────────────────────────────────────────────────────────────
# Theme — dark, minimalist, high contrast
# ─────────────────────────────────────────────────────────────────
_CSS = """
<style>
:root {
  --bg: #0b0d12;
  --panel: #11141b;
  --panel-2: #161a23;
  --border: #1f2330;
  --text: #e7ecf2;
  --muted: #8b93a7;
  --accent: #4ea3ff;
  --good: #22c55e;
  --warn: #f59e0b;
  --bad: #ef4444;
}
.stApp, .main, .block-container {
  background: var(--bg) !important;
  color: var(--text) !important;
}
.block-container { padding-top: 1.2rem; padding-bottom: 2rem; }
section[data-testid="stSidebar"] {
  background: #0a0c11 !important;
  border-right: 1px solid var(--border);
}
section[data-testid="stSidebar"] * { color: var(--text) !important; }
h1, h2, h3, h4, h5 { color: var(--text) !important; letter-spacing: -0.01em; }
hr { border-color: var(--border) !important; }

/* KPI cards */
.kpi {
  background: linear-gradient(180deg, var(--panel) 0%, var(--panel-2) 100%);
  border: 1px solid var(--border);
  border-radius: 14px;
  padding: 16px 18px;
  height: 100%;
}
.kpi .label {
  color: var(--muted);
  font-size: 12px;
  text-transform: uppercase;
  letter-spacing: 0.08em;
  margin-bottom: 6px;
}
.kpi .value {
  color: var(--text);
  font-size: 26px;
  font-weight: 700;
  line-height: 1.1;
}
.kpi .delta { font-size: 12px; margin-top: 4px; }
.kpi .good { color: var(--good); }
.kpi .bad  { color: var(--bad);  }
.kpi .warn { color: var(--warn); }

/* Mode pill */
.mode-pill {
  display: inline-block;
  padding: 4px 10px;
  border-radius: 999px;
  font-size: 11px;
  font-weight: 700;
  letter-spacing: 0.1em;
  text-transform: uppercase;
}
.mode-paper { background: rgba(78,163,255,0.15); color: var(--accent); border:1px solid rgba(78,163,255,0.4); }
.mode-live  { background: rgba(239,68,68,0.15);  color: var(--bad);    border:1px solid rgba(239,68,68,0.4);  }

/* DataFrames */
.stDataFrame { border-radius: 10px; overflow: hidden; }

/* Tabs */
.stTabs [role="tab"] {
  color: var(--muted) !important;
  background: transparent !important;
  border: none !important;
}
.stTabs [aria-selected="true"] {
  color: var(--text) !important;
  border-bottom: 2px solid var(--accent) !important;
}

/* Buttons */
.stButton > button {
  background: var(--panel) !important;
  color: var(--text) !important;
  border: 1px solid var(--border) !important;
  border-radius: 8px !important;
}
.stButton > button:hover {
  border-color: var(--accent) !important;
  color: var(--accent) !important;
}
</style>
"""
st.markdown(_CSS, unsafe_allow_html=True)


# ─────────────────────────────────────────────────────────────────
# Plotly theme
# ─────────────────────────────────────────────────────────────────
def _apply_plot_theme(fig):
    if not _PLOTLY_OK:
        return fig
    fig.update_layout(
        paper_bgcolor="#11141b",
        plot_bgcolor="#11141b",
        font=dict(color="#e7ecf2", family="Inter, system-ui, sans-serif", size=12),
        margin=dict(l=10, r=10, t=40, b=10),
        xaxis=dict(gridcolor="#1f2330", zerolinecolor="#1f2330"),
        yaxis=dict(gridcolor="#1f2330", zerolinecolor="#1f2330"),
        legend=dict(bgcolor="rgba(0,0,0,0)"),
    )
    return fig


# ─────────────────────────────────────────────────────────────────
# Helpers
# ─────────────────────────────────────────────────────────────────
def _kpi(label: str, value: str, delta: Optional[str] = None, klass: str = ""):
    delta_html = (
        f'<div class="delta {klass}">{delta}</div>' if delta else ""
    )
    st.markdown(
        f"""
        <div class="kpi">
          <div class="label">{label}</div>
          <div class="value">{value}</div>
          {delta_html}
        </div>
        """,
        unsafe_allow_html=True,
    )


def _fmt_money(v: float) -> str:
    sign = "-" if v < 0 else ""
    return f"{sign}${abs(v):,.2f}"


def _fmt_pct(v: float) -> str:
    return f"{v * 100:.1f}%"


def _ollama_status() -> dict:
    """Best-effort liveness probe of local-AI backend."""
    out = {
        "backend": _config.LOCAL_AI_BACKEND,
        "url": _config.LOCAL_AI_BASE_URL,
        "model": _config.LOCAL_AI_MODEL,
        "ok": False,
        "latency_ms": None,
    }
    if not _config.LOCAL_AI_ENABLED:
        out["ok"] = False
        out["note"] = "disabled by ATLAS_LLM_ENABLED"
        return out
    try:
        import urllib.request
        import json as _json
        t0 = time.time()
        with urllib.request.urlopen(f"{_config.LOCAL_AI_BASE_URL}/api/tags", timeout=1.5) as r:
            data = _json.loads(r.read().decode())
        out["ok"] = True
        out["latency_ms"] = int((time.time() - t0) * 1000)
        out["models_installed"] = [m.get("name") for m in data.get("models", [])][:8]
    except Exception as e:
        out["ok"] = False
        out["error"] = str(e)[:80]
    return out


# ─────────────────────────────────────────────────────────────────
# Sidebar — mode + refresh + bankroll
# ─────────────────────────────────────────────────────────────────
def _sidebar() -> dict:
    state = get_state()
    st.sidebar.markdown("## Atlas Lotto-Quant")
    st.sidebar.caption("Local-AI · DuckDB · Streamlit")

    # Mode toggle
    st.sidebar.markdown("### Operating Mode")
    current = state.mode
    pill_cls = "mode-paper" if current == OperatingMode.PAPER else "mode-live"
    st.sidebar.markdown(
        f'<span class="mode-pill {pill_cls}">{current.value}</span>',
        unsafe_allow_html=True,
    )
    new_mode = st.sidebar.radio(
        "Switch mode",
        options=[OperatingMode.PAPER.value, OperatingMode.LIVE.value],
        index=0 if current == OperatingMode.PAPER else 1,
        horizontal=True,
        key="mode_radio",
    )
    if new_mode != current.value:
        set_active_mode(OperatingMode.from_string(new_mode))
        st.sidebar.success(f"Switched to {new_mode.upper()}")
        st.rerun()

    if current == OperatingMode.LIVE:
        st.sidebar.warning(
            "LIVE mode: Atlas records intent only. A human must "
            "physically purchase tickets and confirm outcomes."
        )

    # Bankroll
    st.sidebar.markdown("### Bankroll")
    if current == OperatingMode.PAPER:
        new_bk = st.sidebar.number_input(
            "Paper bankroll (USD)",
            min_value=0.0, value=float(state.paper_bankroll),
            step=100.0, format="%.2f",
        )
        if abs(new_bk - state.paper_bankroll) > 1e-6:
            set_active_mode(current, paper_bankroll=new_bk)
            st.rerun()
    else:
        new_bk = st.sidebar.number_input(
            "Live bankroll (USD)",
            min_value=0.0, value=float(state.live_bankroll),
            step=100.0, format="%.2f",
        )
        if abs(new_bk - state.live_bankroll) > 1e-6:
            set_active_mode(current, live_bankroll=new_bk)
            st.rerun()

    # Refresh
    st.sidebar.markdown("### Refresh")
    auto = st.sidebar.toggle("Auto-refresh", value=True)
    interval = st.sidebar.slider("Interval (s)", 5, 120, 15, step=5)
    if st.sidebar.button("Refresh now", use_container_width=True):
        st.rerun()

    # Local-AI panel
    st.sidebar.markdown("### Local AI")
    s = _ollama_status()
    badge = "🟢" if s["ok"] else "🔴"
    st.sidebar.markdown(f"{badge} **{s['backend']}** · `{s['model']}`")
    st.sidebar.caption(s["url"])
    if s["ok"]:
        st.sidebar.caption(f"Latency: {s['latency_ms']} ms")
        models = s.get("models_installed") or []
        if models:
            st.sidebar.caption("Installed: " + ", ".join(models))
    else:
        st.sidebar.caption(s.get("error") or s.get("note", "offline"))

    # Cross-link to Atlas central dashboard (V4)
    st.sidebar.markdown("### Atlas central")
    central_url = atlas_central_url()
    st.sidebar.markdown(
        f"<a href='{central_url}' target='_blank' rel='noopener noreferrer' "
        "style='display:inline-block;padding:8px 12px;background:#1f2937;"
        "border:1px solid #2a3242;border-radius:8px;text-decoration:none;"
        "color:#e7ecf2;font-weight:600;width:100%;text-align:center;'>"
        "→ Atlas Dashboard V4"
        "</a>",
        unsafe_allow_html=True,
    )
    st.sidebar.caption(central_url)

    return {"auto": auto, "interval": interval}


# ─────────────────────────────────────────────────────────────────
# Header
# ─────────────────────────────────────────────────────────────────
def _header(m: DashboardMetrics):
    pill_cls = "mode-paper" if m.mode == OperatingMode.PAPER else "mode-live"
    col1, col2 = st.columns([4, 1])
    with col1:
        st.markdown(
            f"""
            <h2 style="margin-bottom:0">Atlas Lotto-Quant HUD
              <span class="mode-pill {pill_cls}" style="margin-left:10px;font-size:14px">{m.mode.value}</span>
            </h2>
            <div style="color:#8b93a7;font-size:13px;margin-top:4px">
              Real-time edge detection on NC scratch-off & jackpot games · powered by local LLMs
            </div>
            """,
            unsafe_allow_html=True,
        )
    with col2:
        ts = pd.Timestamp.utcnow().strftime("%Y-%m-%d %H:%M:%S UTC")
        central = atlas_central_url()
        st.markdown(
            f'<div style="text-align:right;color:#8b93a7;font-size:12px;margin-top:14px">'
            f'Last refresh<br/><b style="color:#e7ecf2">{ts}</b><br/>'
            f'<a href="{central}" target="_blank" rel="noopener noreferrer" '
            f'style="color:#4ea3ff;text-decoration:none;font-weight:600">'
            f'↗ Atlas V4</a></div>',
            unsafe_allow_html=True,
        )
    st.markdown("<hr/>", unsafe_allow_html=True)


# ─────────────────────────────────────────────────────────────────
# KPI strip
# ─────────────────────────────────────────────────────────────────
def _kpi_strip(m: DashboardMetrics):
    h = m.headline
    pnl = m.pnl
    cols = st.columns(4)
    with cols[0]:
        klass = "good" if pnl.realized_pnl >= 0 else "bad"
        _kpi("Bankroll", _fmt_money(pnl.bankroll_current),
             f"P&L {_fmt_money(pnl.realized_pnl)} · start {_fmt_money(pnl.bankroll_start)}",
             klass)
    with cols[1]:
        roi = (pnl.realized_pnl / pnl.bankroll_start) if pnl.bankroll_start else 0.0
        klass = "good" if roi >= 0 else "bad"
        _kpi("ROI", _fmt_pct(roi),
             f"avg EV/$ {pnl.avg_ev_per_dollar*100:.2f}%", klass)
    with cols[2]:
        _kpi("Sharpe-like", f"{pnl.sharpe_like:.2f}",
             f"max drawdown {_fmt_pct(pnl.max_drawdown)}",
             "good" if pnl.sharpe_like >= 0 else "bad")
    with cols[3]:
        _kpi("Win / Hit Rate",
             f"{_fmt_pct(pnl.win_rate)} / {_fmt_pct(pnl.hit_rate)}",
             f"{pnl.n_fills} fills · {pnl.n_orders} orders")

    cols = st.columns(4)
    with cols[0]:
        _kpi("Games tracked", str(h.get("games_tracked", 0)),
             f"{h.get('ev_positive_games', 0)} EV-positive",
             "good" if h.get("ev_positive_games", 0) > 0 else "")
    with cols[1]:
        best = h.get("best_ev_per_dollar", 0.0)
        _kpi("Best EV/$", f"${best:+.3f}",
             "edge" if best > 0 else "no edge",
             "good" if best > 0 else "warn")
    with cols[2]:
        _kpi("Signals 24h", str(h.get("signals_last_24h", 0)),
             f"{h.get('alerts_last_24h', 0)} alerts dispatched")
    with cols[3]:
        _kpi("Mode", m.mode.value.upper(),
             "Simulated draws" if m.mode == OperatingMode.PAPER else "Manual confirm",
             "warn" if m.mode == OperatingMode.LIVE else "")


# ─────────────────────────────────────────────────────────────────
# Charts
# ─────────────────────────────────────────────────────────────────
def _chart_equity(m: DashboardMetrics):
    if not _PLOTLY_OK:
        st.info("Plotly not installed.")
        return
    curve = m.pnl.equity_curve
    if not curve:
        st.info("No fills yet — run the radar in PAPER mode to populate the equity curve.")
        return
    df = pd.DataFrame(curve)
    df["ts"] = pd.to_datetime(df["ts"], errors="coerce", utc=True)
    df["ts"] = df["ts"].fillna(method="ffill")
    df["idx"] = range(len(df))
    df["peak"] = df["equity"].cummax()
    df["drawdown"] = df["peak"] - df["equity"]

    fig = go.Figure()
    fig.add_trace(go.Scatter(
        x=df["idx"], y=df["peak"], mode="lines",
        line=dict(color="#1f2330", width=1), name="Peak", hoverinfo="skip",
    ))
    fig.add_trace(go.Scatter(
        x=df["idx"], y=df["equity"], mode="lines",
        line=dict(color="#4ea3ff", width=2.5),
        fill="tonexty", fillcolor="rgba(239,68,68,0.10)",
        name="Equity",
    ))
    fig.update_layout(title="Equity curve · drawdown shaded",
                      xaxis_title="Fill #", yaxis_title="Equity ($)")
    st.plotly_chart(_apply_plot_theme(fig), use_container_width=True)


def _chart_ev_scoreboard(m: DashboardMetrics):
    if not _PLOTLY_OK:
        st.info("Plotly not installed.")
        return
    df = m.latest_per_game
    if df.empty:
        st.info("No snapshots yet — radar hasn't run a cycle.")
        return
    top = df.head(15).copy()
    top["color"] = top["ev_per_dollar"].apply(
        lambda v: "#22c55e" if v > 0 else ("#f59e0b" if v > -0.05 else "#ef4444"))
    fig = go.Figure(go.Bar(
        x=top["ev_per_dollar"] * 100,
        y=top["game_name"],
        orientation="h",
        marker_color=top["color"],
        text=[f"{v*100:+.2f}%" for v in top["ev_per_dollar"]],
        textposition="outside",
    ))
    fig.update_layout(title="EV per $ · top games (latest snapshot)",
                      xaxis_title="EV/$ (%)", yaxis=dict(autorange="reversed"))
    st.plotly_chart(_apply_plot_theme(fig), use_container_width=True)


def _chart_anomaly_treemap(m: DashboardMetrics):
    if not _PLOTLY_OK:
        st.info("Plotly not installed.")
        return
    df = m.latest_per_game
    if df.empty:
        return
    df = df.copy()
    df["abs_ev"] = df["ev_per_dollar"].abs() + 0.001
    fig = px.treemap(
        df,
        path=[px.Constant("All games"), "game_name"],
        values="abs_ev",
        color="anomaly_score",
        color_continuous_scale=["#22c55e", "#f59e0b", "#ef4444"],
        range_color=(0, 1),
        custom_data=["ev_adjusted", "depletion_ratio", "anomaly_score"],
    )
    fig.update_traces(
        hovertemplate=(
            "<b>%{label}</b><br>"
            "EV adjusted: $%{customdata[0]:.3f}<br>"
            "Depletion: %{customdata[1]:.1%}<br>"
            "Anomaly: %{customdata[2]:.2f}<extra></extra>"
        )
    )
    fig.update_layout(title="Anomaly heatmap · sized by |EV/$|, colored by anomaly score")
    st.plotly_chart(_apply_plot_theme(fig), use_container_width=True)


def _chart_markov(m: DashboardMetrics):
    if not _PLOTLY_OK:
        st.info("Plotly not installed.")
        return
    df = m.markov_predictions
    if df.empty:
        st.info("No Markov predictions yet.")
        return
    df = df.sort_values("pred_ts").tail(50)
    fig = go.Figure()
    fig.add_trace(go.Scatter(
        x=df["pred_ts"], y=df["tickets_until_ev_positive"],
        mode="lines+markers",
        line=dict(color="#4ea3ff", width=2),
        marker=dict(
            size=8,
            color=df["confidence"], colorscale="Viridis",
            cmin=0, cmax=1, showscale=True,
            colorbar=dict(title="Conf"),
        ),
        name="Tickets to EV+",
    ))
    fig.update_layout(
        title="Markov trajectory · tickets-until-EV-positive (latest 50 predictions)",
        xaxis_title="Prediction time", yaxis_title="Tickets",
    )
    st.plotly_chart(_apply_plot_theme(fig), use_container_width=True)


# ─────────────────────────────────────────────────────────────────
# Tables
# ─────────────────────────────────────────────────────────────────
def _table_signals(m: DashboardMetrics):
    df = m.signals
    if df.empty:
        st.info("No signals yet.")
        return
    show = df.head(50).copy()
    if "ev_net" in show.columns:
        show["ev_net"] = show["ev_net"].astype(float).round(4)
    if "signal_strength" in show.columns:
        show["signal_strength"] = show["signal_strength"].astype(float).round(3)
    st.dataframe(show, use_container_width=True, height=380)


def _table_fills(m: DashboardMetrics):
    df = m.fills
    if df.empty:
        st.info("No fills for this mode yet.")
        return
    show = df.head(100).copy()
    for col in ("cost", "gross_payout", "net_payout", "pnl_net"):
        if col in show.columns:
            show[col] = show[col].astype(float).round(2)
    st.dataframe(show, use_container_width=True, height=420)


def _table_alerts(m: DashboardMetrics):
    df = m.alerts
    if df.empty:
        st.info("No alerts yet — radar hasn't triggered an EV-positive signal.")
        return
    st.dataframe(df.head(50), use_container_width=True, height=380)


def _table_per_game(m: DashboardMetrics):
    if not m.pnl.per_game:
        st.info("No fills aggregated yet.")
        return
    df = pd.DataFrame(m.pnl.per_game)
    for col in ("cost", "net_payout", "pnl_net"):
        if col in df.columns:
            df[col] = df[col].astype(float).round(2)
    if "roi" in df.columns:
        df["roi"] = (df["roi"] * 100).round(2)
    st.dataframe(df, use_container_width=True, height=320)


def _chart_roi_per_game(m: DashboardMetrics):
    """Horizontal bar of ROI% per game, sorted best → worst."""
    if not _PLOTLY_OK:
        st.info("Plotly not installed.")
        return
    if not m.pnl.per_game:
        st.info("No fills aggregated yet — run the radar to populate ROI.")
        return
    df = pd.DataFrame(m.pnl.per_game).sort_values("roi", ascending=True)
    df["roi_pct"] = df["roi"] * 100
    df["color"] = df["roi_pct"].apply(
        lambda v: "#22c55e" if v >= 0 else ("#f59e0b" if v >= -25 else "#ef4444")
    )
    fig = go.Figure(go.Bar(
        x=df["roi_pct"], y=df["game_id"],
        orientation="h", marker_color=df["color"],
        text=[f"{v:+.1f}%" for v in df["roi_pct"]],
        textposition="outside",
        customdata=df[["n_fills", "n_tickets", "pnl_net"]].values,
        hovertemplate=(
            "<b>%{y}</b><br>ROI: %{x:.2f}%%<br>"
            "Fills: %{customdata[0]} · Tickets: %{customdata[1]}<br>"
            "P&L: $%{customdata[2]:.2f}<extra></extra>"
        ),
    ))
    fig.update_layout(title="ROI per game (realized)",
                      xaxis_title="ROI (%)", yaxis_title="")
    st.plotly_chart(_apply_plot_theme(fig), use_container_width=True)


# ───────────────────────────────────────────────────────────────
# Live confirmation form
# ───────────────────────────────────────────────────────────────
def _live_confirmation_panel(m: DashboardMetrics):
    """Form to confirm or cancel LIVE orders without writing code."""
    if m.mode != OperatingMode.LIVE:
        st.info(
            "Switch to **LIVE** mode in the sidebar to confirm physical "
            "ticket outcomes. PAPER mode auto-fills via Monte Carlo."
        )
        return

    db = LottoQuantDB()
    broker = LiveBroker(db, notifier=make_default_notifier(db))
    open_orders = broker.list_open_orders(limit=50)

    st.markdown(
        "#### Confirm physical ticket outcomes\n"
        "Pick a LIVE order, enter the gross payout the retailer paid you "
        "(before tax), and Atlas will record the fill with the correct "
        "NC + federal withholding applied."
    )

    if not open_orders:
        st.success("✓ No open LIVE orders awaiting confirmation.")
        return

    options = {
        f"{o.game_name}  ·  {o.n_tickets}×${o.ticket_price:.2f}  ·  "
        f"{o.created_iso[:19]}  ·  id={o.order_id[:8]}…": o
        for o in open_orders
    }
    label = st.selectbox("Open LIVE orders", list(options.keys()))
    order = options[label]

    col1, col2, col3 = st.columns(3)
    col1.metric("Tickets", order.n_tickets)
    col2.metric("Cost", f"${order.cost():.2f}")
    col3.metric("Expected EV", f"${order.expected_ev:+.2f}")

    with st.form("confirm_outcome_form", clear_on_submit=True):
        gross = st.number_input(
            "Gross payout received ($)",
            min_value=0.0, value=0.0, step=1.0, format="%.2f",
            help="Total prize value before any tax withholding.",
        )
        c1, c2 = st.columns(2)
        confirm_clicked = c1.form_submit_button("✓ Confirm outcome",
                                                 use_container_width=True,
                                                 type="primary")
        cancel_clicked = c2.form_submit_button("✗ Cancel order",
                                                use_container_width=True)

    if confirm_clicked:
        try:
            fill = broker.confirm_outcome(order, gross_payout=float(gross))
            st.success(
                f"Recorded fill {fill.fill_id[:8]}…  "
                f"gross=${fill.gross_payout:.2f}  net=${fill.net_payout:.2f}  "
                f"P&L=${fill.pnl_net:+.2f}"
            )
            st.rerun()
        except Exception as e:
            st.error(f"Failed to record outcome: {e}")

    if cancel_clicked:
        if broker.cancel_order(order.order_id):
            st.warning(f"Order {order.order_id[:8]}… cancelled.")
            st.rerun()
        else:
            st.error("Could not cancel — order may already be filled.")


# ─────────────────────────────────────────────────────────────────
# Backtest panel
# ─────────────────────────────────────────────────────────────────
def _run_backtest_cached(
    *,
    bankroll_start: float,
    kelly_fraction: float,
    max_position_pct: float,
    max_daily_exposure: float,
    min_ev_per_dollar: float,
    rng_seed: int,
    days_back: Optional[int],
) -> BacktestResult:
    """Run a backtest using a temp workspace DB so production stays clean."""
    import tempfile
    fd, tmp_path = tempfile.mkstemp(prefix="lotto_bt_hud_", suffix=".sqlite")
    os.close(fd)
    cfg = BacktestConfig(
        workspace_db=tmp_path,
        bankroll_start=bankroll_start,
        kelly_fraction=kelly_fraction,
        max_position_pct=max_position_pct,
        max_daily_exposure=max_daily_exposure,
        min_ev_per_dollar=min_ev_per_dollar,
        rng_seed=rng_seed,
        days_back=days_back,
    )
    return BacktestEngine(cfg).run()


def _backtest_panel() -> None:
    """Sidebar-driven backtest tab — replays snapshots in paper mode."""
    st.markdown("### Historical replay (paper mode)")
    st.caption(
        "Replays the snapshot archive through the paper broker so you can "
        "calibrate Kelly fraction, EV cutoff, and exposure caps before "
        "committing capital. Backtests never touch the LIVE flow."
    )

    with st.form("backtest_form"):
        c1, c2, c3 = st.columns(3)
        with c1:
            bankroll = st.number_input(
                "Starting bankroll ($)", min_value=50.0, max_value=100_000.0,
                value=1_000.0, step=50.0,
            )
            days_back = st.number_input(
                "Days back (0 = all)", min_value=0, max_value=365,
                value=30, step=1,
            )
        with c2:
            kelly_fraction = st.slider(
                "Kelly fraction", min_value=0.05, max_value=1.0,
                value=0.25, step=0.05,
            )
            max_position_pct = st.slider(
                "Max position pct", min_value=0.005, max_value=0.20,
                value=0.05, step=0.005,
            )
        with c3:
            max_daily_exposure = st.slider(
                "Max daily exposure", min_value=0.05, max_value=0.50,
                value=0.20, step=0.05,
            )
            min_ev_per_dollar = st.number_input(
                "Min EV / $ (filter)", min_value=-1.0, max_value=1.0,
                value=0.0, step=0.01,
            )
        rng_seed = st.number_input("RNG seed", min_value=0, max_value=2**31 - 1,
                                    value=1234, step=1)
        run = st.form_submit_button("Run backtest", type="primary")

    if not run:
        st.info("Configure parameters and click **Run backtest**.")
        return

    with st.spinner("Replaying snapshots…"):
        try:
            res = _run_backtest_cached(
                bankroll_start=float(bankroll),
                kelly_fraction=float(kelly_fraction),
                max_position_pct=float(max_position_pct),
                max_daily_exposure=float(max_daily_exposure),
                min_ev_per_dollar=float(min_ev_per_dollar),
                rng_seed=int(rng_seed),
                days_back=int(days_back) if days_back > 0 else None,
            )
        except Exception as e:
            st.error(f"Backtest failed: {e}")
            return

    if res.notes:
        for n in res.notes:
            st.warning(n)
        if not res.days:
            return

    # KPI strip
    pnl = res.realized_pnl
    pnl_color = "normal" if pnl >= 0 else "inverse"
    k1, k2, k3, k4, k5 = st.columns(5)
    k1.metric("Final bankroll", f"${res.final_bankroll:,.2f}",
              f"{pnl:+,.2f}", delta_color=pnl_color)
    k2.metric("Realized P&L", f"${pnl:+,.2f}")
    k3.metric("Max drawdown", f"{res.max_drawdown * 100:.2f}%")
    k4.metric("Sharpe-like", f"{res.sharpe_like:.3f}")
    k5.metric("Win-rate (days)", f"{res.win_rate * 100:.1f}%")

    # Equity curve
    if _PLOTLY_OK and res.days:
        eq_df = pd.DataFrame(res.equity_curve, columns=["day", "bankroll"])
        fig = px.line(eq_df, x="day", y="bankroll",
                       title="Backtest equity curve", markers=True)
        fig.update_layout(
            height=320, margin=dict(l=10, r=10, t=40, b=10),
            paper_bgcolor="#0e1117", plot_bgcolor="#0e1117",
            font=dict(color="#e6e6e6"),
        )
        st.plotly_chart(fig, use_container_width=True)

    # Daily breakdown
    st.markdown("#### Daily breakdown")
    if res.days:
        df = pd.DataFrame([d.to_dict() for d in res.days])
        for col in ("bankroll_open", "bankroll_close", "total_cost",
                    "gross_payout", "net_payout", "pnl_net", "drawdown"):
            if col in df.columns:
                df[col] = df[col].astype(float).round(2)
        st.dataframe(df, use_container_width=True, height=320)

    # Per-game PnL
    if res.per_game_pnl:
        st.markdown("#### Per-game P&L")
        pg_df = pd.DataFrame(
            [{"game_id": gid, "pnl_net": float(round(v, 2))}
             for gid, v in sorted(res.per_game_pnl.items(),
                                   key=lambda kv: kv[1], reverse=True)]
        )
        if _PLOTLY_OK and not pg_df.empty:
            fig2 = px.bar(
                pg_df.head(20), x="game_id", y="pnl_net",
                title="P&L per game (top 20)",
                color="pnl_net", color_continuous_scale="RdYlGn",
            )
            fig2.update_layout(
                height=300, margin=dict(l=10, r=10, t=40, b=10),
                paper_bgcolor="#0e1117", plot_bgcolor="#0e1117",
                font=dict(color="#e6e6e6"),
            )
            st.plotly_chart(fig2, use_container_width=True)
        st.dataframe(pg_df, use_container_width=True, height=240)

    with st.expander("Run summary (raw)"):
        st.json(res.summary_dict())


# ─────────────────────────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────────────────────────
def main():
    cfg = _sidebar()
    try:
        metrics = MetricsService().collect()
    except Exception as e:
        st.error(f"Failed to load metrics: {e}")
        st.stop()

    _header(metrics)
    _kpi_strip(metrics)

    tabs = st.tabs([
        "Overview",
        "EV Scoreboard",
        "Anomaly Map",
        "Markov",
        "Fills & P&L",
        "Confirm LIVE",
        "Backtest",
        "Signals",
        "Alerts",
    ])

    with tabs[0]:
        c1, c2 = st.columns([3, 2])
        with c1:
            _chart_equity(metrics)
        with c2:
            _chart_ev_scoreboard(metrics)
        st.markdown("#### ROI per game")
        _chart_roi_per_game(metrics)
        st.markdown("#### Per-game performance")
        _table_per_game(metrics)

    with tabs[1]:
        _chart_ev_scoreboard(metrics)
        st.markdown("#### Latest per game")
        if not metrics.latest_per_game.empty:
            show = metrics.latest_per_game.copy()
            for col in ("ev_gross", "ev_adjusted", "ev_per_dollar",
                        "depletion_ratio", "anomaly_score"):
                if col in show.columns:
                    show[col] = show[col].astype(float).round(4)
            st.dataframe(show, use_container_width=True, height=380)

    with tabs[2]:
        _chart_anomaly_treemap(metrics)

    with tabs[3]:
        _chart_markov(metrics)

    with tabs[4]:
        _chart_roi_per_game(metrics)
        st.markdown("#### Fills")
        _table_fills(metrics)

    with tabs[5]:
        _live_confirmation_panel(metrics)

    with tabs[6]:
        _backtest_panel()

    with tabs[7]:
        _table_signals(metrics)

    with tabs[8]:
        _table_alerts(metrics)

    # Auto-refresh
    if cfg.get("auto"):
        time.sleep(cfg.get("interval", 15))
        st.rerun()


if __name__ == "__main__":
    main()
