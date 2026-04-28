"""
lotto_quant.dashboard.hud
=========================

Streamlit real-time HUD for Atlas Lotto-Quant.

Run with::

    streamlit run lotto_quant/dashboard/hud.py

Sections
--------
    - Live EV scoreboard (sortable)
    - Anomaly heatmap
    - Markov trajectory plot
    - Kelly allocation panel
    - Alert history
    - Local-AI backend status (Ollama / DeepSeek / Qwen3)
"""

from __future__ import annotations

import asyncio
import json
from datetime import datetime, timedelta
from typing import Any, Dict, List

import pandas as pd

try:  # streamlit/plotly are optional at import time (unit tests)
    import plotly.express as px
    import plotly.graph_objects as go
    import streamlit as st
except ImportError as e:  # pragma: no cover
    raise ImportError(
        "Streamlit / Plotly missing. Install with: "
        "pip install streamlit plotly"
    ) from e

from .. import config
from ..data.database import LottoQuantDB
from ..llm import LocalAIClient


# ─────────────────────────────────────────────────────────────────────
# Data loaders (cached)
# ─────────────────────────────────────────────────────────────────────
@st.cache_resource
def get_db() -> LottoQuantDB:
    return LottoQuantDB()


@st.cache_data(ttl=60)
def load_snapshots(limit: int = 200) -> pd.DataFrame:
    db = get_db()
    rows = db.latest_snapshots(limit=limit)
    if not rows:
        return pd.DataFrame()
    return pd.DataFrame(rows)


@st.cache_data(ttl=60)
def load_signals(hours: int = 24) -> pd.DataFrame:
    db = get_db()
    rows = db.signals_since(hours=hours)
    if not rows:
        return pd.DataFrame()
    return pd.DataFrame(rows)


# ─────────────────────────────────────────────────────────────────────
# Local-AI status check
# ─────────────────────────────────────────────────────────────────────
def check_local_ai_status() -> Dict[str, Any]:
    async def _go() -> Dict[str, Any]:
        async with LocalAIClient() as ai:
            healthy = await ai.health()
            models: List[str] = []
            if healthy:
                try:
                    models = await ai.list_models()
                except Exception:
                    models = []
            return {
                "backend": ai.backend,
                "base_url": ai.base_url,
                "primary_model": ai.model,
                "healthy": healthy,
                "available_models": models,
            }
    return asyncio.run(_go())


# ─────────────────────────────────────────────────────────────────────
# Layout
# ─────────────────────────────────────────────────────────────────────
def main() -> None:
    st.set_page_config(
        page_title="Atlas Lotto-Quant HUD",
        layout="wide",
        page_icon="🎯",
    )

    st.title("Atlas Lotto-Quant — Live HUD")
    st.caption("Quantitative arbitrage of NCEL scratch-offs and draw games. "
               "Local-AI powered.")

    # Sidebar — controls + AI status
    with st.sidebar:
        st.header("Controls")
        st.metric("Refresh interval", f"{config.SCRAPE_INTERVAL_SECONDS // 60} min")
        st.metric("Quarter-Kelly", f"{config.KELLY_FRACTION:.2f}")
        st.metric("Max position %", f"{config.MAX_POSITION_PCT:.1%}")

        st.divider()
        st.subheader("Local AI Backend")
        try:
            status = check_local_ai_status()
            badge = "🟢 Online" if status["healthy"] else "🔴 Offline"
            st.write(f"**Status:** {badge}")
            st.write(f"**Backend:** `{status['backend']}`")
            st.write(f"**URL:** `{status['base_url']}`")
            st.write(f"**Primary model:** `{status['primary_model']}`")
            if status["available_models"]:
                with st.expander("Available models"):
                    for m in status["available_models"]:
                        st.code(m)
        except Exception as e:
            st.error(f"AI status check failed: {e}")

    # ── Main panels ─────────────────────────────────────────────
    snapshots = load_snapshots(limit=300)
    signals = load_signals(hours=72)

    col1, col2, col3, col4 = st.columns(4)
    col1.metric("Games tracked",
                int(snapshots["game_id"].nunique()) if not snapshots.empty else 0)
    col2.metric("Active signals (72h)", len(signals))
    if not snapshots.empty:
        latest = snapshots.sort_values("snapshot_ts").groupby("game_id").tail(1)
        n_pos = int((latest["ev_adjusted"] > 0).sum())
        col3.metric("EV+ games", n_pos)
        col4.metric("Best EV/$",
                    f"{(latest['ev_adjusted'] / latest['ticket_price']).max():+.4f}")
    else:
        col3.metric("EV+ games", 0)
        col4.metric("Best EV/$", "—")

    st.divider()

    # ── Live scoreboard ─────────────────────────────────────────
    st.subheader("Live EV Scoreboard")
    if snapshots.empty:
        st.info("No snapshots yet. Run the radar with `python -m "
                "lotto_quant.signals.opportunity_radar --bankroll 1000 --once`.")
    else:
        latest = snapshots.sort_values("snapshot_ts").groupby("game_id").tail(1)
        latest = latest.assign(
            ev_per_dollar=latest["ev_adjusted"] / latest["ticket_price"]
        ).sort_values("ev_per_dollar", ascending=False)
        st.dataframe(
            latest[
                ["game_name", "ticket_price", "ev_gross", "ev_adjusted",
                 "ev_per_dollar", "depletion_ratio", "anomaly_score",
                 "snapshot_ts"]
            ],
            use_container_width=True,
            hide_index=True,
        )

        # Anomaly bar chart
        fig = px.bar(
            latest.head(20),
            x="game_name",
            y="anomaly_score",
            color="ev_per_dollar",
            color_continuous_scale="RdYlGn",
            title="Top-20 anomaly scores",
        )
        st.plotly_chart(fig, use_container_width=True)

    # ── Recent signals ───────────────────────────────────────────
    st.subheader("Recent Signals (72h)")
    if signals.empty:
        st.info("No signals fired yet.")
    else:
        st.dataframe(signals, use_container_width=True, hide_index=True)


if __name__ == "__main__":
    main()
