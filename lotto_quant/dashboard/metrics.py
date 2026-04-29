"""
lotto_quant.dashboard.metrics
=============================

Single-shot metrics aggregator used by the HUD.

Pulls every panel's data in one DB transaction so the dashboard renders
in one pass and stays consistent across panels.

Returns plain dicts / DataFrames — no Streamlit imports here, so this module
is unit-testable without the streamlit dependency.
"""

from __future__ import annotations

import json
import logging
from dataclasses import dataclass, field
from datetime import datetime, timezone
from typing import Any, Dict, List, Optional

import pandas as pd

from ..data.database import LottoQuantDB
from ..execution.modes import OperatingMode, get_state
from ..execution.pnl import PnLSnapshot, PnLTracker

logger = logging.getLogger(__name__)


@dataclass
class DashboardMetrics:
    mode: OperatingMode
    bankroll_start: float
    pnl: PnLSnapshot
    snapshots: pd.DataFrame
    latest_per_game: pd.DataFrame
    signals: pd.DataFrame
    alerts: pd.DataFrame
    kelly_recs: pd.DataFrame
    fills: pd.DataFrame
    orders: pd.DataFrame
    markov_predictions: pd.DataFrame
    headline: Dict[str, Any] = field(default_factory=dict)


class MetricsService:
    """Aggregates everything the HUD needs in one go."""

    def __init__(self, db: Optional[LottoQuantDB] = None):
        self.db = db or LottoQuantDB()
        self.pnl = PnLTracker(self.db)

    # ── public ─────────────────────────────────────────────────────
    def collect(self) -> DashboardMetrics:
        state = get_state()
        mode = state.mode
        bankroll = state.paper_bankroll if mode == OperatingMode.PAPER else state.live_bankroll
        pnl_snap = self.pnl.snapshot(mode, bankroll_start=bankroll)

        snapshots_df = self._df(self._fetch_snapshots(limit=1_000))
        latest = self._latest_per_game(snapshots_df)
        signals_df = self._df(self._fetch_signals(limit=200))
        alerts_df = self._df(self._fetch_alerts(limit=200))
        kelly_df = self._df(self._fetch_kelly(limit=200))
        fills_df = self._df(self._fetch_fills(mode, limit=500))
        orders_df = self._df(self._fetch_orders(mode, limit=500))
        markov_df = self._df(self._fetch_markov(limit=100))

        headline = self._headline(latest, pnl_snap, signals_df, alerts_df)
        return DashboardMetrics(
            mode=mode,
            bankroll_start=bankroll,
            pnl=pnl_snap,
            snapshots=snapshots_df,
            latest_per_game=latest,
            signals=signals_df,
            alerts=alerts_df,
            kelly_recs=kelly_df,
            fills=fills_df,
            orders=orders_df,
            markov_predictions=markov_df,
            headline=headline,
        )

    # ── DB pulls (resilient to missing tables) ─────────────────────
    def _fetch_snapshots(self, limit: int) -> List[Dict]:
        try:
            cur = self.db._cursor()
            cur.execute(
                "SELECT snapshot_id, game_id, game_name, ticket_price, "
                "snapshot_ts, ev_gross, ev_adjusted, depletion_ratio, anomaly_score "
                "FROM scratch_off_snapshots ORDER BY snapshot_ts DESC LIMIT ?",
                (limit,),
            )
            rows = cur.fetchall()
        except Exception as e:
            logger.warning("snapshots query failed: %s", e)
            return []
        cols = [
            "snapshot_id", "game_id", "game_name", "ticket_price",
            "snapshot_ts", "ev_gross", "ev_adjusted", "depletion_ratio",
            "anomaly_score",
        ]
        return [dict(zip(cols, r)) for r in rows]

    def _fetch_signals(self, limit: int) -> List[Dict]:
        try:
            cur = self.db._cursor()
            cur.execute(
                "SELECT signal_id, game_id, signal_ts, signal_type, "
                "signal_strength, ev_net, recommended_allocation, alert_sent "
                "FROM ev_signals ORDER BY signal_ts DESC LIMIT ?",
                (limit,),
            )
            rows = cur.fetchall()
        except Exception as e:
            logger.warning("signals query failed: %s", e)
            return []
        cols = [
            "signal_id", "game_id", "signal_ts", "signal_type",
            "signal_strength", "ev_net", "recommended_allocation", "alert_sent",
        ]
        return [dict(zip(cols, r)) for r in rows]

    def _fetch_alerts(self, limit: int) -> List[Dict]:
        try:
            cur = self.db._cursor()
            cur.execute(
                "SELECT alert_id, alert_ts, game_id, message, channel, delivered "
                "FROM alert_log ORDER BY alert_ts DESC LIMIT ?",
                (limit,),
            )
            rows = cur.fetchall()
        except Exception as e:
            logger.warning("alerts query failed: %s", e)
            return []
        cols = ["alert_id", "alert_ts", "game_id", "message", "channel", "delivered"]
        return [dict(zip(cols, r)) for r in rows]

    def _fetch_kelly(self, limit: int) -> List[Dict]:
        try:
            cur = self.db._cursor()
            cur.execute(
                "SELECT rec_id, game_id, rec_ts, bankroll, full_kelly, "
                "final_fraction, position_usd, n_tickets, capped_by "
                "FROM kelly_recommendations ORDER BY rec_ts DESC LIMIT ?",
                (limit,),
            )
            rows = cur.fetchall()
        except Exception as e:
            logger.warning("kelly query failed: %s", e)
            return []
        cols = [
            "rec_id", "game_id", "rec_ts", "bankroll", "full_kelly",
            "final_fraction", "position_usd", "n_tickets", "capped_by",
        ]
        return [dict(zip(cols, r)) for r in rows]

    def _fetch_fills(self, mode: OperatingMode, limit: int) -> List[Dict]:
        try:
            cur = self.db._cursor()
            cur.execute(
                "SELECT fill_id, order_id, game_id, n_tickets, cost, "
                "gross_payout, net_payout, pnl_net, mode, filled_iso "
                "FROM exec_fills WHERE mode = ? ORDER BY filled_iso DESC LIMIT ?",
                (mode.value, limit),
            )
            rows = cur.fetchall()
        except Exception as e:
            logger.warning("fills query failed: %s", e)
            return []
        cols = [
            "fill_id", "order_id", "game_id", "n_tickets", "cost",
            "gross_payout", "net_payout", "pnl_net", "mode", "filled_iso",
        ]
        return [dict(zip(cols, r)) for r in rows]

    def _fetch_orders(self, mode: OperatingMode, limit: int) -> List[Dict]:
        try:
            cur = self.db._cursor()
            cur.execute(
                "SELECT order_id, game_id, game_name, ticket_price, n_tickets, "
                "expected_ev, mode, created_iso, status "
                "FROM exec_orders WHERE mode = ? ORDER BY created_iso DESC LIMIT ?",
                (mode.value, limit),
            )
            rows = cur.fetchall()
        except Exception as e:
            logger.warning("orders query failed: %s", e)
            return []
        cols = [
            "order_id", "game_id", "game_name", "ticket_price", "n_tickets",
            "expected_ev", "mode", "created_iso", "status",
        ]
        return [dict(zip(cols, r)) for r in rows]

    def _fetch_markov(self, limit: int) -> List[Dict]:
        try:
            cur = self.db._cursor()
            cur.execute(
                "SELECT pred_id, game_id, pred_ts, tickets_until_ev_positive, "
                "confidence, anomaly_score "
                "FROM markov_predictions ORDER BY pred_ts DESC LIMIT ?",
                (limit,),
            )
            rows = cur.fetchall()
        except Exception as e:
            logger.warning("markov query failed: %s", e)
            return []
        cols = [
            "pred_id", "game_id", "pred_ts", "tickets_until_ev_positive",
            "confidence", "anomaly_score",
        ]
        return [dict(zip(cols, r)) for r in rows]

    # ── helpers ────────────────────────────────────────────────────
    @staticmethod
    def _df(rows: List[Dict]) -> pd.DataFrame:
        if not rows:
            return pd.DataFrame()
        df = pd.DataFrame(rows)
        # Standardize timestamp dtype
        for col in ("snapshot_ts", "signal_ts", "alert_ts", "rec_ts",
                    "filled_iso", "created_iso", "pred_ts"):
            if col in df.columns:
                df[col] = pd.to_datetime(df[col], errors="coerce", utc=True)
        return df

    @staticmethod
    def _latest_per_game(snapshots: pd.DataFrame) -> pd.DataFrame:
        if snapshots.empty:
            return snapshots
        latest = (
            snapshots.sort_values("snapshot_ts")
            .groupby("game_id", as_index=False)
            .tail(1)
            .copy()
        )
        latest["ev_per_dollar"] = latest["ev_adjusted"] / latest["ticket_price"]
        return latest.sort_values("ev_per_dollar", ascending=False).reset_index(drop=True)

    @staticmethod
    def _headline(
        latest: pd.DataFrame,
        pnl: PnLSnapshot,
        signals: pd.DataFrame,
        alerts: pd.DataFrame,
    ) -> Dict[str, Any]:
        n_games = int(latest.shape[0]) if not latest.empty else 0
        n_pos = int((latest["ev_adjusted"] > 0).sum()) if not latest.empty else 0
        best_ev_pd = (
            float(latest["ev_per_dollar"].max()) if not latest.empty else 0.0
        )
        n_signals_24h = 0
        if not signals.empty:
            cutoff = datetime.now(timezone.utc) - pd.Timedelta(hours=24)
            n_signals_24h = int((signals["signal_ts"] >= cutoff).sum())
        n_alerts_24h = 0
        if not alerts.empty:
            cutoff = datetime.now(timezone.utc) - pd.Timedelta(hours=24)
            n_alerts_24h = int((alerts["alert_ts"] >= cutoff).sum())
        return {
            "games_tracked": n_games,
            "ev_positive_games": n_pos,
            "best_ev_per_dollar": best_ev_pd,
            "signals_last_24h": n_signals_24h,
            "alerts_last_24h": n_alerts_24h,
            "realized_pnl": pnl.realized_pnl,
            "bankroll_current": pnl.bankroll_current,
            "bankroll_start": pnl.bankroll_start,
            "n_fills": pnl.n_fills,
            "n_orders": pnl.n_orders,
            "win_rate": pnl.win_rate,
            "hit_rate": pnl.hit_rate,
            "max_drawdown": pnl.max_drawdown,
            "sharpe_like": pnl.sharpe_like,
            "mode": pnl.mode.value,
        }
