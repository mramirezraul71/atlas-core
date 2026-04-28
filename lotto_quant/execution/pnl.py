"""
lotto_quant.execution.pnl
=========================

P&L tracker — reads execution fills from DuckDB and produces equity curves,
per-game performance, win-rate, hit-rate, Sharpe-equivalent, max drawdown.

The metrics are computed *per mode* (paper vs live) so the dashboard can
toggle between the two without ever mixing them.
"""

from __future__ import annotations

import logging
import math
from dataclasses import dataclass, field
from datetime import datetime
from typing import Dict, List, Optional

from ..data.database import LottoQuantDB
from .modes import OperatingMode

logger = logging.getLogger(__name__)


@dataclass
class PnLSnapshot:
    mode: OperatingMode
    bankroll_start: float
    bankroll_current: float
    realized_pnl: float
    n_orders: int
    n_fills: int
    total_cost: float
    total_gross_payout: float
    total_net_payout: float
    win_count: int
    hit_count: int            # fills with non-zero payout
    win_rate: float           # fills with pnl_net >= 0
    hit_rate: float           # fills with non-zero payout
    avg_ev_per_dollar: float
    sharpe_like: float
    max_drawdown: float
    equity_curve: List[Dict] = field(default_factory=list)
    per_game: List[Dict] = field(default_factory=list)


class PnLTracker:
    """Reads `exec_fills` and produces an aggregated PnLSnapshot."""

    def __init__(self, db: LottoQuantDB):
        self.db = db
        # Ensure exec_orders / exec_fills exist before any read.
        from .broker import ensure_broker_tables
        ensure_broker_tables(self.db)

    # ── public API ────────────────────────────────────────────────
    def snapshot(
        self,
        mode: OperatingMode,
        bankroll_start: float,
    ) -> PnLSnapshot:
        fills = self._fetch_fills(mode)
        n_orders = self._count_orders(mode)
        cost = sum(f["cost"] for f in fills)
        gross = sum(f["gross_payout"] for f in fills)
        net = sum(f["net_payout"] for f in fills)
        realized = sum(f["pnl_net"] for f in fills)
        wins = sum(1 for f in fills if f["pnl_net"] >= 0)
        hits = sum(1 for f in fills if f["gross_payout"] > 0)
        n_f = len(fills)

        equity = self._equity_curve(bankroll_start, fills)
        per_game = self._per_game(fills)
        win_rate = (wins / n_f) if n_f else 0.0
        hit_rate = (hits / n_f) if n_f else 0.0
        avg_ev_pd = (net - cost) / cost if cost > 0 else 0.0
        sharpe = self._sharpe_like(fills)
        max_dd = self._max_drawdown(equity)

        return PnLSnapshot(
            mode=mode,
            bankroll_start=bankroll_start,
            bankroll_current=bankroll_start + realized,
            realized_pnl=realized,
            n_orders=n_orders,
            n_fills=n_f,
            total_cost=cost,
            total_gross_payout=gross,
            total_net_payout=net,
            win_count=wins,
            hit_count=hits,
            win_rate=win_rate,
            hit_rate=hit_rate,
            avg_ev_per_dollar=avg_ev_pd,
            sharpe_like=sharpe,
            max_drawdown=max_dd,
            equity_curve=equity,
            per_game=per_game,
        )

    # ── DB readers ─────────────────────────────────────────────────
    def _fetch_fills(self, mode: OperatingMode) -> List[Dict]:
        cur = self.db._cursor()
        cur.execute(
            "SELECT fill_id, order_id, game_id, n_tickets, cost, "
            "gross_payout, net_payout, pnl_net, mode, filled_iso "
            "FROM exec_fills WHERE mode = ? ORDER BY filled_iso ASC",
            (mode.value,),
        )
        rows = cur.fetchall()
        cols = [
            "fill_id", "order_id", "game_id", "n_tickets", "cost",
            "gross_payout", "net_payout", "pnl_net", "mode", "filled_iso",
        ]
        return [dict(zip(cols, r)) for r in rows]

    def _count_orders(self, mode: OperatingMode) -> int:
        cur = self.db._cursor()
        cur.execute(
            "SELECT COUNT(*) FROM exec_orders WHERE mode = ?", (mode.value,)
        )
        row = cur.fetchone()
        return int(row[0]) if row else 0

    # ── derived metrics ────────────────────────────────────────────
    @staticmethod
    def _equity_curve(start: float, fills: List[Dict]) -> List[Dict]:
        curve = [{"ts": "T0", "equity": start, "pnl_step": 0.0}]
        eq = start
        for f in fills:
            eq += f["pnl_net"]
            curve.append({
                "ts": f["filled_iso"],
                "equity": eq,
                "pnl_step": f["pnl_net"],
            })
        return curve

    @staticmethod
    def _per_game(fills: List[Dict]) -> List[Dict]:
        agg: Dict[str, Dict] = {}
        for f in fills:
            g = agg.setdefault(
                f["game_id"],
                {
                    "game_id": f["game_id"],
                    "n_fills": 0,
                    "n_tickets": 0,
                    "cost": 0.0,
                    "net_payout": 0.0,
                    "pnl_net": 0.0,
                    "wins": 0,
                    "hits": 0,
                },
            )
            g["n_fills"] += 1
            g["n_tickets"] += f["n_tickets"]
            g["cost"] += f["cost"]
            g["net_payout"] += f["net_payout"]
            g["pnl_net"] += f["pnl_net"]
            if f["pnl_net"] >= 0:
                g["wins"] += 1
            if f["gross_payout"] > 0:
                g["hits"] += 1
        out = []
        for g in agg.values():
            g["roi"] = g["pnl_net"] / g["cost"] if g["cost"] > 0 else 0.0
            out.append(g)
        return sorted(out, key=lambda x: x["pnl_net"], reverse=True)

    @staticmethod
    def _sharpe_like(fills: List[Dict]) -> float:
        """ROI-per-fill mean / stdev. Not annualized — lottery has no clock."""
        if len(fills) < 2:
            return 0.0
        rois = [
            (f["pnl_net"] / f["cost"]) if f["cost"] > 0 else 0.0 for f in fills
        ]
        mean = sum(rois) / len(rois)
        var = sum((r - mean) ** 2 for r in rois) / (len(rois) - 1)
        std = math.sqrt(var)
        return (mean / std) if std > 0 else 0.0

    @staticmethod
    def _max_drawdown(equity: List[Dict]) -> float:
        if not equity:
            return 0.0
        peak = equity[0]["equity"]
        max_dd = 0.0
        for e in equity:
            v = e["equity"]
            if v > peak:
                peak = v
            dd = (peak - v) / peak if peak > 0 else 0.0
            if dd > max_dd:
                max_dd = dd
        return max_dd
