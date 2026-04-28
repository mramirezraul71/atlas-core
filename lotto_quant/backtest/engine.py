"""
lotto_quant.backtest.engine
===========================

Historical replay backtester (paper mode).

What it does
------------
1. Reads ``scratch_off_snapshots`` rows from DuckDB / SQLite.
2. Buckets snapshots by calendar day, keeping the most recent snapshot
   per ``game_id`` within each bucket.
3. For each day in chronological order:
     a. Reconstructs ``ScratchOffGame`` objects from ``data_json``.
     b. Runs ``EVCalculator.calculate_adjusted_ev`` per game.
     c. Filters games with ``adjusted_ev_nc > 0`` (or any custom filter).
     d. Calls ``KellyAllocator.calculate_lottery_kelly`` to size positions.
     e. Submits orders through a fresh ``PaperBroker`` (Monte-Carlo fills).
     f. Aggregates daily P&L into the bankroll for the next day.

The backtester writes everything to an **ephemeral DuckDB / SQLite file**
(passed in via ``BacktestConfig.workspace_db``) so it never pollutes the
production database that powers the live HUD. The user picks the source
DB containing snapshots; all writes (orders, fills, alerts) go to the
workspace DB.

Why this matters
----------------
- It lets the user calibrate the strategy (Kelly fraction, EV cutoff,
  per-day cap) on the historical record before letting capital touch
  the LIVE flow.
- It produces a daily equity curve, hit-rate, max drawdown, and a
  per-game breakdown — same metrics the HUD already plots, so we can
  reuse the chart components.

Atlas constraint reminder
-------------------------
This module replays purchase **decisions** in paper mode. It NEVER
records LIVE orders. The Atlas-wide constraint (no auto-execution of
real lottery purchases) holds — backtesting is a Monte-Carlo simulation.
"""

from __future__ import annotations

import json
import logging
import uuid
from collections import defaultdict
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

from ..data.database import LottoQuantDB
from ..execution.broker import PaperBroker, ensure_broker_tables
from ..models.ev_calculator import (
    EVCalculator,
    EVResult,
    PrizeTier,
    ScratchOffGame,
)
from ..models.kelly_allocator import KellyAllocator

logger = logging.getLogger(__name__)


# ─────────────────────────────────────────────────────────────────────
# Config / result types
# ─────────────────────────────────────────────────────────────────────
@dataclass
class BacktestConfig:
    """Tunable parameters for a backtest run."""

    source_db: Optional[str] = None
    """Path to the DB to read historical snapshots from. Default = production DB."""

    workspace_db: Optional[str] = None
    """Path to the ephemeral DB where simulated orders/fills are written."""

    bankroll_start: float = 1_000.0
    kelly_fraction: float = 0.25
    max_position_pct: float = 0.05
    """Max fraction of bankroll allocated to a single game on a single day."""

    max_daily_exposure: float = 0.20
    """Max fraction of bankroll spent across all games on a single day."""

    min_ev_per_dollar: float = 0.0
    """Skip games whose EV/$ is below this threshold."""

    min_tickets: int = 1
    """Minimum tickets to actually submit an order (smaller orders are skipped)."""

    rng_seed: int = 1234
    """Seed for the Monte-Carlo prize draws — reproducible runs."""

    days_back: Optional[int] = None
    """If set, only replay the last N calendar days of snapshots."""


@dataclass
class DailyResult:
    """Aggregated result for one calendar day in the replay."""

    day: str
    bankroll_open: float
    bankroll_close: float
    games_seen: int
    games_played: int
    total_cost: float
    gross_payout: float
    net_payout: float
    pnl_net: float
    drawdown: float

    def to_dict(self) -> Dict[str, Any]:
        return {
            "day": self.day,
            "bankroll_open": self.bankroll_open,
            "bankroll_close": self.bankroll_close,
            "games_seen": self.games_seen,
            "games_played": self.games_played,
            "total_cost": self.total_cost,
            "gross_payout": self.gross_payout,
            "net_payout": self.net_payout,
            "pnl_net": self.pnl_net,
            "drawdown": self.drawdown,
        }


@dataclass
class BacktestResult:
    """Full backtest output, ready for the HUD or a CLI report."""

    config: BacktestConfig
    days: List[DailyResult] = field(default_factory=list)
    n_orders: int = 0
    n_fills: int = 0
    final_bankroll: float = 0.0
    total_cost: float = 0.0
    total_net_payout: float = 0.0
    realized_pnl: float = 0.0
    max_drawdown: float = 0.0
    sharpe_like: float = 0.0
    win_rate: float = 0.0
    per_game_pnl: Dict[str, float] = field(default_factory=dict)
    notes: List[str] = field(default_factory=list)

    @property
    def equity_curve(self) -> List[Tuple[str, float]]:
        out = [("T0", self.config.bankroll_start)]
        for d in self.days:
            out.append((d.day, d.bankroll_close))
        return out

    def summary_dict(self) -> Dict[str, Any]:
        return {
            "n_days": len(self.days),
            "n_orders": self.n_orders,
            "n_fills": self.n_fills,
            "bankroll_start": self.config.bankroll_start,
            "final_bankroll": self.final_bankroll,
            "total_cost": self.total_cost,
            "total_net_payout": self.total_net_payout,
            "realized_pnl": self.realized_pnl,
            "max_drawdown": self.max_drawdown,
            "sharpe_like": self.sharpe_like,
            "win_rate": self.win_rate,
            "per_game_pnl": dict(self.per_game_pnl),
            "notes": list(self.notes),
        }


# ─────────────────────────────────────────────────────────────────────
# Engine
# ─────────────────────────────────────────────────────────────────────
class BacktestEngine:
    """Replays historical snapshots through the paper broker."""

    def __init__(self, cfg: BacktestConfig):
        self.cfg = cfg

    # ── public API ────────────────────────────────────────────────
    def run(self) -> BacktestResult:
        snaps = self._load_snapshots()
        if not snaps:
            return BacktestResult(
                config=self.cfg,
                final_bankroll=self.cfg.bankroll_start,
                notes=["No snapshots available — backtest skipped."],
            )

        days = self._bucket_by_day(snaps)
        if self.cfg.days_back and self.cfg.days_back > 0:
            days = days[-self.cfg.days_back:]

        ev_calc = EVCalculator()
        kelly = KellyAllocator(
            kelly_fraction=self.cfg.kelly_fraction,
            max_position_pct=self.cfg.max_position_pct,
        )

        ws_db = LottoQuantDB(self.cfg.workspace_db) if self.cfg.workspace_db else LottoQuantDB()
        ensure_broker_tables(ws_db)
        # Use a deterministic RNG so the same backtest config produces
        # the same numbers — vital when tuning Kelly fraction etc.
        broker = PaperBroker(ws_db, rng_seed=self.cfg.rng_seed)

        bankroll = self.cfg.bankroll_start
        peak = bankroll
        per_game_pnl: Dict[str, float] = defaultdict(float)
        daily_results: List[DailyResult] = []
        n_orders = 0
        n_fills = 0
        total_cost = 0.0
        total_net = 0.0

        for day, day_snaps in days:
            opening = bankroll
            day_cost = 0.0
            day_gross = 0.0
            day_net = 0.0
            games_played = 0
            day_exposure_cap = self.cfg.max_daily_exposure * bankroll

            # Score every game first so we can prioritize by EV/$
            scored: List[Tuple[float, ScratchOffGame, EVResult]] = []
            for snap in day_snaps:
                game = self._reconstruct_game(snap)
                if game is None:
                    continue
                try:
                    ev = ev_calc.calculate_adjusted_ev(game)
                except Exception as e:  # pragma: no cover
                    logger.debug("EV calc failed for %s: %s", game.game_id, e)
                    continue
                if ev.ev_per_dollar < self.cfg.min_ev_per_dollar:
                    continue
                scored.append((ev.ev_per_dollar, game, ev))

            scored.sort(key=lambda t: t[0], reverse=True)

            for _ev_pd, game, ev in scored:
                if bankroll <= 0:
                    break
                if day_cost >= day_exposure_cap:
                    break

                rec = kelly.calculate_lottery_kelly(
                    ev_result=ev,
                    bankroll=bankroll,
                    current_total_exposure=day_cost,
                    game=game,
                )
                if rec.n_tickets < self.cfg.min_tickets:
                    continue
                # Cap by remaining daily budget
                budget_left = day_exposure_cap - day_cost
                affordable = int(budget_left // game.ticket_price)
                if affordable < self.cfg.min_tickets:
                    continue
                tickets = min(rec.n_tickets, affordable)
                if tickets < self.cfg.min_tickets:
                    continue

                order = broker.submit_order(
                    game=game, n_tickets=tickets, expected_ev=ev.ev_per_dollar,
                )
                # Pull the matching fill back out of the broker DB
                fill = self._latest_fill_for_order(ws_db, order.order_id)
                n_orders += 1
                if fill is not None:
                    n_fills += 1
                    day_cost += fill["cost"]
                    day_gross += fill["gross_payout"]
                    day_net += fill["net_payout"]
                    per_game_pnl[game.game_id] += fill["pnl_net"]
                    games_played += 1

            bankroll += (day_net - day_cost)
            peak = max(peak, bankroll)
            dd = 0.0 if peak <= 0 else (peak - bankroll) / peak

            total_cost += day_cost
            total_net += day_net

            daily_results.append(DailyResult(
                day=day,
                bankroll_open=opening,
                bankroll_close=bankroll,
                games_seen=len(day_snaps),
                games_played=games_played,
                total_cost=day_cost,
                gross_payout=day_gross,
                net_payout=day_net,
                pnl_net=day_net - day_cost,
                drawdown=dd,
            ))

        result = BacktestResult(
            config=self.cfg,
            days=daily_results,
            n_orders=n_orders,
            n_fills=n_fills,
            final_bankroll=bankroll,
            total_cost=total_cost,
            total_net_payout=total_net,
            realized_pnl=bankroll - self.cfg.bankroll_start,
            max_drawdown=max((d.drawdown for d in daily_results), default=0.0),
            sharpe_like=self._sharpe_like(daily_results),
            win_rate=self._win_rate(daily_results),
            per_game_pnl=dict(per_game_pnl),
        )
        return result

    # ── helpers ───────────────────────────────────────────────────
    def _load_snapshots(self) -> List[Dict[str, Any]]:
        db = LottoQuantDB(self.cfg.source_db) if self.cfg.source_db else LottoQuantDB()
        cur = db._cursor()
        cur.execute(
            "SELECT snapshot_id, game_id, game_name, ticket_price, "
            "snapshot_ts, data_json, ev_gross, ev_adjusted, "
            "depletion_ratio, anomaly_score "
            "FROM scratch_off_snapshots "
            "ORDER BY snapshot_ts ASC"
        )
        cols = [
            "snapshot_id", "game_id", "game_name", "ticket_price",
            "snapshot_ts", "data_json", "ev_gross", "ev_adjusted",
            "depletion_ratio", "anomaly_score",
        ]
        rows = cur.fetchall()
        return [dict(zip(cols, r)) for r in rows]

    @staticmethod
    def _bucket_by_day(snaps: List[Dict[str, Any]]) -> List[Tuple[str, List[Dict[str, Any]]]]:
        """Group snapshots by calendar day, keep most-recent per (day, game_id)."""
        by_day: Dict[str, Dict[str, Dict[str, Any]]] = defaultdict(dict)
        for s in snaps:
            ts = s["snapshot_ts"]
            day = BacktestEngine._day_key(ts)
            existing = by_day[day].get(s["game_id"])
            if existing is None or str(s["snapshot_ts"]) > str(existing["snapshot_ts"]):
                by_day[day][s["game_id"]] = s
        out: List[Tuple[str, List[Dict[str, Any]]]] = []
        for day in sorted(by_day.keys()):
            out.append((day, list(by_day[day].values())))
        return out

    @staticmethod
    def _day_key(ts: Any) -> str:
        if isinstance(ts, datetime):
            return ts.date().isoformat()
        s = str(ts)
        return s.split("T")[0].split(" ")[0] if s else "unknown"

    @staticmethod
    def _reconstruct_game(snap: Dict[str, Any]) -> Optional[ScratchOffGame]:
        try:
            payload = json.loads(snap["data_json"]) if snap.get("data_json") else {}
        except Exception:
            return None
        tier_dicts = payload.get("prize_tiers") or []
        tiers: List[PrizeTier] = []
        for td in tier_dicts:
            try:
                tiers.append(PrizeTier(
                    value=float(td.get("value", 0.0)),
                    total_prizes=int(td.get("total_prizes", 0)),
                    remaining_prizes=int(td.get("remaining_prizes", 0)),
                    odds_initial=td.get("odds_initial"),
                ))
            except Exception:
                continue
        if not tiers:
            return None
        return ScratchOffGame(
            game_id=str(snap["game_id"]),
            name=str(snap["game_name"]),
            ticket_price=float(snap.get("ticket_price") or 0.0),
            prize_tiers=tiers,
            snapshot_ts=str(snap.get("snapshot_ts") or ""),
        )

    @staticmethod
    def _latest_fill_for_order(db: LottoQuantDB, order_id: str) -> Optional[Dict[str, Any]]:
        cur = db._cursor()
        cur.execute(
            "SELECT cost, gross_payout, net_payout, pnl_net "
            "FROM exec_fills WHERE order_id = ? ORDER BY filled_iso DESC LIMIT 1",
            (order_id,),
        )
        r = cur.fetchone()
        if not r:
            return None
        return {
            "cost": float(r[0]),
            "gross_payout": float(r[1]),
            "net_payout": float(r[2]),
            "pnl_net": float(r[3]),
        }

    @staticmethod
    def _sharpe_like(days: List[DailyResult]) -> float:
        if not days:
            return 0.0
        rets = np.array([d.pnl_net / max(1.0, d.bankroll_open) for d in days], dtype=np.float64)
        if rets.size < 2 or rets.std(ddof=1) == 0.0:
            return 0.0
        return float(rets.mean() / rets.std(ddof=1) * np.sqrt(252))

    @staticmethod
    def _win_rate(days: List[DailyResult]) -> float:
        played = [d for d in days if d.games_played > 0]
        if not played:
            return 0.0
        wins = sum(1 for d in played if d.pnl_net > 0)
        return wins / len(played)
