"""DuckDB-powered analytics engine for the trading journal.

Provides sub-100ms analytical queries over 77K+ journal rows by loading
the SQLite journal into DuckDB's columnar engine at startup and refreshing
periodically.  All heavy aggregation (equity curves, factor correlation,
daily PnL calendars, R-multiple distributions) runs here instead of in
Python loops.

Usage:
    engine = JournalAnalyticsEngine()
    engine.refresh()                        # load/reload from SQLite
    result = engine.equity_curve("paper")   # <50ms even on 100K rows
"""
from __future__ import annotations

import logging
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import duckdb

from config.settings import settings

logger = logging.getLogger("atlas.analytics.duckdb")

_JOURNAL_DB = Path(settings.journal_db_path)

# Epoch start — ignore pre-reset contaminated data
_EPOCH_START = getattr(settings, "adaptive_learning_epoch_start", "2026-03-30")


class JournalAnalyticsEngine:
    """Columnar analytics engine backed by DuckDB."""

    def __init__(self, journal_path: Path | None = None, epoch_start: str | None = None):
        self._journal_path = journal_path or _JOURNAL_DB
        self._epoch_start = epoch_start or _EPOCH_START
        self._con: duckdb.DuckDBPyConnection | None = None
        self._loaded_at: float = 0
        self._row_count: int = 0

    # ── lifecycle ──────────────────────────────────────────────────
    def _ensure_connection(self) -> duckdb.DuckDBPyConnection:
        if self._con is None:
            self._con = duckdb.connect(":memory:")
            self._con.execute("INSTALL sqlite; LOAD sqlite;")
        return self._con

    def refresh(self, force: bool = False, ttl_sec: int = 60) -> int:
        """Load or reload journal data from SQLite into DuckDB.

        Returns number of rows loaded.  Skips reload if data is fresh
        (within *ttl_sec*) unless *force* is True.
        """
        if not force and self._loaded_at and (time.time() - self._loaded_at) < ttl_sec:
            return self._row_count

        t0 = time.perf_counter()
        con = self._ensure_connection()

        if not self._journal_path.exists():
            logger.warning("Journal DB not found at %s", self._journal_path)
            return 0

        con.execute("DROP TABLE IF EXISTS journal")
        con.execute(f"""
            CREATE TABLE journal AS
            SELECT * FROM sqlite_scan('{self._journal_path.as_posix()}', 'trading_journal')
            WHERE status = 'closed'
              AND (exit_time >= '{self._epoch_start}' OR exit_time IS NULL)
        """)
        self._row_count = con.execute("SELECT count(*) FROM journal").fetchone()[0]
        self._loaded_at = time.time()

        ms = (time.perf_counter() - t0) * 1000
        logger.info("DuckDB journal loaded: %d rows in %.1fms", self._row_count, ms)
        return self._row_count

    # ── queries ────────────────────────────────────────────────────
    def equity_curve(self, account_scope: str = "paper", limit: int = 2000) -> list[dict]:
        """Cumulative equity curve with drawdown, ordered by exit_time."""
        self.refresh()
        con = self._ensure_connection()
        rows = con.execute("""
            WITH base AS (
                SELECT
                    row_number() OVER (ORDER BY exit_time, id) AS n,
                    symbol,
                    strategy_type,
                    exit_time,
                    realized_pnl,
                    risk_at_entry,
                    SUM(realized_pnl) OVER (ORDER BY exit_time, id) AS equity,
                FROM journal
                WHERE account_type = ?
                ORDER BY exit_time, id
                LIMIT ?
            )
            SELECT
                n,
                symbol,
                strategy_type,
                exit_time,
                ROUND(realized_pnl, 2) AS pnl,
                ROUND(equity, 2) AS equity,
                ROUND(CASE
                    WHEN MAX(equity) OVER (ORDER BY n ROWS BETWEEN UNBOUNDED PRECEDING AND CURRENT ROW) > 0
                    THEN (equity - MAX(equity) OVER (ORDER BY n ROWS BETWEEN UNBOUNDED PRECEDING AND CURRENT ROW))
                         / MAX(equity) OVER (ORDER BY n ROWS BETWEEN UNBOUNDED PRECEDING AND CURRENT ROW) * 100
                    ELSE 0 END, 2) AS drawdown_pct,
                ROUND(CASE WHEN ABS(COALESCE(risk_at_entry, 0)) > 0
                    THEN realized_pnl / ABS(risk_at_entry) ELSE 0 END, 3) AS r_multiple,
                realized_pnl > 0 AS win
            FROM base
        """, [account_scope, limit]).fetchall()

        cols = ["n", "symbol", "strategy_type", "exit_time", "pnl",
                "equity", "drawdown_pct", "r_multiple", "win"]
        return [dict(zip(cols, row)) for row in rows]

    def daily_pnl(self, account_scope: str = "paper") -> list[dict]:
        """Daily PnL aggregation for calendar view."""
        self.refresh()
        con = self._ensure_connection()
        rows = con.execute("""
            SELECT
                CAST(exit_time AS DATE) AS day,
                ROUND(SUM(realized_pnl), 2) AS pnl,
                COUNT(*) AS trades,
                MODE(strategy_type) AS dominant_strategy
            FROM journal
            WHERE account_type = ? AND exit_time IS NOT NULL
            GROUP BY CAST(exit_time AS DATE)
            ORDER BY day
        """, [account_scope]).fetchall()
        return [{"date": str(r[0]), "pnl": r[1], "trades": r[2],
                 "dominant_strategy": r[3] or ""} for r in rows]

    def r_multiple_distribution(self, account_scope: str = "paper") -> dict:
        """R-multiple histogram + expectancy."""
        self.refresh()
        con = self._ensure_connection()
        rows = con.execute("""
            SELECT
                ROUND(CASE WHEN ABS(COALESCE(risk_at_entry, 0)) > 0
                    THEN realized_pnl / ABS(risk_at_entry) ELSE 0 END, 2) AS r
            FROM journal
            WHERE account_type = ?
              AND ABS(COALESCE(risk_at_entry, 0)) > 0
        """, [account_scope]).fetchall()

        r_vals = [r[0] for r in rows]
        if not r_vals:
            return {"values": [], "expectancy": 0, "count": 0}

        expectancy = sum(r_vals) / len(r_vals)
        return {
            "values": r_vals,
            "expectancy": round(expectancy, 4),
            "count": len(r_vals),
            "median": round(sorted(r_vals)[len(r_vals) // 2], 4),
        }

    def strategy_performance(self, account_scope: str = "paper") -> list[dict]:
        """Performance breakdown by strategy type."""
        self.refresh()
        con = self._ensure_connection()
        rows = con.execute("""
            SELECT
                strategy_type,
                COUNT(*) AS trades,
                SUM(CASE WHEN realized_pnl > 0 THEN 1 ELSE 0 END) AS wins,
                ROUND(SUM(CASE WHEN realized_pnl > 0 THEN 1 ELSE 0 END) * 100.0 / COUNT(*), 2) AS win_rate,
                ROUND(SUM(realized_pnl), 2) AS total_pnl,
                ROUND(AVG(realized_pnl), 2) AS avg_pnl,
                ROUND(CASE WHEN SUM(CASE WHEN realized_pnl < 0 THEN ABS(realized_pnl) ELSE 0 END) > 0
                    THEN SUM(CASE WHEN realized_pnl > 0 THEN realized_pnl ELSE 0 END)
                         / SUM(CASE WHEN realized_pnl < 0 THEN ABS(realized_pnl) ELSE 0 END)
                    ELSE 999 END, 3) AS profit_factor
            FROM journal
            WHERE account_type = ?
            GROUP BY strategy_type
            ORDER BY total_pnl DESC
        """, [account_scope]).fetchall()

        cols = ["strategy_type", "trades", "wins", "win_rate",
                "total_pnl", "avg_pnl", "profit_factor"]
        return [dict(zip(cols, r)) for r in rows]

    def factor_correlation(self, account_scope: str = "paper", threshold: float = 0.6) -> dict:
        """Detect sector/symbol concentration risk.

        Returns correlation risk alert if any single symbol or strategy
        accounts for more than *threshold* (60%) of open exposure.
        """
        self.refresh()
        con = self._ensure_connection()

        # Symbol concentration in recent trades (last 20)
        rows = con.execute("""
            SELECT
                symbol,
                COUNT(*) AS cnt,
                ROUND(COUNT(*) * 100.0 / SUM(COUNT(*)) OVER (), 2) AS pct
            FROM (
                SELECT symbol FROM journal
                WHERE account_type = ?
                ORDER BY exit_time DESC
                LIMIT 20
            )
            GROUP BY symbol
            ORDER BY cnt DESC
        """, [account_scope]).fetchall()

        alerts = []
        for symbol, cnt, pct in rows:
            if pct >= threshold * 100:
                alerts.append({
                    "type": "symbol_concentration",
                    "symbol": symbol,
                    "pct": pct,
                    "trades": cnt,
                    "severity": "CRITICAL" if pct >= 80 else "WARNING",
                })

        # Strategy concentration
        strat_rows = con.execute("""
            SELECT
                strategy_type,
                COUNT(*) AS cnt,
                ROUND(COUNT(*) * 100.0 / SUM(COUNT(*)) OVER (), 2) AS pct
            FROM (
                SELECT strategy_type FROM journal
                WHERE account_type = ?
                ORDER BY exit_time DESC
                LIMIT 20
            )
            GROUP BY strategy_type
            ORDER BY cnt DESC
        """, [account_scope]).fetchall()

        for strat, cnt, pct in strat_rows:
            if pct >= threshold * 100:
                alerts.append({
                    "type": "strategy_concentration",
                    "strategy": strat,
                    "pct": pct,
                    "trades": cnt,
                    "severity": "CRITICAL" if pct >= 80 else "WARNING",
                })

        return {
            "timestamp": datetime.now(timezone.utc).isoformat(),
            "threshold_pct": threshold * 100,
            "alerts": alerts,
            "risk_level": "CRITICAL" if any(a["severity"] == "CRITICAL" for a in alerts)
                          else "WARNING" if alerts else "OK",
        }

    def full_analytics(self, account_scope: str = "paper") -> dict:
        """Complete analytics bundle for the dashboard — single call."""
        t0 = time.perf_counter()
        self.refresh()

        result = {
            "generated_at": datetime.now(timezone.utc).isoformat(),
            "account_scope": account_scope,
            "row_count": self._row_count,
            "equity_curve": self.equity_curve(account_scope),
            "daily_pnl": self.daily_pnl(account_scope),
            "r_distribution": self.r_multiple_distribution(account_scope),
            "strategy_performance": self.strategy_performance(account_scope),
            "factor_correlation": self.factor_correlation(account_scope),
            "query_ms": round((time.perf_counter() - t0) * 1000, 1),
        }
        logger.info("Full analytics generated in %.1fms (%d rows)",
                     result["query_ms"], self._row_count)
        return result


# Singleton
_engine: JournalAnalyticsEngine | None = None


def get_analytics_engine() -> JournalAnalyticsEngine:
    global _engine
    if _engine is None:
        _engine = JournalAnalyticsEngine()
    return _engine
