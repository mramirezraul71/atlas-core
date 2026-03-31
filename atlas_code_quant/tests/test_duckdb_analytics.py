"""Tests for learning.duckdb_analytics — DuckDB columnar analytics engine."""
import sqlite3
import tempfile
from pathlib import Path

import pytest


@pytest.fixture
def journal_db(tmp_path):
    """Create a minimal SQLite journal for testing."""
    db_path = tmp_path / "test_journal.sqlite3"
    con = sqlite3.connect(str(db_path))
    con.execute("""
        CREATE TABLE trading_journal (
            id INTEGER PRIMARY KEY,
            symbol TEXT,
            strategy_type TEXT,
            account_type TEXT DEFAULT 'paper',
            status TEXT DEFAULT 'closed',
            entry_time TEXT,
            exit_time TEXT,
            realized_pnl REAL DEFAULT 0,
            risk_at_entry REAL DEFAULT 0
        )
    """)
    # Insert test trades
    trades = [
        (1, "SPY", "equity_long", "paper", "closed", "2026-03-30T10:00:00", "2026-03-30T14:00:00", 50.0, 100.0),
        (2, "SPY", "equity_long", "paper", "closed", "2026-03-30T10:30:00", "2026-03-30T14:30:00", -30.0, 100.0),
        (3, "AAPL", "equity_long", "paper", "closed", "2026-03-30T11:00:00", "2026-03-30T15:00:00", 80.0, 50.0),
        (4, "QQQ", "equity_short", "paper", "closed", "2026-03-31T09:00:00", "2026-03-31T12:00:00", -20.0, 80.0),
        (5, "SPY", "equity_long", "paper", "closed", "2026-03-31T10:00:00", "2026-03-31T14:00:00", 60.0, 100.0),
    ]
    con.executemany(
        "INSERT INTO trading_journal (id, symbol, strategy_type, account_type, status, entry_time, exit_time, realized_pnl, risk_at_entry) VALUES (?,?,?,?,?,?,?,?,?)",
        trades,
    )
    con.commit()
    con.close()
    return db_path


@pytest.fixture
def engine(journal_db):
    from learning.duckdb_analytics import JournalAnalyticsEngine
    eng = JournalAnalyticsEngine(journal_path=journal_db, epoch_start="2026-03-01")
    return eng


def test_refresh_loads_rows(engine):
    count = engine.refresh(force=True)
    assert count == 5


def test_equity_curve(engine):
    curve = engine.equity_curve("paper")
    assert len(curve) == 5
    # Cumulative: 50, 20, 100, 80, 140
    assert curve[0]["pnl"] == 50.0
    assert curve[-1]["equity"] == 140.0


def test_daily_pnl(engine):
    days = engine.daily_pnl("paper")
    assert len(days) == 2  # March 30 and March 31
    march30 = [d for d in days if "2026-03-30" in d["date"]][0]
    assert march30["pnl"] == 100.0  # 50 - 30 + 80
    assert march30["trades"] == 3


def test_r_multiple_distribution(engine):
    dist = engine.r_multiple_distribution("paper")
    assert dist["count"] == 5
    assert "expectancy" in dist
    assert "median" in dist


def test_strategy_performance(engine):
    perf = engine.strategy_performance("paper")
    assert len(perf) >= 1
    long_perf = [p for p in perf if p["strategy_type"] == "equity_long"][0]
    assert long_perf["trades"] == 4
    assert long_perf["total_pnl"] == 160.0  # 50 - 30 + 80 + 60


def test_factor_correlation(engine):
    risk = engine.factor_correlation("paper")
    assert "risk_level" in risk
    assert "alerts" in risk
    # SPY has 3 out of 5 trades = 60%, should trigger WARNING
    spy_alerts = [a for a in risk["alerts"] if a.get("symbol") == "SPY"]
    assert len(spy_alerts) >= 1


def test_full_analytics(engine):
    full = engine.full_analytics("paper")
    assert "equity_curve" in full
    assert "daily_pnl" in full
    assert "r_distribution" in full
    assert "strategy_performance" in full
    assert "factor_correlation" in full
    assert full["row_count"] == 5


def test_refresh_ttl_skips(engine):
    engine.refresh(force=True)
    # Second call within TTL should return cached count
    count = engine.refresh(force=False, ttl_sec=60)
    assert count == 5


def test_empty_journal(tmp_path):
    from learning.duckdb_analytics import JournalAnalyticsEngine
    db = tmp_path / "empty.sqlite3"
    con = sqlite3.connect(str(db))
    con.execute("""
        CREATE TABLE trading_journal (
            id INTEGER PRIMARY KEY, symbol TEXT, strategy_type TEXT,
            account_type TEXT, status TEXT, entry_time TEXT, exit_time TEXT,
            realized_pnl REAL, risk_at_entry REAL
        )
    """)
    con.commit()
    con.close()
    eng = JournalAnalyticsEngine(journal_path=db, epoch_start="2026-03-01")
    curve = eng.equity_curve("paper")
    assert curve == []
