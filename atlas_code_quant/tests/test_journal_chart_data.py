"""Tests for journal chart_data() transformations."""
from __future__ import annotations

from datetime import datetime, timedelta
from types import SimpleNamespace
from unittest.mock import patch, MagicMock

import pytest


def _make_row(*, pnl: float, symbol: str = "SPY", strategy_type: str = "equity_long",
              risk: float = 100.0, exit_offset_days: int = 0, account_type: str = "paper"):
    base = datetime(2026, 3, 15, 10, 0, 0) + timedelta(days=exit_offset_days)
    row = MagicMock()
    row.realized_pnl = pnl
    row.symbol = symbol
    row.strategy_type = strategy_type
    row.risk_at_entry = risk
    row.entry_time = base - timedelta(hours=2)
    row.exit_time = base
    row.account_type = account_type
    row.status = "closed"
    return row


@pytest.fixture
def mock_journal_service():
    """Imports the service and patches DB session."""
    from journal.service import TradingJournalService
    svc = TradingJournalService.__new__(TradingJournalService)
    return svc


def test_chart_data_equity_accumulation(mock_journal_service):
    rows = [_make_row(pnl=50.0, exit_offset_days=0),
            _make_row(pnl=-20.0, exit_offset_days=1),
            _make_row(pnl=30.0, exit_offset_days=2)]
    with patch("journal.service.session_scope") as mock_ss:
        mock_db = MagicMock()
        mock_db.execute.return_value.scalars.return_value.all.return_value = rows
        mock_ss.return_value.__enter__ = lambda s: mock_db
        mock_ss.return_value.__exit__ = MagicMock(return_value=False)
        result = mock_journal_service.chart_data(account_scope="paper", limit=500)

    trades = result["trades"]
    assert len(trades) == 3
    assert trades[0]["equity"] == 50.0
    assert trades[1]["equity"] == 30.0   # 50 - 20
    assert trades[2]["equity"] == 60.0   # 30 + 30
    assert trades[0]["n"] == 1
    assert trades[2]["n"] == 3


def test_chart_data_drawdown_calculation(mock_journal_service):
    rows = [_make_row(pnl=100.0, exit_offset_days=0),
            _make_row(pnl=-40.0, exit_offset_days=1)]
    with patch("journal.service.session_scope") as mock_ss:
        mock_db = MagicMock()
        mock_db.execute.return_value.scalars.return_value.all.return_value = rows
        mock_ss.return_value.__enter__ = lambda s: mock_db
        mock_ss.return_value.__exit__ = MagicMock(return_value=False)
        result = mock_journal_service.chart_data(account_scope="paper", limit=500)

    trades = result["trades"]
    assert trades[0]["drawdown_pct"] == 0.0  # peak = 100, equity = 100
    assert trades[1]["drawdown_pct"] == -40.0  # (60-100)/100 * 100


def test_chart_data_r_multiple(mock_journal_service):
    rows = [_make_row(pnl=200.0, risk=100.0, exit_offset_days=0),
            _make_row(pnl=-50.0, risk=100.0, exit_offset_days=1)]
    with patch("journal.service.session_scope") as mock_ss:
        mock_db = MagicMock()
        mock_db.execute.return_value.scalars.return_value.all.return_value = rows
        mock_ss.return_value.__enter__ = lambda s: mock_db
        mock_ss.return_value.__exit__ = MagicMock(return_value=False)
        result = mock_journal_service.chart_data(account_scope="paper", limit=500)

    trades = result["trades"]
    assert trades[0]["r_multiple"] == 2.0
    assert trades[1]["r_multiple"] == -0.5


def test_chart_data_r_multiple_zero_risk(mock_journal_service):
    rows = [_make_row(pnl=50.0, risk=0.0, exit_offset_days=0)]
    with patch("journal.service.session_scope") as mock_ss:
        mock_db = MagicMock()
        mock_db.execute.return_value.scalars.return_value.all.return_value = rows
        mock_ss.return_value.__enter__ = lambda s: mock_db
        mock_ss.return_value.__exit__ = MagicMock(return_value=False)
        result = mock_journal_service.chart_data(account_scope="paper", limit=500)

    assert result["trades"][0]["r_multiple"] == 0.0


def test_chart_data_calendar_aggregation(mock_journal_service):
    rows = [_make_row(pnl=50.0, strategy_type="equity_long", exit_offset_days=0),
            _make_row(pnl=-10.0, strategy_type="equity_long", exit_offset_days=0),
            _make_row(pnl=30.0, strategy_type="iron_condor", exit_offset_days=1)]
    with patch("journal.service.session_scope") as mock_ss:
        mock_db = MagicMock()
        mock_db.execute.return_value.scalars.return_value.all.return_value = rows
        mock_ss.return_value.__enter__ = lambda s: mock_db
        mock_ss.return_value.__exit__ = MagicMock(return_value=False)
        result = mock_journal_service.chart_data(account_scope="paper", limit=500)

    cal = result["calendar"]
    assert len(cal) == 2
    assert cal[0]["pnl"] == 40.0  # 50 - 10
    assert cal[0]["trades"] == 2
    assert cal[0]["dominant_strategy"] == "equity_long"
    assert cal[1]["pnl"] == 30.0
    assert cal[1]["trades"] == 1
    assert cal[1]["dominant_strategy"] == "iron_condor"


def test_chart_data_empty(mock_journal_service):
    with patch("journal.service.session_scope") as mock_ss:
        mock_db = MagicMock()
        mock_db.execute.return_value.scalars.return_value.all.return_value = []
        mock_ss.return_value.__enter__ = lambda s: mock_db
        mock_ss.return_value.__exit__ = MagicMock(return_value=False)
        result = mock_journal_service.chart_data(account_scope="paper", limit=500)

    assert result["total_trades"] == 0
    assert result["trades"] == []
    assert result["calendar"] == []


def test_chart_data_win_flag(mock_journal_service):
    rows = [_make_row(pnl=10.0, exit_offset_days=0),
            _make_row(pnl=-5.0, exit_offset_days=1),
            _make_row(pnl=0.0, exit_offset_days=2)]
    with patch("journal.service.session_scope") as mock_ss:
        mock_db = MagicMock()
        mock_db.execute.return_value.scalars.return_value.all.return_value = rows
        mock_ss.return_value.__enter__ = lambda s: mock_db
        mock_ss.return_value.__exit__ = MagicMock(return_value=False)
        result = mock_journal_service.chart_data(account_scope="paper", limit=500)

    assert result["trades"][0]["win"] is True
    assert result["trades"][1]["win"] is False
    assert result["trades"][2]["win"] is False
