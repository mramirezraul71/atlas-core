"""Tests para WeeklyReporter — Fase 0 Semana 4."""
from __future__ import annotations

import json
import sqlite3
from pathlib import Path
from unittest.mock import MagicMock, patch

import pytest

from atlas_code_quant.reports.weekly_report import WeeklyReporter, WeeklyStats


# ── Fixtures ──────────────────────────────────────────────────────────────────

def _make_journal(path: Path, trades: list[dict]) -> None:
    with sqlite3.connect(str(path)) as conn:
        conn.execute("""
            CREATE TABLE trading_journal (
                id INTEGER PRIMARY KEY,
                symbol TEXT,
                strategy_type TEXT,
                entry_price REAL,
                exit_price REAL,
                realized_pnl REAL,
                status TEXT,
                entry_time TEXT
            )
        """)
        for t in trades:
            conn.execute(
                "INSERT INTO trading_journal (symbol, strategy_type, entry_price, exit_price, realized_pnl, status, entry_time) VALUES (?,?,?,?,?,?,?)",
                ("SPY", "bull_put", 100.0, 101.0, t["pnl"], t.get("status", "closed"), t["time"]),
            )
        conn.commit()


@pytest.fixture()
def state_file(tmp_path: Path) -> Path:
    state = {
        "auton_mode": "paper_autonomous",
        "fail_safe_active": False,
        "operational_error_count": 0,
        "signal_validator_enabled": True,
        "var_monitor_enabled": True,
        "kelly_dynamic_enabled": False,
        "sentiment_score": 0.15,
        "sentiment_source": "vader:reddit_wallstreetbets",
    }
    p = tmp_path / "state.json"
    p.write_text(json.dumps(state))
    return p


@pytest.fixture()
def reporter_with_trades(tmp_path: Path, state_file: Path) -> WeeklyReporter:
    from datetime import datetime, timedelta
    today = datetime.now()
    monday = today - timedelta(days=today.weekday())

    trades = [
        {"pnl": 120.0, "time": monday.strftime("%Y-%m-%d") + " 10:00:00"},
        {"pnl": -40.0, "time": monday.strftime("%Y-%m-%d") + " 11:00:00"},
        {"pnl": 80.0,  "time": monday.strftime("%Y-%m-%d") + " 14:00:00"},
        {"pnl": 60.0,  "time": monday.strftime("%Y-%m-%d") + " 15:00:00"},
        {"pnl": -25.0, "time": monday.strftime("%Y-%m-%d") + " 15:30:00"},
    ]
    db = tmp_path / "journal.sqlite3"
    _make_journal(db, trades)
    return WeeklyReporter(
        db_path=db,
        state_path=state_file,
        bot_token="fake_token",
        chat_id="12345",
    )


# ── Tests WeeklyStats ─────────────────────────────────────────────────────────

def test_stats_win_rate(reporter_with_trades: WeeklyReporter):
    stats = reporter_with_trades.compute_stats()
    assert stats.total_trades == 5
    assert stats.wins == 3
    assert stats.losses == 2
    assert stats.win_rate == pytest.approx(0.6, abs=0.01)


def test_stats_pnl(reporter_with_trades: WeeklyReporter):
    stats = reporter_with_trades.compute_stats()
    assert stats.total_pnl == pytest.approx(195.0, abs=0.01)


def test_stats_profit_factor(reporter_with_trades: WeeklyReporter):
    stats = reporter_with_trades.compute_stats()
    # gross_wins = 120+80+60=260, gross_losses = 40+25=65
    assert stats.profit_factor == pytest.approx(260.0 / 65.0, abs=0.01)


def test_stats_best_worst(reporter_with_trades: WeeklyReporter):
    stats = reporter_with_trades.compute_stats()
    assert stats.best_trade == 120.0
    assert stats.worst_trade == -40.0


def test_stats_no_trades(tmp_path: Path, state_file: Path):
    db = tmp_path / "journal.sqlite3"
    _make_journal(db, [])
    reporter = WeeklyReporter(db_path=db, state_path=state_file)
    stats = reporter.compute_stats()
    assert stats.total_trades == 0
    assert stats.win_rate == 0.0
    assert stats.profit_factor == 0.0


def test_stats_missing_db(tmp_path: Path, state_file: Path):
    reporter = WeeklyReporter(
        db_path=tmp_path / "nonexistent.db",
        state_path=state_file,
    )
    stats = reporter.compute_stats()
    assert stats.total_trades == 0


# ── Tests mensaje ─────────────────────────────────────────────────────────────

def test_message_contains_win_rate(reporter_with_trades: WeeklyReporter):
    stats = reporter_with_trades.compute_stats()
    state = reporter_with_trades._load_state()
    msg = reporter_with_trades._build_message(stats, state)
    assert "Win Rate" in msg
    assert "60.0%" in msg


def test_message_contains_fase0_kpis(reporter_with_trades: WeeklyReporter):
    stats = reporter_with_trades.compute_stats()
    state = reporter_with_trades._load_state()
    msg = reporter_with_trades._build_message(stats, state)
    assert "Fase 0" in msg
    assert "meta" in msg.lower() or "Meta" in msg


def test_message_contains_module_status(reporter_with_trades: WeeklyReporter):
    stats = reporter_with_trades.compute_stats()
    state = reporter_with_trades._load_state()
    msg = reporter_with_trades._build_message(stats, state)
    assert "Signal Validator" in msg
    assert "VaR Monitor" in msg
    assert "Kelly" in msg
    assert "Sentiment" in msg


# ── Tests envío Telegram ──────────────────────────────────────────────────────

def test_send_calls_telegram_api(reporter_with_trades: WeeklyReporter):
    """send() debe llamar a la API de Telegram."""
    mock_response = MagicMock()
    mock_response.read.return_value = json.dumps({"ok": True}).encode()
    mock_response.__enter__ = lambda s: s
    mock_response.__exit__ = MagicMock(return_value=False)

    with patch("urllib.request.urlopen", return_value=mock_response):
        result = reporter_with_trades.send()

    assert result is True


def test_send_returns_false_without_credentials(tmp_path: Path, state_file: Path):
    """Sin token, send() debe devolver False sin lanzar excepción."""
    db = tmp_path / "journal.sqlite3"
    _make_journal(db, [])
    reporter = WeeklyReporter(
        db_path=db, state_path=state_file,
        bot_token="", chat_id="",
    )
    result = reporter.send()
    assert result is False


def test_send_handles_network_error(reporter_with_trades: WeeklyReporter):
    """send() debe devolver False si hay error de red, sin lanzar excepción."""
    with patch("urllib.request.urlopen", side_effect=Exception("Connection refused")):
        result = reporter_with_trades.send()
    assert result is False
