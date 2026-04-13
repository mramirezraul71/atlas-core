"""Tests para KellyDynamic — Fase 0 Semana 3."""
from __future__ import annotations

import sqlite3
import tempfile
from pathlib import Path
from unittest.mock import patch

import pytest

from atlas_code_quant.risk.kelly_dynamic import (
    KellyDynamic,
    KellyDynamicResult,
    _DEFENSIVE_MODE_THRESHOLD,
    _DRAWDOWN_HALT_THRESHOLD,
    _DRAWDOWN_REDUCE_THRESHOLD,
)


# ── Fixtures ──────────────────────────────────────────────────────────────────

def _create_journal(path: Path, trades: list[dict]) -> None:
    """Crea un journal SQLite mínimo con trades de prueba."""
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
                (
                    t.get("symbol", "SPY"),
                    t.get("strategy", "bull_put_credit_spread"),
                    t.get("entry_price", 100.0),
                    t.get("exit_price", 101.0),
                    t.get("pnl", 50.0),
                    t.get("status", "closed"),
                    t.get("time", "2026-04-10 10:00:00"),
                ),
            )
        conn.commit()


@pytest.fixture()
def db_with_wins(tmp_path: Path) -> Path:
    """Journal con mayoría de trades ganadores."""
    trades = [
        {"pnl": 100.0, "time": "2026-03-01 10:00:00"},
        {"pnl": 80.0,  "time": "2026-03-05 10:00:00"},
        {"pnl": -30.0, "time": "2026-03-10 10:00:00"},
        {"pnl": 120.0, "time": "2026-03-15 10:00:00"},
        {"pnl": 60.0,  "time": "2026-03-20 10:00:00"},
        {"pnl": -20.0, "time": "2026-03-25 10:00:00"},
        {"pnl": 90.0,  "time": "2026-04-01 10:00:00"},
        {"pnl": 50.0,  "time": "2026-04-05 10:00:00"},
    ]
    p = tmp_path / "journal.sqlite3"
    _create_journal(p, trades)
    return p


@pytest.fixture()
def db_with_consecutive_losses(tmp_path: Path) -> Path:
    """Journal con 3 pérdidas consecutivas recientes → modo defensivo."""
    trades = [
        {"pnl": 100.0, "time": "2026-03-01 10:00:00"},
        {"pnl": 80.0,  "time": "2026-03-05 10:00:00"},
        {"pnl": 60.0,  "time": "2026-03-10 10:00:00"},
        {"pnl": -50.0, "time": "2026-04-08 10:00:00"},
        {"pnl": -60.0, "time": "2026-04-09 10:00:00"},
        {"pnl": -40.0, "time": "2026-04-10 10:00:00"},
    ]
    p = tmp_path / "journal.sqlite3"
    _create_journal(p, trades)
    return p


@pytest.fixture()
def db_empty(tmp_path: Path) -> Path:
    """Journal vacío."""
    p = tmp_path / "journal.sqlite3"
    _create_journal(p, [])
    return p


# ── Tests básicos ─────────────────────────────────────────────────────────────

def test_recommend_size_returns_result(db_with_wins: Path):
    kd = KellyDynamic(db_path=db_with_wins)
    result = kd.recommend_size("SPY", "bull_put_credit_spread", capital=50000)
    assert isinstance(result, KellyDynamicResult)
    assert result.symbol == "SPY"
    assert result.strategy == "bull_put_credit_spread"


def test_recommended_pct_in_range(db_with_wins: Path):
    kd = KellyDynamic(db_path=db_with_wins)
    result = kd.recommend_size("SPY", "bull_put_credit_spread", capital=50000)
    assert 0.0 <= result.recommended_pct <= 1.0


def test_recommended_usd_coherent(db_with_wins: Path):
    capital = 50000.0
    kd = KellyDynamic(db_path=db_with_wins)
    result = kd.recommend_size("SPY", "bull_put_credit_spread", capital=capital)
    assert result.recommended_usd == pytest.approx(capital * result.recommended_pct, abs=1.0)


# ── Tests circuit breakers ────────────────────────────────────────────────────

def test_defensive_mode_on_consecutive_losses(db_with_consecutive_losses: Path):
    kd = KellyDynamic(db_path=db_with_consecutive_losses)
    result = kd.recommend_size("SPY", "bull_put_credit_spread", capital=50000)
    assert result.circuit_breaker == "defensive", (
        f"Esperaba 'defensive' con {result.consecutive_losses} pérdidas consecutivas"
    )
    assert result.consecutive_losses >= _DEFENSIVE_MODE_THRESHOLD


def test_defensive_mode_halves_position(db_with_consecutive_losses: Path):
    """En modo defensivo, el tamaño debe ser la mitad del normal."""
    kd = KellyDynamic(db_path=db_with_consecutive_losses)
    result = kd.recommend_size("SPY", "bull_put_credit_spread", capital=50000)
    # Kelly base no-defensivo sería el doble
    base_pct = result.kelly.position_pct
    if result.circuit_breaker == "defensive":
        assert result.recommended_pct == pytest.approx(base_pct * 0.5, abs=0.001)


def test_circuit_breaker_halted_on_large_drawdown(tmp_path: Path):
    """Drawdown > 5% del capital activa HALT total."""
    capital = 10000.0
    # Pérdidas recientes que suman > 5% del capital
    trades = [
        {"pnl": -300.0, "time": "2026-04-08 10:00:00"},
        {"pnl": -250.0, "time": "2026-04-09 10:00:00"},
        {"pnl": -200.0, "time": "2026-04-10 10:00:00"},
    ]
    p = tmp_path / "journal.sqlite3"
    _create_journal(p, trades)
    kd = KellyDynamic(db_path=p)
    result = kd.recommend_size("SPY", "bull_put_credit_spread", capital=capital)
    assert result.circuit_breaker == "halted"
    assert result.recommended_pct == 0.0
    assert result.recommended_usd == 0.0


# ── Tests con journal vacío o sin historial ───────────────────────────────────

def test_empty_journal_no_crash(db_empty: Path):
    kd = KellyDynamic(db_path=db_empty)
    result = kd.recommend_size("SPY", "bull_put_credit_spread", capital=50000)
    assert result.circuit_breaker == "none"
    assert result.recommended_pct >= 0.0


def test_missing_db_no_crash(tmp_path: Path):
    kd = KellyDynamic(db_path=tmp_path / "nonexistent.db")
    result = kd.recommend_size("SPY", "bull_put_credit_spread", capital=50000)
    assert isinstance(result, KellyDynamicResult)
    assert result.recommended_pct >= 0.0


def test_insufficient_samples_uses_conservative_default(db_empty: Path):
    """Con menos de min_samples, debe usar default conservador."""
    kd = KellyDynamic(db_path=db_empty)
    result = kd.recommend_size("SPY", "bull_put_credit_spread", capital=50000)
    # Con 0 trades, Kelly no puede calcular — debe usar default
    assert result.kelly.samples == 0
    assert result.recommended_pct <= 0.20  # nunca más del cap máximo


# ── Tests portfolio snapshot ──────────────────────────────────────────────────

def test_portfolio_snapshot(db_with_wins: Path):
    kd = KellyDynamic(db_path=db_with_wins)
    snap = kd.get_portfolio_snapshot(capital=50000)
    assert snap["kelly_dynamic_enabled"] is True
    assert snap["capital"] == 50000
    assert "active_strategies" in snap
    assert "timestamp" in snap


def test_to_dict_serializable(db_with_wins: Path):
    """El resultado debe ser serializable a JSON."""
    import json
    kd = KellyDynamic(db_path=db_with_wins)
    result = kd.recommend_size("SPY", "bull_put_credit_spread", capital=50000)
    d = result.to_dict()
    # No debe lanzar excepción
    json.dumps(d)
