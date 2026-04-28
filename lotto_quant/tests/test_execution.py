"""Unit tests for the execution layer (modes, broker, P&L)."""

from __future__ import annotations

import os
import tempfile
from pathlib import Path

import pytest

# Force SQLite (DuckDB unavailable in CI test image)
os.environ.setdefault("ATLAS_LLM_ENABLED", "false")


@pytest.fixture(autouse=True)
def _isolate_state(tmp_path, monkeypatch):
    """Each test gets its own DB + mode-state file."""
    state_path = tmp_path / "atlas_mode.json"
    db_path = tmp_path / "lotto.sqlite"

    from lotto_quant import config as _cfg
    from lotto_quant.execution import modes
    monkeypatch.setattr(_cfg, "DB_PATH", str(db_path), raising=False)
    monkeypatch.setattr(_cfg, "DB_FALLBACK_SQLITE", str(db_path), raising=False)
    monkeypatch.setattr(modes, "_STATE_FILE", state_path, raising=False)
    modes.reset_state_for_tests()
    yield


def _build_game():
    from lotto_quant.models.ev_calculator import PrizeTier, ScratchOffGame
    return ScratchOffGame(
        game_id="GAME-X",
        name="Test Game",
        ticket_price=5.0,
        total_tickets_printed=1_000_000,
        prize_tiers=[
            PrizeTier(value=10_000, total_prizes=10, remaining_prizes=5),
            PrizeTier(value=100,    total_prizes=1_000, remaining_prizes=200),
            PrizeTier(value=10,     total_prizes=20_000, remaining_prizes=4_000),
            PrizeTier(value=5,      total_prizes=50_000, remaining_prizes=10_000),
        ],
    )


# ─────────────────────────────────────────────────────────────────
# OperatingMode
# ─────────────────────────────────────────────────────────────────
def test_operating_mode_defaults_paper():
    from lotto_quant.execution.modes import get_active_mode, OperatingMode
    assert get_active_mode() == OperatingMode.PAPER


def test_set_active_mode_persists(tmp_path):
    from lotto_quant.execution.modes import (
        OperatingMode, set_active_mode, get_state, _clear_cache,
    )
    set_active_mode(OperatingMode.LIVE, live_bankroll=500.0)
    state = get_state()
    assert state.mode == OperatingMode.LIVE
    assert state.live_bankroll == 500.0

    # Re-load from disk (clear cache only — keep file)
    _clear_cache()
    s2 = get_state()
    assert s2.mode == OperatingMode.LIVE
    assert s2.live_bankroll == 500.0


def test_operating_mode_from_string_alias():
    from lotto_quant.execution.modes import OperatingMode
    assert OperatingMode.from_string("production") == OperatingMode.LIVE
    assert OperatingMode.from_string("PROD") == OperatingMode.LIVE
    assert OperatingMode.from_string("real") == OperatingMode.LIVE
    assert OperatingMode.from_string("") == OperatingMode.PAPER
    assert OperatingMode.from_string("paper") == OperatingMode.PAPER


# ─────────────────────────────────────────────────────────────────
# PaperBroker
# ─────────────────────────────────────────────────────────────────
def test_paper_broker_submit_creates_order_and_fill():
    from lotto_quant.data.database import LottoQuantDB
    from lotto_quant.execution.broker import PaperBroker
    from lotto_quant.execution.modes import OperatingMode

    db = LottoQuantDB()
    broker = PaperBroker(db, rng_seed=42)
    game = _build_game()
    order = broker.submit_order(game=game, n_tickets=10, expected_ev=0.05)

    assert order.mode == OperatingMode.PAPER
    assert order.n_tickets == 10
    assert order.cost() == pytest.approx(50.0)

    # Order persisted
    cur = db._cursor()
    cur.execute("SELECT COUNT(*) FROM exec_orders WHERE order_id = ?", (order.order_id,))
    assert cur.fetchone()[0] == 1

    # Fill persisted
    cur.execute("SELECT COUNT(*) FROM exec_fills WHERE order_id = ?", (order.order_id,))
    assert cur.fetchone()[0] == 1


def test_paper_broker_rejects_zero_tickets():
    from lotto_quant.data.database import LottoQuantDB
    from lotto_quant.execution.broker import PaperBroker
    db = LottoQuantDB()
    broker = PaperBroker(db, rng_seed=1)
    with pytest.raises(ValueError):
        broker.submit_order(game=_build_game(), n_tickets=0, expected_ev=0.0)


# ─────────────────────────────────────────────────────────────────
# LiveBroker
# ─────────────────────────────────────────────────────────────────
def test_live_broker_records_intent_only():
    from lotto_quant.data.database import LottoQuantDB
    from lotto_quant.execution.broker import LiveBroker

    db = LottoQuantDB()
    broker = LiveBroker(db)
    game = _build_game()
    order = broker.submit_order(game=game, n_tickets=4, expected_ev=0.1)

    cur = db._cursor()
    cur.execute("SELECT status FROM exec_orders WHERE order_id = ?", (order.order_id,))
    assert cur.fetchone()[0] == "OPEN"
    # No fill yet
    cur.execute("SELECT COUNT(*) FROM exec_fills WHERE order_id = ?", (order.order_id,))
    assert cur.fetchone()[0] == 0


def test_live_broker_confirm_outcome_creates_fill():
    from lotto_quant.data.database import LottoQuantDB
    from lotto_quant.execution.broker import LiveBroker

    db = LottoQuantDB()
    broker = LiveBroker(db)
    order = broker.submit_order(game=_build_game(), n_tickets=2, expected_ev=0.1)
    fill = broker.confirm_outcome(order, gross_payout=20.0)

    assert fill.gross_payout == 20.0
    assert fill.cost == pytest.approx(10.0)
    # NC tax adjustment (under federal threshold so no fed tax)
    assert fill.net_payout == pytest.approx(20.0)  # under $600 → no withholding

    cur = db._cursor()
    cur.execute("SELECT status FROM exec_orders WHERE order_id = ?", (order.order_id,))
    assert cur.fetchone()[0] == "CONFIRMED"


# ─────────────────────────────────────────────────────────────────
# PnLTracker
# ─────────────────────────────────────────────────────────────────
def test_pnl_snapshot_empty_returns_zeros():
    from lotto_quant.data.database import LottoQuantDB
    from lotto_quant.execution.modes import OperatingMode
    from lotto_quant.execution.pnl import PnLTracker
    from lotto_quant.execution.broker import ensure_broker_tables

    db = LottoQuantDB()
    ensure_broker_tables(db)
    pnl = PnLTracker(db)
    snap = pnl.snapshot(OperatingMode.PAPER, bankroll_start=1_000.0)
    assert snap.n_fills == 0
    assert snap.realized_pnl == 0.0
    assert snap.bankroll_current == 1_000.0
    assert snap.max_drawdown == 0.0
    # Equity curve always seeded with starting point
    assert len(snap.equity_curve) == 1


def test_pnl_snapshot_after_paper_fills():
    from lotto_quant.data.database import LottoQuantDB
    from lotto_quant.execution.broker import PaperBroker
    from lotto_quant.execution.modes import OperatingMode
    from lotto_quant.execution.pnl import PnLTracker

    db = LottoQuantDB()
    broker = PaperBroker(db, rng_seed=7)
    game = _build_game()
    for _ in range(5):
        broker.submit_order(game=game, n_tickets=20, expected_ev=0.0)

    pnl = PnLTracker(db)
    snap = pnl.snapshot(OperatingMode.PAPER, bankroll_start=1_000.0)
    assert snap.n_fills == 5
    assert snap.n_orders == 5
    assert snap.total_cost == pytest.approx(20 * 5 * 5.0)  # 20 tickets × $5 × 5 fills
    assert len(snap.equity_curve) == 6  # T0 + 5 fills
    assert isinstance(snap.win_rate, float)
    assert 0.0 <= snap.win_rate <= 1.0


# ──────────────────────────────────────────────────────────────
# LiveBroker helpers (HUD form)
# ──────────────────────────────────────────────────────────────
def test_live_broker_list_open_orders():
    from lotto_quant.data.database import LottoQuantDB
    from lotto_quant.execution.broker import LiveBroker

    db = LottoQuantDB()
    broker = LiveBroker(db)
    o1 = broker.submit_order(game=_build_game(), n_tickets=2, expected_ev=0.0)
    o2 = broker.submit_order(game=_build_game(), n_tickets=3, expected_ev=0.0)
    open_ = broker.list_open_orders()
    ids = {o.order_id for o in open_}
    assert o1.order_id in ids
    assert o2.order_id in ids
    assert all(o.status == "OPEN" for o in open_)

    # After confirming one, only the other remains open
    broker.confirm_outcome(o1, gross_payout=0.0)
    open_ = broker.list_open_orders()
    ids = {o.order_id for o in open_}
    assert o1.order_id not in ids
    assert o2.order_id in ids


def test_live_broker_get_order_returns_none_for_unknown():
    from lotto_quant.data.database import LottoQuantDB
    from lotto_quant.execution.broker import LiveBroker
    db = LottoQuantDB()
    broker = LiveBroker(db)
    assert broker.get_order("does-not-exist") is None


def test_live_broker_cancel_order_only_when_open():
    from lotto_quant.data.database import LottoQuantDB
    from lotto_quant.execution.broker import LiveBroker

    db = LottoQuantDB()
    broker = LiveBroker(db)
    order = broker.submit_order(game=_build_game(), n_tickets=2, expected_ev=0.0)
    # First cancel succeeds
    assert broker.cancel_order(order.order_id) is True
    # Second cancel fails (status changed)
    assert broker.cancel_order(order.order_id) is False

    # No fill was created for the cancelled order
    cur = db._cursor()
    cur.execute("SELECT COUNT(*) FROM exec_fills WHERE order_id = ?", (order.order_id,))
    assert cur.fetchone()[0] == 0


def test_pnl_filters_by_mode():
    from lotto_quant.data.database import LottoQuantDB
    from lotto_quant.execution.broker import PaperBroker, LiveBroker
    from lotto_quant.execution.modes import OperatingMode
    from lotto_quant.execution.pnl import PnLTracker

    db = LottoQuantDB()
    paper = PaperBroker(db, rng_seed=3)
    paper.submit_order(game=_build_game(), n_tickets=5, expected_ev=0.0)

    live = LiveBroker(db)
    live.submit_order(game=_build_game(), n_tickets=5, expected_ev=0.0)

    pnl = PnLTracker(db)
    snap_paper = pnl.snapshot(OperatingMode.PAPER, bankroll_start=1_000.0)
    snap_live = pnl.snapshot(OperatingMode.LIVE, bankroll_start=1_000.0)
    assert snap_paper.n_fills == 1
    assert snap_live.n_fills == 0  # live = intent only
    assert snap_live.n_orders == 1
