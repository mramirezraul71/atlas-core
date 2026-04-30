"""Unit tests for the dashboard metrics aggregator."""

from __future__ import annotations

import os

import pytest

os.environ.setdefault("ATLAS_LLM_ENABLED", "false")


@pytest.fixture(autouse=True)
def _isolate(tmp_path, monkeypatch):
    state = tmp_path / "atlas_mode.json"
    db = tmp_path / "metrics.sqlite"
    from lotto_quant import config as _cfg
    from lotto_quant.execution import modes
    monkeypatch.setattr(_cfg, "DB_PATH", str(db), raising=False)
    monkeypatch.setattr(_cfg, "DB_FALLBACK_SQLITE", str(db), raising=False)
    monkeypatch.setattr(modes, "_STATE_FILE", state, raising=False)
    modes.reset_state_for_tests()
    yield


def _build_game():
    from lotto_quant.models.ev_calculator import PrizeTier, ScratchOffGame
    return ScratchOffGame(
        game_id="GAME-M",
        name="Metrics Game",
        ticket_price=5.0,
        total_tickets_printed=1_000_000,
        prize_tiers=[
            PrizeTier(value=100, total_prizes=1_000, remaining_prizes=200),
            PrizeTier(value=10,  total_prizes=20_000, remaining_prizes=4_000),
            PrizeTier(value=5,   total_prizes=50_000, remaining_prizes=10_000),
        ],
    )


def test_metrics_collect_empty_state():
    from lotto_quant.dashboard.metrics import MetricsService
    from lotto_quant.execution.modes import OperatingMode

    m = MetricsService().collect()
    assert m.mode == OperatingMode.PAPER
    assert m.snapshots.empty
    assert m.signals.empty
    assert m.alerts.empty
    assert m.fills.empty
    assert m.headline["games_tracked"] == 0
    assert m.headline["mode"] == "paper"


def test_metrics_collect_after_paper_fills():
    from lotto_quant.dashboard.metrics import MetricsService
    from lotto_quant.data.database import LottoQuantDB
    from lotto_quant.execution.broker import PaperBroker
    from lotto_quant.execution.modes import OperatingMode

    db = LottoQuantDB()
    broker = PaperBroker(db, rng_seed=11)
    for _ in range(3):
        broker.submit_order(game=_build_game(), n_tickets=10, expected_ev=0.0)

    m = MetricsService(db=db).collect()
    assert m.mode == OperatingMode.PAPER
    assert len(m.fills) == 3
    assert len(m.orders) == 3
    assert m.headline["n_fills"] == 3
    assert m.pnl.bankroll_start == 1_000.0  # default paper bankroll


def test_metrics_collect_with_snapshots_populates_headline():
    from lotto_quant.dashboard.metrics import MetricsService
    from lotto_quant.data.database import LottoQuantDB

    db = LottoQuantDB()
    db.insert_snapshot(
        game_id="G1", game_name="Alpha", ticket_price=5.0,
        data={}, ev_gross=0.10, ev_adjusted=0.08,
        depletion_ratio=0.7, anomaly_score=0.5,
    )
    db.insert_snapshot(
        game_id="G2", game_name="Beta", ticket_price=10.0,
        data={}, ev_gross=-0.20, ev_adjusted=-0.25,
        depletion_ratio=0.3, anomaly_score=0.1,
    )
    m = MetricsService(db=db).collect()
    assert not m.snapshots.empty
    assert not m.latest_per_game.empty
    assert m.headline["games_tracked"] == 2
    assert m.headline["ev_positive_games"] == 1


def test_metrics_filter_fills_by_mode():
    from lotto_quant.dashboard.metrics import MetricsService
    from lotto_quant.data.database import LottoQuantDB
    from lotto_quant.execution.broker import PaperBroker, LiveBroker
    from lotto_quant.execution.modes import OperatingMode, set_active_mode

    db = LottoQuantDB()
    PaperBroker(db, rng_seed=1).submit_order(
        game=_build_game(), n_tickets=5, expected_ev=0.0
    )
    LiveBroker(db).submit_order(
        game=_build_game(), n_tickets=5, expected_ev=0.0
    )

    # Paper mode → only paper fills shown
    set_active_mode(OperatingMode.PAPER)
    m_paper = MetricsService(db=db).collect()
    assert len(m_paper.fills) == 1
    assert len(m_paper.orders) == 1

    # Switch to live → fills empty (live=intent), orders=1
    set_active_mode(OperatingMode.LIVE)
    m_live = MetricsService(db=db).collect()
    assert len(m_live.fills) == 0
    assert len(m_live.orders) == 1
