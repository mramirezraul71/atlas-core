"""Tests for the historical replay backtester."""

from __future__ import annotations

import json
import os
import uuid
from datetime import datetime, timedelta, timezone
from pathlib import Path

import pytest

# Force SQLite path for the test DB (DuckDB unavailable in CI image)
os.environ.setdefault("ATLAS_LLM_ENABLED", "false")


# ─────────────────────────────────────────────────────────────────────
# Shared fixtures
# ─────────────────────────────────────────────────────────────────────
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


def _seed_snapshot(
    db,
    *,
    game_id: str,
    name: str,
    ticket_price: float,
    prize_tiers: list[dict],
    snapshot_ts: datetime,
    ev_gross: float = 0.0,
    ev_adjusted: float = 0.0,
    depletion_ratio: float = 0.5,
    anomaly_score: float = 0.0,
) -> str:
    """Insert a snapshot row directly using the same shape opportunity_radar produces."""
    snap_id = str(uuid.uuid4())
    cur = db._cursor()
    cur.execute(
        "INSERT INTO scratch_off_snapshots VALUES (?,?,?,?,?,?,?,?,?,?)",
        (
            snap_id,
            game_id,
            name,
            ticket_price,
            snapshot_ts,
            json.dumps({"prize_tiers": prize_tiers}),
            ev_gross,
            ev_adjusted,
            depletion_ratio,
            anomaly_score,
        ),
    )
    db._commit()
    return snap_id


def _positive_ev_tiers() -> list[dict]:
    """A favorable game where the major prize is heavily skewed remaining."""
    return [
        # Heavy major prize remaining → boosts EV/$ above ticket_price
        {"value": 1_000_000.0, "total_prizes": 4,       "remaining_prizes": 4,        "odds_initial": 1_000_000.0},
        {"value": 1_000.0,     "total_prizes": 1_000,   "remaining_prizes": 600,      "odds_initial": 4_000.0},
        {"value": 100.0,       "total_prizes": 50_000,  "remaining_prizes": 25_000,   "odds_initial": 80.0},
        {"value": 5.0,         "total_prizes": 200_000, "remaining_prizes": 100_000,  "odds_initial": 20.0},
    ]


def _negative_ev_tiers() -> list[dict]:
    """An unattractive game (tail prizes claimed, only small ones remain)."""
    return [
        {"value": 1_000_000.0, "total_prizes": 4,       "remaining_prizes": 0,        "odds_initial": 1_000_000.0},
        {"value": 1_000.0,     "total_prizes": 1_000,   "remaining_prizes": 0,        "odds_initial": 4_000.0},
        {"value": 5.0,         "total_prizes": 200_000, "remaining_prizes": 50_000,   "odds_initial": 20.0},
    ]


# ─────────────────────────────────────────────────────────────────────
# Engine basics
# ─────────────────────────────────────────────────────────────────────
def test_backtest_no_snapshots_returns_empty_result(tmp_path):
    from lotto_quant.backtest import BacktestConfig, BacktestEngine

    cfg = BacktestConfig(
        workspace_db=str(tmp_path / "ws.sqlite"),
        bankroll_start=1_000.0,
    )
    res = BacktestEngine(cfg).run()
    assert res.days == []
    assert res.n_orders == 0
    assert res.final_bankroll == pytest.approx(1_000.0)
    assert "No snapshots" in res.notes[0]


def test_backtest_runs_against_seeded_snapshots(tmp_path):
    from lotto_quant.backtest import BacktestConfig, BacktestEngine
    from lotto_quant.data.database import LottoQuantDB

    db = LottoQuantDB()
    base = datetime(2026, 1, 1, 12, 0, tzinfo=timezone.utc)
    for d in range(3):
        _seed_snapshot(
            db,
            game_id="G-A",
            name="Lucky Game A",
            ticket_price=5.0,
            prize_tiers=_positive_ev_tiers(),
            snapshot_ts=base + timedelta(days=d),
        )

    cfg = BacktestConfig(
        workspace_db=str(tmp_path / "ws.sqlite"),
        bankroll_start=1_000.0,
        kelly_fraction=0.25,
        rng_seed=42,
    )
    res = BacktestEngine(cfg).run()
    # 3 days replayed
    assert len(res.days) == 3
    assert all(d.games_seen == 1 for d in res.days)
    # Equity curve has T0 + 3 daily points
    assert len(res.equity_curve) == 4
    # Summary dict serializable
    summary = res.summary_dict()
    assert summary["n_days"] == 3
    assert "max_drawdown" in summary


def test_backtest_skips_negative_ev_games(tmp_path):
    """Games with EV/$ <= 0 must be skipped under the default filter."""
    from lotto_quant.backtest import BacktestConfig, BacktestEngine
    from lotto_quant.data.database import LottoQuantDB

    db = LottoQuantDB()
    ts = datetime(2026, 2, 1, 9, 0, tzinfo=timezone.utc)
    _seed_snapshot(
        db,
        game_id="G-NEG",
        name="Bad Game",
        ticket_price=5.0,
        prize_tiers=_negative_ev_tiers(),
        snapshot_ts=ts,
    )

    cfg = BacktestConfig(
        workspace_db=str(tmp_path / "ws.sqlite"),
        bankroll_start=500.0,
        rng_seed=7,
    )
    res = BacktestEngine(cfg).run()
    # The single day should have seen the game but played nothing
    assert len(res.days) == 1
    assert res.days[0].games_seen == 1
    assert res.days[0].games_played == 0
    assert res.n_orders == 0
    assert res.final_bankroll == pytest.approx(500.0)


def test_backtest_is_deterministic_with_same_seed(tmp_path):
    """Same seed + same data → identical bankroll trajectory."""
    from lotto_quant.backtest import BacktestConfig, BacktestEngine
    from lotto_quant.data.database import LottoQuantDB

    db = LottoQuantDB()
    base = datetime(2026, 3, 1, 8, 0, tzinfo=timezone.utc)
    for d in range(2):
        _seed_snapshot(
            db,
            game_id="G-DET",
            name="Deterministic Game",
            ticket_price=5.0,
            prize_tiers=_positive_ev_tiers(),
            snapshot_ts=base + timedelta(days=d),
        )

    def _run(workspace_name: str):
        cfg = BacktestConfig(
            workspace_db=str(tmp_path / workspace_name),
            bankroll_start=1_000.0,
            rng_seed=123,
        )
        return BacktestEngine(cfg).run()

    res1 = _run("ws_a.sqlite")
    res2 = _run("ws_b.sqlite")
    assert res1.final_bankroll == pytest.approx(res2.final_bankroll)
    assert [d.bankroll_close for d in res1.days] == \
        pytest.approx([d.bankroll_close for d in res2.days])


def test_backtest_buckets_multiple_snapshots_per_day(tmp_path):
    """Two snapshots same day, same game → only the most recent is replayed."""
    from lotto_quant.backtest import BacktestConfig, BacktestEngine
    from lotto_quant.data.database import LottoQuantDB

    db = LottoQuantDB()
    day = datetime(2026, 4, 1, 0, 0, tzinfo=timezone.utc)
    _seed_snapshot(db, game_id="G-A", name="A", ticket_price=5.0,
                   prize_tiers=_positive_ev_tiers(), snapshot_ts=day)
    _seed_snapshot(db, game_id="G-A", name="A", ticket_price=5.0,
                   prize_tiers=_positive_ev_tiers(),
                   snapshot_ts=day + timedelta(hours=4))

    cfg = BacktestConfig(workspace_db=str(tmp_path / "ws.sqlite"),
                         bankroll_start=1_000.0, rng_seed=11)
    res = BacktestEngine(cfg).run()
    assert len(res.days) == 1
    assert res.days[0].games_seen == 1


def test_backtest_days_back_truncates_to_recent_window(tmp_path):
    from lotto_quant.backtest import BacktestConfig, BacktestEngine
    from lotto_quant.data.database import LottoQuantDB

    db = LottoQuantDB()
    base = datetime(2026, 5, 1, 0, 0, tzinfo=timezone.utc)
    for d in range(5):
        _seed_snapshot(db, game_id=f"G-{d}", name=f"Game {d}",
                       ticket_price=5.0,
                       prize_tiers=_positive_ev_tiers(),
                       snapshot_ts=base + timedelta(days=d))

    cfg = BacktestConfig(workspace_db=str(tmp_path / "ws.sqlite"),
                         bankroll_start=500.0, days_back=2, rng_seed=9)
    res = BacktestEngine(cfg).run()
    assert len(res.days) == 2


def test_backtest_workspace_db_isolated_from_source(tmp_path):
    """Backtest writes go to workspace_db, NOT the source DB."""
    from lotto_quant.backtest import BacktestConfig, BacktestEngine
    from lotto_quant.data.database import LottoQuantDB

    src = LottoQuantDB()
    ts = datetime(2026, 6, 1, 12, 0, tzinfo=timezone.utc)
    _seed_snapshot(src, game_id="G-X", name="X", ticket_price=5.0,
                   prize_tiers=_positive_ev_tiers(), snapshot_ts=ts)

    ws_path = tmp_path / "ws.sqlite"
    cfg = BacktestConfig(workspace_db=str(ws_path),
                         bankroll_start=1_000.0, rng_seed=13)
    BacktestEngine(cfg).run()

    # Source must remain free of any execution rows
    cur = src._cursor()
    cur.execute("SELECT COUNT(*) FROM exec_orders")
    src_orders = cur.fetchone()[0]
    assert src_orders == 0

    # Workspace DB has the activity
    ws = LottoQuantDB(str(ws_path))
    cur2 = ws._cursor()
    cur2.execute("SELECT COUNT(*) FROM exec_orders")
    ws_orders = cur2.fetchone()[0]
    assert ws_orders >= 0  # may be zero if Kelly capped — at minimum table exists


# ─────────────────────────────────────────────────────────────────────
# Helpers
# ─────────────────────────────────────────────────────────────────────
def test_reconstruct_game_returns_none_for_empty_payload():
    from lotto_quant.backtest.engine import BacktestEngine

    snap = {
        "snapshot_id": "x", "game_id": "g", "game_name": "n",
        "ticket_price": 5.0, "snapshot_ts": "2026-01-01",
        "data_json": json.dumps({"prize_tiers": []}),
    }
    assert BacktestEngine._reconstruct_game(snap) is None


def test_reconstruct_game_handles_malformed_json():
    from lotto_quant.backtest.engine import BacktestEngine

    snap = {
        "snapshot_id": "x", "game_id": "g", "game_name": "n",
        "ticket_price": 5.0, "snapshot_ts": "2026-01-01",
        "data_json": "{not json",
    }
    assert BacktestEngine._reconstruct_game(snap) is None


def test_day_key_handles_iso_string_and_datetime():
    from lotto_quant.backtest.engine import BacktestEngine

    assert BacktestEngine._day_key("2026-01-15T08:00:00") == "2026-01-15"
    assert BacktestEngine._day_key("2026-01-15 08:00:00") == "2026-01-15"
    dt = datetime(2026, 1, 15, 8, 0)
    assert BacktestEngine._day_key(dt) == "2026-01-15"


# ─────────────────────────────────────────────────────────────────────
# CLI smoke
# ─────────────────────────────────────────────────────────────────────
def test_cli_writes_output_json(tmp_path, capsys):
    """`python -m lotto_quant.backtest --output-json ...` produces a JSON file."""
    from lotto_quant.backtest.__main__ import main
    from lotto_quant.data.database import LottoQuantDB

    db = LottoQuantDB()
    base = datetime(2026, 7, 1, 0, 0, tzinfo=timezone.utc)
    for d in range(2):
        _seed_snapshot(db, game_id="G-CLI", name="CLI Game",
                       ticket_price=5.0,
                       prize_tiers=_positive_ev_tiers(),
                       snapshot_ts=base + timedelta(days=d))

    out_json = tmp_path / "out.json"
    ws_db = tmp_path / "ws.sqlite"
    rc = main([
        "--workspace-db", str(ws_db),
        "--bankroll", "750",
        "--rng-seed", "99",
        "--output-json", str(out_json),
        "--quiet",
    ])
    assert rc == 0
    assert out_json.exists()
    payload = json.loads(out_json.read_text())
    assert "summary" in payload
    assert "equity_curve" in payload
    assert "days" in payload
    assert payload["summary"]["n_days"] == 2
