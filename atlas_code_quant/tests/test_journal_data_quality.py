from __future__ import annotations

from datetime import datetime, timedelta
from types import SimpleNamespace

from learning.journal_data_quality import build_journal_quality_scorecard, filter_closed_entries


def _row(
    *,
    day: int,
    pnl: float = 10.0,
    strategy_type: str = "equity_long",
    entry_price: float = 100.0,
    exit_price: float = 101.0,
    duration_sec: float = 600.0,
) -> SimpleNamespace:
    exit_time = datetime(2026, 3, day, 10, 0, 0)
    entry_time = exit_time - timedelta(seconds=duration_sec)
    return SimpleNamespace(
        entry_price=entry_price,
        exit_price=exit_price,
        realized_pnl=pnl,
        strategy_type=strategy_type,
        symbol="SPY",
        entry_time=entry_time,
        exit_time=exit_time,
        updated_at=exit_time,
    )


def test_build_journal_quality_scorecard_blocks_contaminated_dataset():
    rows = [_row(day=30) for _ in range(55)]
    rows.extend(_row(day=29, entry_price=-50.0) for _ in range(5))

    scorecard = build_journal_quality_scorecard(rows)

    assert scorecard["status"] == "critical"
    assert scorecard["learning_allowed"] is False
    assert "negative_entry_price_ratio" in scorecard["blocked_reasons"]
    assert "outlier_close_day_concentration" in scorecard["blocked_reasons"]
    assert scorecard["metrics"]["negative_entry_price_count"] == 5
    assert scorecard["outlier_days"][0]["date"] == "2026-03-30"


def test_filter_closed_entries_excludes_invalid_rows_without_outlier_day_filter():
    good = _row(day=30, pnl=25.0)
    negative_price = _row(day=30, entry_price=-10.0)
    too_short = _row(day=31, duration_sec=3.0)

    filtered = filter_closed_entries([good, negative_price, too_short], include_outlier_days=False)

    assert filtered == [good]


def test_filter_closed_entries_can_exclude_outlier_days():
    rows = [_row(day=30, pnl=10.0) for _ in range(55)]
    rows.extend(_row(day=31, pnl=12.0) for _ in range(5))
    scorecard = build_journal_quality_scorecard(rows)

    filtered = filter_closed_entries(rows, scorecard=scorecard, include_outlier_days=True)

    assert len(filtered) == 5
    assert all(row.exit_time.date().isoformat() == "2026-03-31" for row in filtered)
