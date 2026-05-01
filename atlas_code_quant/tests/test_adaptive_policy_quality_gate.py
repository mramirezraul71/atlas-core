from __future__ import annotations

from datetime import datetime, timedelta
from types import SimpleNamespace

from learning.adaptive_policy import AdaptiveLearningService


def _row(
    *,
    day: int,
    pnl: float = 10.0,
    strategy_type: str = "equity_long",
    account_type: str = "paper",
    entry_price: float = 100.0,
    exit_price: float = 101.0,
    duration_sec: float = 600.0,
):
    exit_time = datetime(2026, 3, day, 10, 0, 0)
    entry_time = exit_time - timedelta(seconds=duration_sec)
    return SimpleNamespace(
        account_type=account_type,
        strategy_type=strategy_type,
        symbol="SPY",
        realized_pnl=pnl,
        risk_at_entry=100.0,
        entry_notional=1000.0,
        entry_price=entry_price,
        exit_price=exit_price,
        entry_time=entry_time,
        exit_time=exit_time,
        updated_at=exit_time,
    )


class _FakeSession:
    def __init__(self, rows):
        self._rows = rows

    def scalars(self, _query):
        return list(self._rows)


class _FakeSessionScope:
    def __init__(self, rows):
        self._rows = rows

    def __enter__(self):
        return _FakeSession(self._rows)

    def __exit__(self, exc_type, exc, tb):
        return False


def test_adaptive_learning_blocks_when_journal_quality_is_critical(tmp_path, monkeypatch):
    rows = [_row(day=30) for _ in range(55)]
    rows.extend(_row(day=31, entry_price=-45.0) for _ in range(5))

    monkeypatch.setattr("learning.adaptive_policy.session_scope", lambda: _FakeSessionScope(rows))
    service = AdaptiveLearningService(snapshot_path=tmp_path / "adaptive_policy_snapshot.json")

    payload = service.refresh(force=True)
    status = service.status()

    assert payload["learning_allowed"] is False
    assert payload["sample_count"] == 0
    assert payload["raw_sample_count"] == 60
    assert payload["quality"]["status"] == "critical"
    assert "negative_entry_price_ratio" in payload["quality"]["blocked_reasons"]
    assert status["learning_allowed"] is False
    assert status["sample_count"] == 0
    assert status["raw_sample_count"] == 60
