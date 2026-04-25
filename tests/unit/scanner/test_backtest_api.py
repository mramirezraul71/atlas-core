from __future__ import annotations

from datetime import date

from atlas_scanner.api.backtest import (
    BacktestScanner,
    scan_backtest_offline,
    scan_walk_forward_offline,
)
from atlas_scanner.backtest.engine import BacktestRequest, BacktestResult


def test_scan_backtest_offline_builds_request_and_delegates(monkeypatch) -> None:
    captured: dict[str, object] = {}
    expected = BacktestResult(
        request=BacktestRequest(start_date=date(2026, 1, 1), end_date=date(2026, 1, 1)),
        results=(),
        meta={"ok": True},
    )

    def _fake_run_backtest(request: BacktestRequest) -> BacktestResult:
        captured["request"] = request
        return expected

    monkeypatch.setattr("atlas_scanner.api.backtest.run_backtest", _fake_run_backtest)

    result = scan_backtest_offline(
        start_date=date(2026, 1, 2),
        end_date=date(2026, 1, 4),
        universe_symbols=("SPY", "QQQ"),
        score_threshold=0.75,
        scan_filters={"min_ref_price": 10},
    )

    request = captured["request"]
    assert isinstance(request, BacktestRequest)
    assert request.start_date == date(2026, 1, 2)
    assert request.end_date == date(2026, 1, 4)
    assert request.universe_symbols == ("SPY", "QQQ")
    assert request.score_threshold == 0.75
    assert request.scan_filters == {"min_ref_price": 10}
    assert result is expected


def test_scan_walk_forward_offline_builds_request_and_delegates(monkeypatch) -> None:
    captured: dict[str, object] = {}
    expected = BacktestResult(
        request=BacktestRequest(start_date=date(2026, 1, 1), end_date=date(2026, 1, 1)),
        results=(),
        meta={"wf": True},
    )

    def _fake_run_walk_forward(request: BacktestRequest, window_days: int) -> BacktestResult:
        captured["request"] = request
        captured["window_days"] = window_days
        return expected

    monkeypatch.setattr("atlas_scanner.api.backtest.run_walk_forward", _fake_run_walk_forward)

    result = scan_walk_forward_offline(
        start_date=date(2026, 1, 1),
        end_date=date(2026, 1, 10),
        window_days=5,
        universe_symbols=("SPY",),
    )

    request = captured["request"]
    assert isinstance(request, BacktestRequest)
    assert request.start_date == date(2026, 1, 1)
    assert request.end_date == date(2026, 1, 10)
    assert request.universe_symbols == ("SPY",)
    assert captured["window_days"] == 5
    assert result is expected


def test_backtest_scanner_defaults_are_applied(monkeypatch) -> None:
    captured: dict[str, object] = {}
    expected = BacktestResult(
        request=BacktestRequest(start_date=date(2026, 1, 1), end_date=date(2026, 1, 1)),
        results=(),
        meta={},
    )

    def _fake_run_backtest(request: BacktestRequest) -> BacktestResult:
        captured["request"] = request
        return expected

    monkeypatch.setattr("atlas_scanner.api.backtest.run_backtest", _fake_run_backtest)

    scanner = BacktestScanner()
    result = scanner.scan_backtest(start_date=date(2026, 2, 1), end_date=date(2026, 2, 2))

    request = captured["request"]
    assert isinstance(request, BacktestRequest)
    assert request.universe_symbols == ()
    assert request.scan_filters == {}
    assert request.score_threshold == 0.60
    assert result is expected

