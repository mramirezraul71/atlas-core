from __future__ import annotations

from atlas_code_quant.api import main


def test_rebased_recent_equity_curve_anchors_to_current_realized_equity() -> None:
    chart_trades = [
        {"exit_time": "2026-04-10T14:30:00Z", "pnl": 50.0, "equity": 50.0},
        {"exit_time": "2026-04-10T14:31:00Z", "pnl": -20.0, "equity": 30.0},
        {"exit_time": "2026-04-10T14:32:00Z", "pnl": 30.0, "equity": 60.0},
    ]
    canonical_balances = {"total_equity": 96331.03}
    canonical_totals = {"open_pnl": 2227.62}

    curve = main._rebased_recent_equity_curve(
        chart_trades,
        canonical_balances=canonical_balances,
        canonical_totals=canonical_totals,
    )

    assert len(curve) == 3
    assert curve[0]["value"] == 94093.41
    assert curve[-1]["value"] == 94103.41


def test_drawdown_curve_from_equity_uses_rebased_series() -> None:
    equity_curve = [
        {"time": 1, "value": 100000.0},
        {"time": 2, "value": 99000.0},
        {"time": 3, "value": 101000.0},
        {"time": 4, "value": 100500.0},
    ]

    drawdown_curve = main._drawdown_curve_from_equity(equity_curve)

    assert drawdown_curve == [
        {"time": 1, "value": 0.0},
        {"time": 2, "value": -1.0},
        {"time": 3, "value": 0.0},
        {"time": 4, "value": -0.495},
    ]
