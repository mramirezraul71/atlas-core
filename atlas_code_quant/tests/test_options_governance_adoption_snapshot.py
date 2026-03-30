from __future__ import annotations

from datetime import datetime, timedelta
from types import SimpleNamespace

from atlas_code_quant.journal.service import build_options_governance_adoption_snapshot


def _entry(*, strategy_type: str, status: str, symbol: str, updated_offset_min: int, account_type: str = "paper") -> SimpleNamespace:
    now = datetime(2026, 3, 29, 11, 0, 0)
    return SimpleNamespace(
        status=status,
        symbol=symbol,
        strategy_type=strategy_type,
        account_type=account_type,
        updated_at=now - timedelta(minutes=updated_offset_min),
        entry_time=now - timedelta(hours=4),
    )


def test_options_governance_adoption_snapshot_warns_when_not_exercised() -> None:
    entries = [
        _entry(strategy_type="equity_long", status="open", symbol="AAPL", updated_offset_min=5),
        _entry(strategy_type="equity_short", status="closed", symbol="MSFT", updated_offset_min=15),
    ]

    snapshot = build_options_governance_adoption_snapshot(entries, account_type="paper", limit=5)

    assert snapshot["summary"]["option_entries_total"] == 0
    assert snapshot["alerts"]
    assert snapshot["alerts"][0]["code"] == "options_governance_not_exercised"


def test_options_governance_adoption_snapshot_classifies_family_mix_and_vertical_concentration() -> None:
    entries = [
        _entry(strategy_type="bull_call_debit_spread", status="closed", symbol="AAPL", updated_offset_min=5),
        _entry(strategy_type="bear_put_debit_spread", status="closed", symbol="NVDA", updated_offset_min=10),
        _entry(strategy_type="bull_put_credit_spread", status="open", symbol="AMD", updated_offset_min=15),
        _entry(strategy_type="bear_call_credit_spread", status="closed", symbol="TSLA", updated_offset_min=20),
        _entry(strategy_type="bull_call_debit_spread", status="open", symbol="QQQ", updated_offset_min=25),
        _entry(strategy_type="call_calendar_spread", status="open", symbol="SPY", updated_offset_min=30),
    ]

    snapshot = build_options_governance_adoption_snapshot(entries, account_type="paper", limit=10)

    assert snapshot["summary"]["option_entries_total"] == 6
    assert snapshot["summary"]["open_option_positions"] == 3
    assert snapshot["summary"]["closed_option_trades"] == 3
    assert snapshot["summary"]["vertical_count"] == 5
    assert snapshot["summary"]["time_spread_count"] == 1
    assert snapshot["summary"]["strategy_diversity_count"] >= 4
    assert snapshot["summary"]["vertical_share_pct"] == 83.33
    assert snapshot["family_mix"][0]["family"] == "directional_vertical"
    assert any(alert["code"] == "options_mix_concentrated_in_verticals" for alert in snapshot["alerts"])
