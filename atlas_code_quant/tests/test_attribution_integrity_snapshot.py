from __future__ import annotations

from datetime import datetime, timedelta

from atlas_code_quant.journal.models import TradingJournal
from atlas_code_quant.journal.service import build_attribution_integrity_snapshot


def _entry(*, strategy_type: str, status: str, symbol: str, updated_offset_min: int) -> TradingJournal:
    now = datetime.utcnow()
    return TradingJournal(
        journal_key=f"paper:test:{symbol}:{strategy_type}:{status}",
        account_type="paper",
        account_id="paper-test",
        strategy_id=f"{strategy_type}:{symbol}",
        tracker_strategy_id=f"{strategy_type}:{symbol}:tracker",
        strategy_type=strategy_type,
        symbol=symbol,
        legs_signature=f"{symbol}:{strategy_type}",
        legs_details="[]",
        thesis_rich_text="",
        status=status,
        entry_time=now - timedelta(hours=2),
        updated_at=now - timedelta(minutes=updated_offset_min),
        last_synced_at=now - timedelta(minutes=updated_offset_min),
    )


def test_attribution_integrity_snapshot_flags_untracked_open_positions() -> None:
    payload = build_attribution_integrity_snapshot(
        [
            _entry(strategy_type="equity_long", status="open", symbol="AAPL", updated_offset_min=20),
            _entry(strategy_type="untracked", status="open", symbol="TSLA", updated_offset_min=10),
            _entry(strategy_type="unknown", status="closed", symbol="MSFT", updated_offset_min=5),
        ],
        account_type="paper",
        limit=5,
    )

    assert payload["summary"]["open_positions"] == 2
    assert payload["summary"]["open_untracked_count"] == 1
    assert payload["summary"]["recent_flagged_count"] == 2
    assert payload["summary"]["attributed_open_positions_pct"] == 50.0
    assert payload["alerts"][0]["code"] == "open_positions_unattributed"
    assert payload["flagged_entries"][0]["strategy_type"] in {"unknown", "untracked"}


def test_attribution_integrity_snapshot_is_clean_when_all_entries_are_attributed() -> None:
    payload = build_attribution_integrity_snapshot(
        [
            _entry(strategy_type="equity_long", status="open", symbol="AAPL", updated_offset_min=10),
            _entry(strategy_type="equity_short", status="closed", symbol="TSLA", updated_offset_min=5),
        ],
        account_type="paper",
        limit=5,
    )

    assert payload["summary"]["open_untracked_count"] == 0
    assert payload["summary"]["recent_flagged_count"] == 0
    assert payload["summary"]["attributed_open_positions_pct"] == 100.0
    assert payload["alerts"] == []
