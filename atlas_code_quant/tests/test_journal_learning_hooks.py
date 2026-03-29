from __future__ import annotations
from atlas_code_quant.journal.models import TradingJournal
from atlas_code_quant.journal.service import (
    TradingJournalService,
    _select_matching_open_entry,
)
from atlas_code_quant.learning.ic_signal_tracker import ICSignalTracker


class _Brain:
    def __init__(self) -> None:
        self.outcomes: list[dict] = []

    def record_journal_outcome(self, payload: dict) -> None:
        self.outcomes.append(payload)


class _Learning:
    def __init__(self) -> None:
        self.calls: list[bool] = []

    def refresh(self, force: bool = False) -> dict:
        self.calls.append(force)
        return {"refreshed": force}


def test_process_closed_entry_payloads_updates_ic_and_learning(tmp_path):
    tracker = ICSignalTracker(tracker_path=tmp_path / "ic_tracker.json")
    signal_id = tracker.record_signal(
        symbol="AAPL",
        method="equity_long",
        predicted_move_pct=2.0,
        entry_price=100.0,
        timeframe="1h",
        selection_score=88.0,
    )
    tracker._state["signals"][signal_id]["recorded_at"] = "2026-03-28T10:00:00+00:00"
    tracker._save()

    brain = _Brain()
    learning = _Learning()
    service = TradingJournalService(
        tracker=object(),  # type: ignore[arg-type]
        brain=brain,
        learning=learning,  # type: ignore[arg-type]
        ic_tracker=tracker,
    )

    closed_entries = [{
        "journal_key": "paper:test:AAPL:1",
        "symbol": "AAPL",
        "strategy_type": "equity_long",
        "entry_price": 100.0,
        "exit_price": 103.0,
        "entry_time": "2026-03-28T10:01:00+00:00",
    }]

    outcomes = service._process_closed_entry_payloads("paper", closed_entries)

    assert outcomes[0]["ic_updated"] is True
    assert outcomes[0]["signal_id"] == signal_id
    assert tracker._state["signals"][signal_id]["outcome_available"] is True
    assert tracker._state["signals"][signal_id]["actual_return_pct"] == 3.0
    assert learning.calls == [True]
    assert brain.outcomes == closed_entries


def test_process_closed_entry_payloads_handles_empty_batch(tmp_path):
    service = TradingJournalService(
        tracker=object(),  # type: ignore[arg-type]
        brain=_Brain(),
        learning=_Learning(),  # type: ignore[arg-type]
        ic_tracker=ICSignalTracker(tracker_path=tmp_path / "ic_tracker.json"),
    )

    outcomes = service._process_closed_entry_payloads("paper", [])

    assert outcomes == []


def test_select_matching_open_entry_reuses_untracked_entry_for_same_equity_structure():
    entry = TradingJournal(
        journal_key="paper:test:untracked:SKM:1",
        account_type="paper",
        account_id="ACC",
        strategy_id="untracked:SKM:old",
        tracker_strategy_id="",
        strategy_type="untracked",
        symbol="SKM",
        legs_signature="equity:SKM",
        legs_details=(
            '[{"symbol":"SKM","asset_class":"equity","side":"long",'
            '"signed_qty":1.0,"entry_price":30.27,"current_price":29.92}]'
        ),
        status="open",
    )
    strategy = {
        "strategy_id": "equity_long:SKM:newtracker",
        "strategy_type": "equity_long",
        "underlying": "SKM",
        "positions": [
            {
                "symbol": "SKM",
                "asset_class": "equity",
                "side": "long",
                "signed_qty": 1.0,
                "entry_price": 30.27,
                "current_price": 29.92,
            }
        ],
    }

    matched = _select_matching_open_entry(
        [entry],
        strategy=strategy,
        strategy_id="equity_long:SKM:abcdef12345678",
    )

    assert matched is entry


def test_select_matching_open_entry_does_not_reuse_untracked_entry_for_opposite_side():
    entry = TradingJournal(
        journal_key="paper:test:untracked:YUM:1",
        account_type="paper",
        account_id="ACC",
        strategy_id="untracked:YUM:old",
        tracker_strategy_id="",
        strategy_type="untracked",
        symbol="YUM",
        legs_signature="equity:YUM",
        legs_details=(
            '[{"symbol":"YUM","asset_class":"equity","side":"long",'
            '"signed_qty":1.0,"entry_price":153.25,"current_price":153.96}]'
        ),
        status="open",
    )
    strategy = {
        "strategy_id": "equity_short:YUM:newtracker",
        "strategy_type": "equity_short",
        "underlying": "YUM",
        "positions": [
            {
                "symbol": "YUM",
                "asset_class": "equity",
                "side": "short",
                "signed_qty": -1.0,
                "entry_price": 153.25,
                "current_price": 153.96,
            }
        ],
    }

    matched = _select_matching_open_entry(
        [entry],
        strategy=strategy,
        strategy_id="equity_short:YUM:abcdef12345678",
    )

    assert matched is None
