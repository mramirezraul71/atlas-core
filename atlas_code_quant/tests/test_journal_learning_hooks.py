from __future__ import annotations
from contextlib import contextmanager

import atlas_code_quant.journal.service as journal_service_module
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


def test_select_matching_open_entry_reuses_attributed_entry_when_incoming_snapshot_is_untracked():
    entry = TradingJournal(
        journal_key="paper:test:equity_long:XOM:1",
        account_type="paper",
        account_id="ACC",
        strategy_id="equity_long:XOM:stable",
        tracker_strategy_id="equity_long:XOM:stable",
        strategy_type="equity_long",
        symbol="XOM",
        legs_signature="equity:XOM",
        legs_details=(
            '[{"symbol":"XOM","asset_class":"equity","side":"long",'
            '"signed_qty":1.0,"entry_price":111.87,"current_price":112.34}]'
        ),
        status="open",
    )
    strategy = {
        "strategy_id": "untracked:XOM:regressed",
        "strategy_type": "untracked",
        "underlying": "XOM",
        "positions": [
            {
                "symbol": "XOM",
                "asset_class": "equity",
                "side": "long",
                "signed_qty": 1.0,
                "entry_price": 111.87,
                "current_price": 112.34,
            }
        ],
    }

    matched = _select_matching_open_entry(
        [entry],
        strategy=strategy,
        strategy_id="untracked:XOM:deadbeef123456",
    )

    assert matched is entry


def test_sync_scope_skips_when_same_scope_is_already_running() -> None:
    service = TradingJournalService(
        tracker=object(),  # type: ignore[arg-type]
        brain=_Brain(),
        learning=_Learning(),  # type: ignore[arg-type]
    )
    sync_lock = service._scope_sync_lock("paper")
    assert sync_lock.acquire(blocking=False) is True
    try:
        outcome = service.sync_scope("paper")
    finally:
        sync_lock.release()

    assert outcome["skipped"] is True
    assert outcome["reason"] == "sync_already_running"
    assert outcome["scope"] == "paper"


def test_sync_scope_preserves_open_entries_when_strategy_snapshot_is_empty(monkeypatch) -> None:
    class _Tracker:
        def build_summary(self, **_: object) -> dict:
            return {"strategies": []}

    class _Account:
        scope = "paper"
        account_id = "ACC"

    open_entry = TradingJournal(
        journal_key="paper:ACC:equity_long:ZD:1",
        account_type="paper",
        account_id="ACC",
        strategy_id="equity_long:ZD:test",
        tracker_strategy_id="equity_long:ZD:test",
        strategy_type="equity_long",
        symbol="ZD",
        legs_signature="equity:ZD",
        legs_details='[{"symbol":"ZD","asset_class":"equity","side":"long","signed_qty":1.0}]',
        status="open",
    )

    class _FakeResult:
        def __init__(self, rows):
            self._rows = rows

        def scalars(self):
            return self

        def all(self):
            return list(self._rows)

    class _FakeDb:
        def execute(self, _query):
            return _FakeResult([open_entry])

    @contextmanager
    def _fake_session_scope():
        yield _FakeDb()

    monkeypatch.setattr(journal_service_module, "resolve_account_session", lambda **_: (object(), _Account()))
    monkeypatch.setattr(journal_service_module, "session_scope", _fake_session_scope)

    service = TradingJournalService(
        tracker=_Tracker(),  # type: ignore[arg-type]
        brain=_Brain(),
        learning=_Learning(),  # type: ignore[arg-type]
    )

    outcome = service.sync_scope("paper")

    assert outcome["skipped"] is True
    assert outcome["reason"] == "empty_strategy_snapshot"
    assert outcome["open_entries_preserved"] == 1


def test_close_entry_requires_broker_close_evidence() -> None:
    service = TradingJournalService(
        tracker=object(),  # type: ignore[arg-type]
        brain=_Brain(),
        learning=_Learning(),  # type: ignore[arg-type]
    )
    entry = TradingJournal(
        journal_key="paper:ACC:equity_long:AAOI:1",
        account_type="paper",
        account_id="ACC",
        strategy_id="equity_long:AAOI:test",
        tracker_strategy_id="equity_long:AAOI:test",
        strategy_type="equity_long",
        symbol="AAOI",
        legs_signature="equity:AAOI",
        legs_details='[{"symbol":"AAOI","asset_class":"equity","side":"long","signed_qty":1.0}]',
        entry_price=416.0,
        mark_price=392.84,
        status="open",
    )

    closed = service._close_entry(entry, [])

    assert closed is False
    assert entry.status == "open"
    assert entry.exit_time is None


def test_sync_scope_partial_snapshot_guard_skips_mass_close(monkeypatch) -> None:
    class _Tracker:
        def build_summary(self, **_: object) -> dict:
            return {
                "strategies": [
                    {
                        "strategy_id": "equity_long:AAOI:test",
                        "strategy_type": "equity_long",
                        "underlying": "AAOI",
                        "positions": [
                            {
                                "symbol": "AAOI",
                                "asset_class": "equity",
                                "side": "long",
                                "signed_qty": 1.0,
                                "entry_price": 10.0,
                                "current_price": 11.0,
                            }
                        ],
                    }
                ]
            }

    class _Account:
        scope = "paper"
        account_id = "ACC"

    open_entries = []
    for idx in range(40):
        symbol = f"S{idx}"
        open_entries.append(
            TradingJournal(
                journal_key=f"paper:ACC:equity_long:{symbol}:{idx}",
                account_type="paper",
                account_id="ACC",
                strategy_id=f"equity_long:{symbol}:id{idx}",
                tracker_strategy_id=f"equity_long:{symbol}:id{idx}",
                strategy_type="equity_long",
                symbol=symbol,
                legs_signature=f"equity:{symbol}",
                legs_details=f'[{{"symbol":"{symbol}","asset_class":"equity","side":"long","signed_qty":1.0}}]',
                status="open",
            )
        )

    class _FakeResult:
        def __init__(self, rows):
            self._rows = rows

        def scalars(self):
            return self

        def all(self):
            return list(self._rows)

    class _FakeDb:
        def __init__(self):
            self.added = []

        def execute(self, _query):
            return _FakeResult(open_entries)

        def add(self, item):
            self.added.append(item)

        def flush(self):
            return None

    @contextmanager
    def _fake_session_scope():
        yield _FakeDb()

    monkeypatch.setattr(journal_service_module, "resolve_account_session", lambda **_: (object(), _Account()))
    monkeypatch.setattr(journal_service_module, "session_scope", _fake_session_scope)
    monkeypatch.setattr(journal_service_module.TradingJournalService, "_trade_events", lambda self, client, account_id: [])
    monkeypatch.setattr(journal_service_module, "_estimate_iv_rank", lambda *args, **kwargs: None)

    service = TradingJournalService(
        tracker=_Tracker(),  # type: ignore[arg-type]
        brain=_Brain(),
        learning=_Learning(),  # type: ignore[arg-type]
    )

    outcome = service.sync_scope("paper")

    assert outcome["skipped"] is True
    assert outcome["reason"] == "partial_strategy_snapshot_guard"
    assert outcome["open_entries_preserved"] >= 39


def test_upsert_open_entry_normalizes_signed_prices(monkeypatch) -> None:
    class _Db:
        def __init__(self):
            self.added = []

        def execute(self, _query):
            class _Result:
                def scalars(self_inner):
                    return self_inner

                def all(self_inner):
                    return []

            return _Result()

        def add(self, item):
            self.added.append(item)

        def flush(self):
            return None

    class _Account:
        scope = "paper"
        account_id = "ACC"

    service = TradingJournalService(
        tracker=object(),  # type: ignore[arg-type]
        brain=_Brain(),
        learning=_Learning(),  # type: ignore[arg-type]
    )
    monkeypatch.setattr(journal_service_module, "_estimate_iv_rank", lambda *args, **kwargs: None)
    db = _Db()
    strategy = {
        "strategy_id": "equity_short:XYZ:test",
        "strategy_type": "equity_short",
        "underlying": "XYZ",
        "positions": [
            {
                "symbol": "XYZ",
                "asset_class": "equity",
                "side": "short",
                "signed_qty": -1.0,
                "entry_price": 45.91,
                "current_price": 45.82,
            }
        ],
        "open_pnl": 0.09,
        "spot": 45.82,
        "bpr_model": 45.91,
    }

    _strategy_id, created_now, skipped_now = service._upsert_open_entry(db, _Account(), object(), strategy, [])

    assert created_now is True
    assert isinstance(skipped_now, bool)
    entry = db.added[0]
    assert entry.entry_price == 45.91
    assert entry.entry_notional == 45.91
