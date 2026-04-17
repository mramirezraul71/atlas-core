from __future__ import annotations

import json
import logging
from contextlib import contextmanager
from datetime import datetime, timedelta
from types import SimpleNamespace

import pytest

import atlas_code_quant.journal.service as journal_service_module
from atlas_code_quant.journal.service import TradingJournalService, _is_recent_entry
from atlas_code_quant.learning.ic_signal_tracker import ICSignalTracker


class _Tracker:
    def __init__(self, strategies: list[dict]) -> None:
        self._strategies = strategies

    def build_summary(self, **_: object) -> dict:
        return {"strategies": list(self._strategies)}


class _Brain:
    def record_journal_outcome(self, payload: dict) -> dict:
        return {"ok": True, "journal_key": payload.get("journal_key")}


class _Learning:
    def refresh(self, force: bool = False) -> dict:
        return {"ok": True, "force": force}


class _Client:
    def __init__(self, positions: list[dict]) -> None:
        self._positions = positions

    def positions(self, account_id: str) -> list[dict]:
        return [dict(item) for item in self._positions]


class _Account:
    scope = "paper"
    account_id = "ACC-1"


class _FakeResult:
    def __init__(self, rows):
        self._rows = rows

    def scalars(self):
        return self

    def all(self):
        return list(self._rows)


class _FakeDb:
    def __init__(self, rows):
        self.rows = rows

    def execute(self, _query):
        return _FakeResult(self.rows)

    def add(self, _item):
        return None

    def flush(self):
        return None


def _entry(
    *,
    strategy_id: str,
    symbol: str,
    broker_order_ids_json: str,
    signed_qty: float = 1.0,
    entry_time: datetime | None = None,
):
    return SimpleNamespace(
        strategy_id=strategy_id,
        symbol=symbol,
        status="open",
        journal_key=f"{strategy_id}:{symbol}",
        broker_order_ids_json=broker_order_ids_json,
        entry_time=entry_time or (datetime.utcnow() - timedelta(hours=2)),
        legs_details=json.dumps(
            [
                {
                    "symbol": symbol,
                    "asset_class": "equity",
                    "signed_qty": signed_qty,
                }
            ]
        ),
    )


def _install_session(monkeypatch, rows):
    @contextmanager
    def _fake_session_scope():
        yield _FakeDb(rows)

    monkeypatch.setattr(journal_service_module, "session_scope", _fake_session_scope)


def _install_service_basics(monkeypatch, *, positions: list[dict], rows: list, strategies: list[dict]) -> TradingJournalService:
    _install_session(monkeypatch, rows)
    monkeypatch.setattr(
        journal_service_module,
        "resolve_account_session",
        lambda **kwargs: (_Client(positions), _Account()),
    )
    monkeypatch.setattr(TradingJournalService, "_trade_events", lambda self, client, account_id: [])
    monkeypatch.setattr(TradingJournalService, "_upsert_open_entry", lambda self, db, account, client, strategy, matched: (strategy["strategy_id"], False, False))
    monkeypatch.setattr(TradingJournalService, "_should_guard_mass_close", lambda self, **kwargs: False)
    monkeypatch.setattr(TradingJournalService, "_close_entry", lambda self, entry, matched_events: False)
    monkeypatch.setattr(TradingJournalService, "_process_closed_entry_payloads", lambda self, scope, closed_entries: [])
    return TradingJournalService(
        tracker=_Tracker(strategies=strategies),  # type: ignore[arg-type]
        brain=_Brain(),
        learning=_Learning(),  # type: ignore[arg-type]
    )


@pytest.fixture
def mock_broker_client():
    return _Client([])


@pytest.fixture
def mock_session():
    return _Account()


@pytest.fixture
def sample_broker_position():
    return {"symbol": "SPY", "quantity": 100, "id": "12345"}


@pytest.fixture
def sample_journal_entry():
    return _entry(
        strategy_id="sample",
        symbol="SPY",
        broker_order_ids_json='{"id":"12345"}',
        signed_qty=100,
        entry_time=datetime.utcnow(),
    )


@pytest.mark.integration
class TestSyncScopeFallback:
    def test_sync_scope_match_by_broker_order_id(self, monkeypatch) -> None:
        rows = [_entry(strategy_id="s1", symbol="SPY", broker_order_ids_json='{"id":"12345"}', signed_qty=100)]
        service = _install_service_basics(
            monkeypatch,
            positions=[{"id": "12345", "symbol": "SPY", "quantity": 100}],
            rows=rows,
            strategies=[{"strategy_id": "s1", "underlying": "SPY", "positions": [{"symbol": "SPY"}]}],
        )
        result = service.sync_scope("paper")
        assert result["reconciliation_status"] == "OK"
        assert len(result["matched"]) == 1
        assert result["unmatched_broker"] == []
        assert result["unmatched_journal"] == []

    def test_sync_scope_fallback_match_symbol_qty_recency(self, monkeypatch, caplog) -> None:
        rows = [_entry(strategy_id="s1", symbol="SPY", broker_order_ids_json="[]", signed_qty=100)]
        service = _install_service_basics(
            monkeypatch,
            positions=[{"symbol": "SPY", "quantity": 100}],
            rows=rows,
            strategies=[{"strategy_id": "s1", "underlying": "SPY", "positions": [{"symbol": "SPY"}]}],
        )
        with caplog.at_level(logging.INFO, logger="quant.journal"):
            result = service.sync_scope("paper")
        assert result["reconciliation_status"] == "OK"
        assert len(result["matched"]) == 1
        assert "fallback" in caplog.text.lower()

    def test_sync_scope_phantom_detection(self, monkeypatch, caplog) -> None:
        rows = [_entry(strategy_id="s1", symbol="SPY", broker_order_ids_json='{"id":"12345"}', signed_qty=100)]
        service = _install_service_basics(
            monkeypatch,
            positions=[
                {"id": "12345", "symbol": "SPY", "quantity": 100},
                {"id": "98765", "symbol": "QQQ", "quantity": 50},
            ],
            rows=rows,
            strategies=[{"strategy_id": "s1", "underlying": "SPY", "positions": [{"symbol": "SPY"}]}],
        )
        result = service.sync_scope("paper")
        assert result["reconciliation_status"] == "PARTIAL_UNMATCHED_BROKER"
        assert any(item.get("symbol") == "QQQ" for item in result["unmatched_broker"])
        assert "PHANTOM DETECTED" in caplog.text

    def test_sync_scope_unmatched_journal_entry(self, monkeypatch) -> None:
        rows = [
            _entry(strategy_id="s1", symbol="SPY", broker_order_ids_json='{"id":"12345"}', signed_qty=100),
            _entry(strategy_id="s2", symbol="QQQ", broker_order_ids_json="[]", signed_qty=100),
        ]
        service = _install_service_basics(
            monkeypatch,
            positions=[{"id": "12345", "symbol": "SPY", "quantity": 100}],
            rows=rows,
            strategies=[
                {"strategy_id": "s1", "underlying": "SPY", "positions": [{"symbol": "SPY"}]},
                {"strategy_id": "s2", "underlying": "QQQ", "positions": [{"symbol": "QQQ"}]},
            ],
        )
        result = service.sync_scope("paper")
        assert result["reconciliation_status"] == "PARTIAL_UNMATCHED_JOURNAL"
        assert any(item.get("symbol") == "QQQ" for item in result["unmatched_journal"])

    def test_sync_scope_fallback_match_qty_mismatch(self, monkeypatch) -> None:
        rows = [_entry(strategy_id="s1", symbol="SPY", broker_order_ids_json="[]", signed_qty=100)]
        service = _install_service_basics(
            monkeypatch,
            positions=[{"symbol": "SPY", "quantity": 50}],
            rows=rows,
            strategies=[{"strategy_id": "s1", "underlying": "SPY", "positions": [{"symbol": "SPY"}]}],
        )
        result = service.sync_scope("paper")
        assert any(item.get("symbol") == "SPY" for item in result["unmatched_journal"])
        assert any(item.get("symbol") == "SPY" for item in result["unmatched_broker"])

    def test_sync_scope_fallback_match_old_entry(self, monkeypatch) -> None:
        rows = [
            _entry(
                strategy_id="s1",
                symbol="SPY",
                broker_order_ids_json="[]",
                signed_qty=100,
                entry_time=datetime.utcnow() - timedelta(days=5),
            )
        ]
        service = _install_service_basics(
            monkeypatch,
            positions=[{"symbol": "SPY", "quantity": 100}],
            rows=rows,
            strategies=[{"strategy_id": "s1", "underlying": "SPY", "positions": [{"symbol": "SPY"}]}],
        )
        result = service.sync_scope("paper")
        assert any(item.get("symbol") == "SPY" for item in result["unmatched_journal"])
        assert any(item.get("symbol") == "SPY" for item in result["unmatched_broker"])

    def test_sync_scope_multiple_positions_complex(self, monkeypatch) -> None:
        rows = [
            _entry(strategy_id="s1", symbol="SPY", broker_order_ids_json='{"id":"OID-1"}', signed_qty=100),
            _entry(strategy_id="s2", symbol="QQQ", broker_order_ids_json="[]", signed_qty=50),
            _entry(strategy_id="s3", symbol="IWM", broker_order_ids_json='{"id":"OID-3"}', signed_qty=30),
        ]
        service = _install_service_basics(
            monkeypatch,
            positions=[
                {"id": "OID-1", "symbol": "SPY", "quantity": 100},
                {"symbol": "QQQ", "quantity": 50},
                {"symbol": "AAPL", "quantity": 20, "id": "OID-X"},
            ],
            rows=rows,
            strategies=[
                {"strategy_id": "s1", "underlying": "SPY", "positions": [{"symbol": "SPY"}]},
                {"strategy_id": "s2", "underlying": "QQQ", "positions": [{"symbol": "QQQ"}]},
                {"strategy_id": "s3", "underlying": "IWM", "positions": [{"symbol": "IWM"}]},
            ],
        )
        result = service.sync_scope("paper")
        assert len(result["matched"]) == 2
        assert any(item.get("symbol") == "AAPL" for item in result["unmatched_broker"])
        assert any(item.get("symbol") == "IWM" for item in result["unmatched_journal"])
        assert result["reconciliation_status"].startswith("PARTIAL_UNMATCHED_")

    def test_sync_scope_empty_broker_empty_journal(self, monkeypatch) -> None:
        service = _install_service_basics(monkeypatch, positions=[], rows=[], strategies=[])
        result = service.sync_scope("paper")
        assert result["matched"] == []
        assert result["unmatched_broker"] == []
        assert result["unmatched_journal"] == []
        assert result["reconciliation_status"] == "OK"

    def test_sync_scope_logging_phantom(self, monkeypatch, caplog) -> None:
        service = _install_service_basics(
            monkeypatch,
            positions=[{"symbol": "QQQ", "quantity": 42, "id": "OID-QQQ"}],
            rows=[],
            strategies=[],
        )
        service.sync_scope("paper")
        assert "PHANTOM DETECTED" in caplog.text
        assert "QQQ" in caplog.text
        assert "42" in caplog.text

    def test_post_fill_reconciliation(self, monkeypatch) -> None:
        rows = [_entry(strategy_id="s1", symbol="SPY", broker_order_ids_json="[]", signed_qty=100)]
        service = _install_service_basics(
            monkeypatch,
            positions=[{"id": "OID-RECOVERED", "symbol": "SPY", "quantity": 100}],
            rows=rows,
            strategies=[{"strategy_id": "s1", "underlying": "SPY", "positions": [{"symbol": "SPY"}]}],
        )

        result = service.sync_scope("paper")

        assert result["recovered_broker_order_ids"] == 1
        assert json.loads(rows[0].broker_order_ids_json) == ["OID-RECOVERED"]
        assert result["journal_entries"][0]["broker_order_id"] == "OID-RECOVERED"

    def test_post_fill_reconciliation_rejects_ambiguous_broker_matches(self, monkeypatch) -> None:
        rows = [_entry(strategy_id="s1", symbol="SPY", broker_order_ids_json="[]", signed_qty=100)]
        service = _install_service_basics(
            monkeypatch,
            positions=[
                {"id": "OID-RECOVERED-1", "symbol": "SPY", "quantity": 100},
                {"id": "OID-RECOVERED-2", "symbol": "SPY", "quantity": 100},
            ],
            rows=rows,
            strategies=[{"strategy_id": "s1", "underlying": "SPY", "positions": [{"symbol": "SPY"}]}],
        )

        result = service.sync_scope("paper")

        assert result["recovered_broker_order_ids"] == 0
        assert json.loads(rows[0].broker_order_ids_json) == []
        assert result["journal_entries"][0]["broker_order_id"] is None

    def test_ic_tracker_closed_loop(self, monkeypatch, tmp_path) -> None:
        tracker = ICSignalTracker(tracker_path=tmp_path / "ic_tracker.json")
        signal_id = tracker.record_signal(
            symbol="AAPL",
            method="equity_long",
            predicted_move_pct=2.0,
            entry_price=100.0,
            timeframe="1h",
            selection_score=86.0,
        )
        tracker._state["signals"][signal_id]["recorded_at"] = "2026-04-17T13:50:00+00:00"
        tracker._save()

        rows = [
            _entry(
                strategy_id="s1",
                symbol="AAPL",
                broker_order_ids_json="[]",
                signed_qty=1,
                entry_time=datetime(2026, 4, 17, 14, 0, 0),
            )
        ]
        _install_session(monkeypatch, rows)
        monkeypatch.setattr(
            journal_service_module,
            "resolve_account_session",
            lambda **kwargs: (_Client([{"id": "OID-AAPL-1", "symbol": "AAPL", "quantity": 1}]), _Account()),
        )
        monkeypatch.setattr(TradingJournalService, "_trade_events", lambda self, client, account_id: [])
        monkeypatch.setattr(
            TradingJournalService,
            "_upsert_open_entry",
            lambda self, db, account, client, strategy, matched: (strategy["strategy_id"], False, False),
        )
        monkeypatch.setattr(TradingJournalService, "_should_guard_mass_close", lambda self, **kwargs: False)
        monkeypatch.setattr(TradingJournalService, "_close_entry", lambda self, entry, matched_events: False)

        class _TrackedBrain(_Brain):
            def __init__(self) -> None:
                self.outcomes: list[dict] = []

            def record_journal_outcome(self, payload: dict) -> dict:
                self.outcomes.append(payload)
                return {"ok": True, "journal_key": payload.get("journal_key")}

        brain = _TrackedBrain()

        class _TrackedLearning(_Learning):
            def __init__(self) -> None:
                self.calls: list[bool] = []

            def refresh(self, force: bool = False) -> dict:
                self.calls.append(force)
                return {"ok": True, "force": force}

        learning = _TrackedLearning()
        service = TradingJournalService(
            tracker=_Tracker(strategies=[{"strategy_id": "s1", "underlying": "AAPL", "positions": [{"symbol": "AAPL"}]}]),  # type: ignore[arg-type]
            brain=brain,
            learning=learning,  # type: ignore[arg-type]
            ic_tracker=tracker,
        )

        result = service.sync_scope("paper")
        assert result["recovered_broker_order_ids"] == 1
        assert json.loads(rows[0].broker_order_ids_json) == ["OID-AAPL-1"]

        outcomes = service._process_closed_entry_payloads(
            "paper",
            [
                {
                    "journal_key": "paper:test:AAPL:1",
                    "symbol": "AAPL",
                    "strategy_type": "equity_long",
                    "entry_price": 100.0,
                    "exit_price": 103.0,
                    "entry_time": "2026-04-17T14:00:00+00:00",
                }
            ],
        )

        assert outcomes[0]["ic_updated"] is True
        assert outcomes[0]["signal_id"] == signal_id
        assert tracker._state["signals"][signal_id]["outcome_available"] is True
        assert learning.calls == [True]
        assert len(brain.outcomes) == 1


@pytest.mark.unit
class TestSyncScopeHelpers:
    def test_is_recent_entry_true(self) -> None:
        entry_time = datetime.utcnow() - timedelta(hours=12)
        assert _is_recent_entry(entry_time, days=1) is True

    def test_is_recent_entry_false(self) -> None:
        entry_time = datetime.utcnow() - timedelta(days=2)
        assert _is_recent_entry(entry_time, days=1) is False
