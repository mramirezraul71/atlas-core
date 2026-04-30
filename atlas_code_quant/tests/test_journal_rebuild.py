from __future__ import annotations

from datetime import date, datetime
from pathlib import Path

import atlas_code_quant.journal.service as journal_service_module
from atlas_code_quant.journal.service import (
    _rebuild_journal_rows_from_closed_positions,
    _rebuild_journal_rows_from_events,
    rebuild_from_broker_history,
)


def test_rebuild_journal_rows_from_events_pairs_long_buy_sell() -> None:
    events = [
        {"timestamp": datetime(2026, 4, 11, 10, 0, 0), "symbol": "AAOI", "action": "buy", "price": 100.0, "quantity": 2.0, "commission": 1.0, "raw": {"id": 1}},
        {"timestamp": datetime(2026, 4, 11, 10, 15, 0), "symbol": "AAOI", "action": "sell", "price": 105.0, "quantity": 2.0, "commission": 1.0, "raw": {"id": 2}},
    ]

    rows = _rebuild_journal_rows_from_events(events, account_scope="paper", account_id="ACC", symbols=["AAOI"])

    assert len(rows) == 1
    assert rows[0]["strategy_type"] == "equity_long"
    assert rows[0]["entry_price"] == 100.0
    assert rows[0]["exit_price"] == 105.0
    assert rows[0]["realized_pnl"] == 8.0


def test_rebuild_journal_rows_from_events_pairs_short_sell_buy() -> None:
    events = [
        {"timestamp": datetime(2026, 4, 11, 10, 0, 0), "symbol": "SPY", "action": "sell", "price": 500.0, "quantity": 1.0, "commission": 0.5, "raw": {"id": 1}},
        {"timestamp": datetime(2026, 4, 11, 10, 30, 0), "symbol": "SPY", "action": "buy", "price": 490.0, "quantity": 1.0, "commission": 0.5, "raw": {"id": 2}},
    ]

    rows = _rebuild_journal_rows_from_events(events, account_scope="paper", account_id="ACC", symbols=["SPY"])

    assert len(rows) == 1
    assert rows[0]["strategy_type"] == "equity_short"
    assert rows[0]["entry_price"] == 500.0
    assert rows[0]["exit_price"] == 490.0
    assert rows[0]["realized_pnl"] == 9.0


def test_rebuild_from_broker_history_returns_quality_payload(monkeypatch, tmp_path: Path) -> None:
    class _Client:
        pass

    class _Account:
        scope = "paper"
        account_id = "ACC"

    monkeypatch.setattr(journal_service_module, "resolve_account_session", lambda **_: (_Client(), _Account()))
    monkeypatch.setattr(
        journal_service_module.TradingJournalService,
        "_trade_events",
        lambda self, client, account_id: [
            {"timestamp": datetime(2026, 4, 11, 10, 0, 0), "symbol": "AAOI", "action": "buy", "price": 100.0, "quantity": 1.0, "commission": 0.0, "raw": {}},
            {"timestamp": datetime(2026, 4, 11, 11, 0, 0), "symbol": "AAOI", "action": "sell", "price": 110.0, "quantity": 1.0, "commission": 0.0, "raw": {}},
        ],
    )
    monkeypatch.setattr(journal_service_module.TradingJournalService, "init_db", lambda self: None)

    payload = rebuild_from_broker_history(
        start_date="2026-04-01",
        end_date="2026-04-12",
        symbols=["AAOI"],
        validate_quality=True,
        quality_threshold_pct=80.0,
        persist=False,
        export_dir=tmp_path,
    )

    assert payload["journal_rebuilt"] == 1
    assert payload["score_calidad"] >= 80.0
    assert payload["ml_habilitado"] is True
    assert Path(payload["export_path"]).exists()


def test_rebuild_from_broker_history_keeps_ml_disabled_when_no_rows(monkeypatch, tmp_path: Path) -> None:
    class _Client:
        pass

    class _Account:
        scope = "paper"
        account_id = "ACC"

    monkeypatch.setattr(journal_service_module, "resolve_account_session", lambda **_: (_Client(), _Account()))
    monkeypatch.setattr(journal_service_module.TradingJournalService, "_trade_events", lambda self, client, account_id: [])
    monkeypatch.setattr(journal_service_module.TradingJournalService, "init_db", lambda self: None)

    payload = rebuild_from_broker_history(
        start_date="2026-04-01",
        end_date="2026-04-12",
        symbols=["AAOI"],
        validate_quality=True,
        quality_threshold_pct=80.0,
        persist=False,
        export_dir=tmp_path,
    )

    assert payload["journal_rebuilt"] == 0
    assert payload["ml_habilitado"] is False


def test_rebuild_journal_rows_from_closed_positions_creates_rows() -> None:
    rows = _rebuild_journal_rows_from_closed_positions(
        [
            {
                "symbol": "AXTI",
                "open_date": "2026-03-30T00:00:00.000Z",
                "close_date": "2026-04-02T00:00:00.000Z",
                "quantity": 1.0,
                "cost": 68.19,
                "proceeds": 42.4,
                "gain_loss": -25.79,
            }
        ],
        account_scope="paper",
        account_id="ACC",
        symbols=["AXTI"],
    )

    assert len(rows) == 1
    assert rows[0]["strategy_type"] == "rebuild_equity_closed"
    assert rows[0]["entry_price"] == 68.19
    assert rows[0]["exit_price"] == 42.4
    assert rows[0]["realized_pnl"] == -25.79


def test_rebuild_from_broker_history_falls_back_to_closed_positions(monkeypatch, tmp_path: Path) -> None:
    class _Client:
        pass

    class _Account:
        scope = "paper"
        account_id = "ACC"

    monkeypatch.setattr(journal_service_module, "resolve_account_session", lambda **_: (_Client(), _Account()))
    monkeypatch.setattr(journal_service_module.TradingJournalService, "_trade_events", lambda self, client, account_id: [])
    monkeypatch.setattr(
        journal_service_module.TradingJournalService,
        "_closed_positions",
        lambda self, client, account_id: [
            {
                "symbol": "AXTI",
                "open_date": "2026-03-30T00:00:00.000Z",
                "close_date": "2026-04-02T00:00:00.000Z",
                "quantity": 1.0,
                "cost": 68.19,
                "proceeds": 42.4,
                "gain_loss": -25.79,
            }
        ],
    )
    monkeypatch.setattr(journal_service_module.TradingJournalService, "init_db", lambda self: None)

    payload = rebuild_from_broker_history(
        start_date="2026-04-01",
        end_date="2026-04-12",
        symbols=["AXTI"],
        validate_quality=True,
        quality_threshold_pct=80.0,
        persist=False,
        export_dir=tmp_path,
    )

    assert payload["history_source"] == "gainloss_closed_positions"
    assert payload["closed_positions_considered"] == 1
    assert payload["journal_rebuilt"] == 1
    assert payload["ml_habilitado"] is True


def test_rebuild_from_broker_history_skips_negative_price_rows(monkeypatch, tmp_path: Path) -> None:
    class _Client:
        pass

    class _Account:
        scope = "paper"
        account_id = "ACC"

    monkeypatch.setattr(journal_service_module, "resolve_account_session", lambda **_: (_Client(), _Account()))
    monkeypatch.setattr(
        journal_service_module.TradingJournalService,
        "_trade_events",
        lambda self, client, account_id: [],
    )
    monkeypatch.setattr(journal_service_module.TradingJournalService, "init_db", lambda self: None)
    monkeypatch.setattr(
        journal_service_module,
        "_rebuild_journal_rows_from_events",
        lambda *args, **kwargs: [
            {
                "journal_key": "valid-row",
                "account_type": "paper",
                "account_id": "ACC",
                "strategy_id": "valid",
                "tracker_strategy_id": None,
                "strategy_type": "equity_long",
                "symbol": "AAOI",
                "legs_signature": "AAOI",
                "legs_details": "[]",
                "entry_price": 100.0,
                "exit_price": 110.0,
                "fees": 0.0,
                "win_rate_at_entry": None,
                "current_win_rate_pct": None,
                "iv_rank": None,
                "thesis_rich_text": "",
                "entry_time": datetime(2026, 4, 11, 10, 0, 0),
                "exit_time": datetime(2026, 4, 11, 11, 0, 0),
                "status": "closed",
                "is_level4": False,
                "realized_pnl": 10.0,
                "unrealized_pnl": 0.0,
                "mark_price": None,
                "spot_price": None,
                "entry_notional": 100.0,
                "risk_at_entry": 50.0,
                "greeks_json": "{}",
                "attribution_json": "{}",
                "post_mortem_json": "{}",
                "post_mortem_text": "",
                "broker_order_ids_json": "[]",
                "raw_entry_payload_json": "{}",
                "raw_exit_payload_json": "[]",
                "created_at": datetime(2026, 4, 11, 11, 0, 0),
                "updated_at": datetime(2026, 4, 11, 11, 0, 0),
                "last_synced_at": datetime(2026, 4, 11, 11, 0, 0),
            },
            {
                "journal_key": "invalid-row",
                "account_type": "paper",
                "account_id": "ACC",
                "strategy_id": "invalid",
                "tracker_strategy_id": None,
                "strategy_type": "equity_long",
                "symbol": "SPY",
                "legs_signature": "SPY",
                "legs_details": "[]",
                "entry_price": -10.0,
                "exit_price": 12.0,
                "fees": 0.0,
                "win_rate_at_entry": None,
                "current_win_rate_pct": None,
                "iv_rank": None,
                "thesis_rich_text": "",
                "entry_time": datetime(2026, 4, 11, 12, 0, 0),
                "exit_time": datetime(2026, 4, 11, 13, 0, 0),
                "status": "closed",
                "is_level4": False,
                "realized_pnl": 22.0,
                "unrealized_pnl": 0.0,
                "mark_price": None,
                "spot_price": None,
                "entry_notional": 10.0,
                "risk_at_entry": 5.0,
                "greeks_json": "{}",
                "attribution_json": "{}",
                "post_mortem_json": "{}",
                "post_mortem_text": "",
                "broker_order_ids_json": "[]",
                "raw_entry_payload_json": "{}",
                "raw_exit_payload_json": "[]",
                "created_at": datetime(2026, 4, 11, 13, 0, 0),
                "updated_at": datetime(2026, 4, 11, 13, 0, 0),
                "last_synced_at": datetime(2026, 4, 11, 13, 0, 0),
            },
        ],
    )

    payload = rebuild_from_broker_history(
        start_date="2026-04-11",
        end_date="2026-04-12",
        validate_quality=True,
        quality_threshold_pct=80.0,
        persist=False,
        export_dir=tmp_path,
    )

    assert payload["journal_rebuilt"] == 1
    assert payload["invalid_rebuilt_rows"] == 1
    assert payload["invalid_rebuilt_row_samples"][0]["symbol"] == "SPY"
    assert "negative_entry_price" in payload["invalid_rebuilt_row_samples"][0]["flags"]


def test_rebuild_from_broker_history_refreshes_adaptive_snapshot_and_resets_operation_state(monkeypatch, tmp_path: Path) -> None:
    class _Client:
        pass

    class _Account:
        scope = "paper"
        account_id = "ACC"

    class _FakeResult:
        def __init__(self, rowcount: int = 0) -> None:
            self.rowcount = rowcount

    class _FakeSession:
        def __init__(self) -> None:
            self.added: list[object] = []
            self.executed = 0

        def execute(self, _query):
            self.executed += 1
            return _FakeResult(0)

        def add(self, item: object) -> None:
            self.added.append(item)

    class _FakeSessionScope:
        def __init__(self) -> None:
            self.session = _FakeSession()

        def __enter__(self):
            return self.session

        def __exit__(self, exc_type, exc, tb):
            return False

    refresh_calls: list[date] = []
    reset_calls: list[str] = []

    def _fake_refresh(*, min_cutoff_date):
        refresh_calls.append(min_cutoff_date)
        return {"sample_count": 7}

    def _fake_reset(*, account_scope: str):
        reset_calls.append(account_scope)
        return {"ok": True, "account_scope": account_scope}

    monkeypatch.setattr(journal_service_module, "resolve_account_session", lambda **_: (_Client(), _Account()))
    monkeypatch.setattr(
        journal_service_module.TradingJournalService,
        "_trade_events",
        lambda self, client, account_id: [
            {"timestamp": datetime(2026, 4, 11, 10, 0, 0), "symbol": "AAOI", "action": "buy", "price": 100.0, "quantity": 1.0, "commission": 0.0, "raw": {}},
            {"timestamp": datetime(2026, 4, 11, 11, 0, 0), "symbol": "AAOI", "action": "sell", "price": 110.0, "quantity": 1.0, "commission": 0.0, "raw": {}},
        ],
    )
    monkeypatch.setattr(journal_service_module.TradingJournalService, "init_db", lambda self: None)
    monkeypatch.setattr(journal_service_module, "session_scope", lambda: _FakeSessionScope())
    monkeypatch.setattr(journal_service_module, "_refresh_adaptive_snapshot_after_rebuild", _fake_refresh)
    monkeypatch.setattr(journal_service_module, "_reset_post_rebuild_operational_state", _fake_reset)

    payload = rebuild_from_broker_history(
        start_date="2026-04-01",
        end_date="2026-04-12",
        symbols=["AAOI"],
        validate_quality=True,
        quality_threshold_pct=80.0,
        persist=True,
        export_dir=tmp_path,
    )

    assert refresh_calls == [date(2026, 4, 11)]
    assert reset_calls == ["paper"]
    assert payload["adaptive_snapshot_cutoff"] == "2026-04-11"
    assert payload["adaptive_snapshot_sample_count"] == 7
    assert payload["operation_reset"]["ok"] is True
