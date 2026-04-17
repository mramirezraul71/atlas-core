from __future__ import annotations

import asyncio
from datetime import datetime
from zoneinfo import ZoneInfo

from atlas_code_quant.api import main


def test_market_hours_gate(monkeypatch) -> None:
    monkeypatch.setattr(main.settings, "market_open_schedule_open_et", "09:30")
    monkeypatch.setattr(main.settings, "market_open_schedule_close_et", "16:00")
    monkeypatch.setattr(main, "_broker_open_positions_count", lambda **kwargs: 0)

    monday_before_open = main._entry_pass_runtime_gate(
        action="submit",
        account_scope="paper",
        account_id=None,
        now_et=datetime(2026, 4, 13, 8, 45, tzinfo=ZoneInfo("America/New_York")),
    )
    saturday_midday = main._entry_pass_runtime_gate(
        action="submit",
        account_scope="paper",
        account_id=None,
        now_et=datetime(2026, 4, 11, 12, 0, tzinfo=ZoneInfo("America/New_York")),
    )

    assert monday_before_open["skip_entry_pass"] is True
    assert monday_before_open["reason"] == "market_closed"
    assert monday_before_open["market_open"] is False
    assert saturday_midday["skip_entry_pass"] is True
    assert saturday_midday["reason_detail"] == "market_closed_weekend"


def test_max_open_positions_guard(monkeypatch) -> None:
    monkeypatch.setattr(main.settings, "market_open_max_positions", 3)
    monkeypatch.setattr(main.settings, "market_open_schedule_open_et", "09:30")
    monkeypatch.setattr(main.settings, "market_open_schedule_close_et", "16:00")
    monkeypatch.setattr(main, "_broker_open_positions_count", lambda **kwargs: 3)

    payload = main._entry_pass_runtime_gate(
        action="submit",
        account_scope="paper",
        account_id=None,
        now_et=datetime(2026, 4, 13, 10, 0, tzinfo=ZoneInfo("America/New_York")),
    )

    assert payload["skip_entry_pass"] is True
    assert payload["reason"] == "max_open_positions"
    assert payload["open_positions"] == 3
    assert payload["max_open_positions"] == 3


def test_exit_governance_closes(monkeypatch) -> None:
    submitted_orders = []

    async def _no_sleep(_seconds: int) -> None:
        return None

    class _EventStore:
        def append(self, *_args, **_kwargs) -> None:
            return None

    def _empty_report(_limit: int) -> dict[str, object]:
        main._AUTO_CYCLE_STATE["running"] = False
        return {"candidates": []}

    monkeypatch.setattr(main.asyncio, "sleep", _no_sleep)
    monkeypatch.setattr(
        main._OPERATION_CENTER,
        "get_config",
        lambda: {"auton_mode": "paper_autonomous", "account_scope": "paper", "executor_mode": "paper_api"},
    )
    monkeypatch.setattr(
        main,
        "_entry_pass_runtime_gate",
        lambda **kwargs: {
            "action": kwargs.get("action"),
            "skip_entry_pass": False,
            "reason": None,
            "reason_detail": None,
            "open_positions": 0,
            "max_open_positions": 3,
            "market_open": True,
        },
    )
    monkeypatch.setattr(
        main._JOURNAL_PRO,
        "exit_governance_snapshot",
        lambda **kwargs: {
            "enabled": True,
            "recommendations": [
                {
                    "symbol": "SPY",
                    "strategy_type": "equity_long",
                    "recommendation": "exit_now",
                    "urgency": "high",
                    "exit_reason": "hard_stop_loss_r",
                }
            ],
        },
    )
    monkeypatch.setattr(main._JOURNAL, "sync_scope", lambda scope: {"scope": scope, "closed": 0})
    monkeypatch.setattr(main._SCANNER, "report", _empty_report)
    monkeypatch.setattr(main, "get_event_store", lambda: _EventStore())
    monkeypatch.setattr(
        main._OPERATION_CENTER,
        "evaluate_candidate",
        lambda **kwargs: submitted_orders.append(kwargs["order"]) or {"blocked": False},
    )

    asyncio.run(main._auto_cycle_loop(0, 1))

    assert len(submitted_orders) == 1
    close_order = submitted_orders[0]
    assert close_order.symbol == "SPY"
    assert close_order.position_effect == "close"
    assert close_order.side == "sell"
