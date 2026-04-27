"""Tests F9.3 — PaperBroker open/close + position monitor + journal."""
from __future__ import annotations

import time

from atlas_code_quant.execution.paper_broker import PaperBroker, PaperOpenPosition
from atlas_code_quant.execution.position_monitor import PositionMonitor
from atlas_code_quant.journal import TradeJournal
from atlas_code_quant.strategies.contracts import (
    OptionLeg,
    StrategyPlan,
)


def _plan(trace_id: str = "tr-1") -> StrategyPlan:
    return StrategyPlan(
        strategy="vertical_spread",
        symbol="SPY",
        direction="long",
        legs=[
            OptionLeg(side="buy", right="call", strike_offset=0.0, qty=1, expiry_dte=14),
            OptionLeg(side="sell", right="call", strike_offset=5.0, qty=1, expiry_dte=14),
        ],
        notional_estimate_usd=500.0,
        max_loss_estimate_usd=200.0,
        horizon_min=30,
        rationale="t",
        status="planned",
        trace_id=trace_id,
    )


def test_open_position_returns_id_and_targets() -> None:
    broker = PaperBroker()
    pos = broker.open_position(_plan(), entry_price=2.00, trace_id="tr-1")
    assert pos.position_id.startswith("POS-")
    assert pos.entry_price == 2.00
    assert pos.take_profit_price == round(2.00 * 1.5, 4)
    assert pos.stop_loss_price == 0.0
    assert pos.status == "open"
    assert broker.get_open_position(pos.position_id) is pos


def test_open_is_idempotent_by_trace_id() -> None:
    broker = PaperBroker()
    p = _plan("tr-2")
    a = broker.open_position(p, entry_price=1.50)
    b = broker.open_position(p, entry_price=1.50)
    assert a.position_id == b.position_id
    assert len(broker.open_positions()) == 1


def test_close_position_calculates_realized_pnl() -> None:
    broker = PaperBroker()
    pos = broker.open_position(_plan(), entry_price=2.00, trace_id="tr-3")
    closed = broker.close_position(pos.position_id, exit_price=3.00, reason="take_profit")
    # PnL = (3.00 - 2.00) * qty * 100 = 200.00
    assert closed.realized_pnl_usd == 200.00
    assert closed.exit_reason == "take_profit"
    assert closed.status == "closed"
    assert broker.get_open_position(pos.position_id) is None
    assert closed in broker.closed_positions()


def test_close_idempotent_on_already_closed() -> None:
    broker = PaperBroker()
    pos = broker.open_position(_plan(), entry_price=2.00, trace_id="tr-4")
    first = broker.close_position(pos.position_id, exit_price=2.50, reason="manual")
    # Re-cerrar es idempotente: devuelve la posición cerrada existente sin alterar PnL
    again = broker.close_position(pos.position_id, exit_price=999.0, reason="ignored")
    assert again is first
    # PnL = (2.5 - 2.0) * qty(2) * 100 = 100.0
    assert again.realized_pnl_usd == first.realized_pnl_usd == 100.0
    assert again.exit_reason == "manual"
    # Sigue habiendo sólo una posición en cerradas
    assert len(broker.closed_positions()) == 1


def test_close_unknown_position_raises() -> None:
    broker = PaperBroker()
    try:
        broker.close_position("POS-NOPE", exit_price=1.0, reason="x")
    except KeyError as e:
        assert "unknown_position" in str(e)
        return
    raise AssertionError("expected KeyError")


def test_monitor_take_profit() -> None:
    broker = PaperBroker()
    pos = broker.open_position(_plan(), entry_price=2.00, trace_id="tr-tp")
    monitor = PositionMonitor()
    decision = monitor.evaluate(pos, last_price=3.10)
    assert decision.action == "close"
    assert decision.reason == "take_profit"
    assert decision.suggested_exit_price == pos.take_profit_price


def test_monitor_stop_loss() -> None:
    broker = PaperBroker()
    pos = broker.open_position(_plan(), entry_price=2.00, trace_id="tr-sl")
    monitor = PositionMonitor()
    decision = monitor.evaluate(pos, last_price=-0.10)
    assert decision.action == "close"
    assert decision.reason == "stop_loss"


def test_monitor_time_stop() -> None:
    broker = PaperBroker()
    pos = broker.open_position(_plan(), entry_price=2.00, trace_id="tr-ts",
                               time_stop_seconds=1)
    monitor = PositionMonitor()
    # Avanzamos el reloj artificialmente
    decision = monitor.evaluate(pos, last_price=2.10, now=time.time() + 5)
    assert decision.action == "close"
    assert decision.reason == "time_stop"


def test_monitor_hold_in_band() -> None:
    broker = PaperBroker()
    pos = broker.open_position(_plan(), entry_price=2.00, trace_id="tr-hold")
    monitor = PositionMonitor()
    decision = monitor.evaluate(pos, last_price=2.30)
    assert decision.action == "hold"
    assert decision.reason == "hold"


def test_journal_records_open_and_close() -> None:
    broker = PaperBroker()
    journal = TradeJournal()
    pos = broker.open_position(_plan("trace-J"), entry_price=2.00)
    journal.record_open(pos)
    closed = broker.close_position(pos.position_id, exit_price=3.00, reason="take_profit")
    journal.record_close(closed)

    entries = journal.entries(trace_id="trace-J")
    assert len(entries) == 2
    events = [e.event for e in entries]
    assert "trade_open" in events and "trade_close" in events
    assert journal.has_complete_cycle("trace-J")
    close_entry = next(e for e in entries if e.event == "trade_close")
    assert close_entry.payload["realized_pnl_usd"] == 200.00


def test_journal_jsonl_persistence(tmp_path) -> None:
    log_file = tmp_path / "journal.jsonl"
    broker = PaperBroker()
    journal = TradeJournal(log_path=str(log_file))
    pos = broker.open_position(_plan("trace-FS"), entry_price=2.00)
    journal.record_open(pos)
    closed = broker.close_position(pos.position_id, exit_price=2.80, reason="take_profit")
    journal.record_close(closed)

    assert log_file.exists()
    lines = log_file.read_text(encoding="utf-8").strip().splitlines()
    assert len(lines) == 2
