"""Tests F6 — TradierAdapter (paper-first, sin HTTP)."""
from __future__ import annotations

import pytest

from atlas_code_quant.execution.tradier_adapter import (
    OrderRequest,
    TradierAdapter,
    TradierConfig,
)


def _order(symbol: str = "AAPL") -> OrderRequest:
    return OrderRequest(symbol=symbol, side="buy_to_open", quantity=1, order_type="market")


def test_dry_run_default_returns_dry_run_ticket() -> None:
    adapter = TradierAdapter(config=TradierConfig(dry_run=True))
    ticket = adapter.submit(_order())
    assert ticket.status == "dry_run"
    assert ticket.broker_order_id.startswith("DR-")
    assert "DRY_RUN" in ticket.rationale


def test_live_blocked_when_live_flag_disabled() -> None:
    adapter = TradierAdapter(
        config=TradierConfig(dry_run=False, live_enabled=False)
    )
    ticket = adapter.submit(_order())
    assert ticket.status == "blocked"
    assert ticket.rationale == "live_disabled_by_flag"


def test_live_enabled_returns_rejected_skeleton() -> None:
    adapter = TradierAdapter(
        config=TradierConfig(dry_run=False, live_enabled=True)
    )
    ticket = adapter.submit(_order())
    assert ticket.status == "rejected"
    assert "skeleton" in ticket.rationale


def test_rate_limit_blocks_after_max_per_minute() -> None:
    adapter = TradierAdapter(
        config=TradierConfig(dry_run=True, max_orders_per_minute=2)
    )
    a = adapter.submit(_order())
    b = adapter.submit(_order())
    c = adapter.submit(_order())
    assert a.status == "dry_run"
    assert b.status == "dry_run"
    assert c.status == "blocked"
    assert c.rationale == "rate_limit_exceeded"


def test_cancel_all_and_reconcile_are_noop_audited() -> None:
    adapter = TradierAdapter(config=TradierConfig(dry_run=True))
    cancel = adapter.cancel_all()
    rec = adapter.reconcile_positions()
    assert cancel["ok"] is True and cancel["dry_run"] is True
    assert rec["ok"] is True and rec["positions"] == []


def test_config_from_env(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv("ATLAS_TRADIER_DRY_RUN", "false")
    monkeypatch.setenv("ATLAS_LIVE_TRADING_ENABLED", "true")
    monkeypatch.setenv("ATLAS_MAX_ORDERS_PER_MINUTE", "7")
    cfg = TradierConfig.from_env()
    assert cfg.dry_run is False
    assert cfg.live_enabled is True
    assert cfg.max_orders_per_minute == 7
