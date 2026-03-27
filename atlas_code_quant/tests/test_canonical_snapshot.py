from __future__ import annotations

from types import SimpleNamespace
from unittest.mock import patch

from atlas_code_quant.monitoring.canonical_snapshot import CanonicalSnapshotService


class _Session:
    def __init__(self) -> None:
        self.scope = "paper"
        self.account_id = "PAPER-123"
        self.total_equity = 9750.0

    def to_dict(self) -> dict:
        return {
            "scope": self.scope,
            "account_id": self.account_id,
            "total_equity": self.total_equity,
        }


class _Tracker:
    def snapshot(self, **_: object) -> SimpleNamespace:
        position = SimpleNamespace(
            symbol="AAPL240419C00190000",
            underlying="AAPL",
            asset_class="option",
            side="long",
            quantity_abs=1,
            signed_qty=1,
            entry_price=4.5,
            current_price=3.2,
            current_pnl=-130.0,
            strike=190.0,
            option_type="call",
            expiration="2026-04-19",
            dte=23,
        )
        return SimpleNamespace(
            session=_Session(),
            groups=[{"strategy_id": "call:aapl:1"}],
            normalized_positions=[position],
        )

    def build_summary(self, **_: object) -> dict:
        return {
            "balances": {
                "total_equity": 9750.0,
                "cash": 9100.0,
                "margin": 0.0,
                "option_buying_power": 5000.0,
                "current_requirement": 250.0,
            },
            "pdt_status": {"day_trades_last_window": 1},
            "strategies": [{"strategy_id": "call:aapl:1"}],
            "alerts": [],
            "refresh_interval_sec": 5,
        }


def test_build_snapshot_is_tradier_first_and_reconciles():
    service = CanonicalSnapshotService(_Tracker())
    with patch.object(service, "_paper_local_snapshot", return_value={
        "source": "paper_local",
        "label": "Paper local",
        "status": "ok",
        "equity": 10020.0,
        "open_positions": 0,
        "open_pnl": 0.0,
        "updated_at": "2026-03-27T12:00:00",
    }), patch.object(service, "_optionstrat_snapshot", return_value={
        "source": "optionstrat",
        "label": "OptionStrat",
        "status": "ok",
        "open_positions": 2,
        "strategies": ["iron_condor"],
        "updated_at": "2026-03-27T12:00:00",
    }), patch.object(service, "_atlas_internal_snapshot", return_value={
        "source": "atlas_internal",
        "label": "Atlas internal",
        "status": "ok",
        "equity": 9500.0,
        "open_positions": 1,
        "open_pnl": -150.0,
        "updated_at": "2026-03-27T12:00:00",
    }):
        snapshot = service.build_snapshot(account_scope="paper", account_id="PAPER-123")

    assert snapshot["source"] == "tradier"
    assert snapshot["account_scope"] == "paper"
    assert snapshot["account_id"] == "PAPER-123"
    assert snapshot["totals"]["positions"] == 1
    assert snapshot["balances"]["total_equity"] == 9750.0
    assert snapshot["reconciliation"]["state"] == "failed"
    assert {item["comparison"] for item in snapshot["reconciliation"]["items"]} == {"atlas_internal", "paper_local"}


def test_build_status_payload_includes_balances_and_simulators():
    service = CanonicalSnapshotService(_Tracker())
    with patch.object(service, "_paper_local_snapshot", return_value={
        "source": "paper_local",
        "label": "Paper local",
        "status": "ok",
        "equity": 9751.0,
        "open_positions": 1,
        "open_pnl": -129.0,
        "updated_at": "2026-03-27T12:00:00",
    }), patch.object(service, "_optionstrat_snapshot", return_value={
        "source": "optionstrat",
        "label": "OptionStrat",
        "status": "ok",
        "open_positions": 1,
        "strategies": ["calendar"],
        "updated_at": "2026-03-27T12:00:00",
    }), patch.object(service, "_atlas_internal_snapshot", return_value={
        "source": "atlas_internal",
        "label": "Atlas internal",
        "status": "ok",
        "equity": 9750.0,
        "open_positions": 1,
        "open_pnl": -130.0,
        "updated_at": "2026-03-27T12:00:00",
    }):
        payload = service.build_status_payload(
            account_scope="paper",
            account_id="PAPER-123",
            uptime_sec=42.0,
            active_strategies=["gamma_scalper"],
            pdt_status={"day_trades_last_window": 1},
        )

    assert payload["source"] == "tradier"
    assert payload["balances"]["cash"] == 9100.0
    assert payload["open_positions"] == 1
    assert payload["simulators"]["optionstrat"]["open_positions"] == 1
    assert payload["reconciliation"]["state"] == "healthy"
