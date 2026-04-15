from __future__ import annotations

import sys
from pathlib import Path
from unittest.mock import MagicMock

import pytest

ROOT = Path(__file__).resolve().parents[2]
QUANT_ROOT = ROOT / "atlas_code_quant"
if str(QUANT_ROOT) not in sys.path:
    sys.path.insert(0, str(QUANT_ROOT))

from api.schemas import OrderRequest
import execution.tradier_execution as tradier_execution_module


class _FakeClient:
    def __init__(self, *, place_results: list[dict] | None = None, orders_results: list[list[dict]] | None = None) -> None:
        self.place_results = list(place_results or [])
        self.orders_results = list(orders_results or [])

    def place_order(self, account_id: str, data: dict) -> dict:
        if self.place_results:
            return dict(self.place_results.pop(0))
        return {"id": "default-order", "status": "accepted"}

    def orders(self, account_id: str) -> list[dict]:
        if self.orders_results:
            return [dict(item) for item in self.orders_results.pop(0)]
        return []


class _FakeSession:
    scope = "paper"
    classification = "paper"
    account_id = "ACC-PAPER"
    cash_available = 100000.0
    option_buying_power = 100000.0
    total_equity = 150000.0

    def to_dict(self) -> dict:
        return {
            "scope": self.scope,
            "classification": self.classification,
            "account_id": self.account_id,
        }


@pytest.mark.unit
class TestTradierExecutionFallback:
    def test_stream_order_confirmation_success(self) -> None:
        client = _FakeClient(place_results=[{"id": "12345", "status": "filled", "exec_quantity": 100}])
        session = _FakeSession()
        result = tradier_execution_module._stream_order_confirmation(
            client=client,
            session=session,
            payload={"symbol": "SPY", "side": "buy", "quantity": "100", "type": "market"},
            timeout_sec=5,
        )
        assert result["order_id"] == "12345"
        assert result["status"] == "filled"

    def test_stream_order_confirmation_timeout(self, monkeypatch) -> None:
        client = _FakeClient(place_results=[{"status": "pending"}])
        session = _FakeSession()
        perf_counter = iter([0.0, 6.0])
        monkeypatch.setattr(tradier_execution_module.time, "perf_counter", lambda: next(perf_counter))
        with pytest.raises(TimeoutError):
            tradier_execution_module._stream_order_confirmation(
                client=client,
                session=session,
                payload={"symbol": "SPY", "side": "buy", "quantity": "100", "type": "market"},
                timeout_sec=5,
            )

    def test_polling_get_orders_finds_match(self) -> None:
        client = _FakeClient(
            place_results=[{"status": "pending"}],
            orders_results=[[{"id": "67890", "symbol": "SPY", "qty": 100, "side": "buy", "type": "market"}]],
        )
        session = _FakeSession()
        result = tradier_execution_module._polling_get_orders(
            client=client,
            session=session,
            payload={"symbol": "SPY", "side": "buy", "quantity": 100, "type": "market"},
            timeout_sec=5,
        )
        assert result["order_id"] == "67890"
        assert result["method_used"] == "polling_get_orders"

    def test_polling_get_orders_timeout(self) -> None:
        client = _FakeClient(place_results=[{"status": "pending"}], orders_results=[[], [], []])
        session = _FakeSession()
        with pytest.raises(TimeoutError):
            tradier_execution_module._polling_get_orders(
                client=client,
                session=session,
                payload={"symbol": "SPY", "side": "buy", "quantity": 100, "type": "market"},
                timeout_sec=1,
            )

    def test_polling_match_order_by_symbol_side_qty(self) -> None:
        payload = {"symbol": "SPY", "side": "buy", "quantity": 100, "type": "market"}
        broker_order = {"symbol": "SPY", "side": "buy", "qty": 100, "type": "market"}
        assert tradier_execution_module._match_order(broker_order, payload) is True

    def test_polling_match_order_mismatch_symbol(self) -> None:
        payload = {"symbol": "SPY", "side": "buy", "quantity": 100, "type": "market"}
        broker_order = {"symbol": "QQQ", "side": "buy", "qty": 100, "type": "market"}
        assert tradier_execution_module._match_order(broker_order, payload) is False

    def test_execute_with_fallback_stream_success(self, monkeypatch) -> None:
        client = _FakeClient()
        session = _FakeSession()
        polling_mock = MagicMock(return_value={"order_id": "x", "method_used": "polling_get_orders"})
        monkeypatch.setattr(
            tradier_execution_module,
            "_stream_order_confirmation",
            lambda **kwargs: {"order_id": "ABC", "status": "ok", "method_used": "stream_confirmation"},
        )
        monkeypatch.setattr(tradier_execution_module, "_polling_get_orders", polling_mock)
        result = tradier_execution_module._execute_with_fallback(
            client=client,
            session=session,
            payload={"symbol": "SPY", "side": "buy", "quantity": 1, "type": "market"},
        )
        assert result["order_id"] == "ABC"
        polling_mock.assert_not_called()

    def test_execute_with_fallback_stream_fails_polling_succeeds(self, monkeypatch, caplog) -> None:
        client = _FakeClient()
        session = _FakeSession()

        def _fail_stream(**kwargs):
            raise TimeoutError("stream timeout")

        monkeypatch.setattr(tradier_execution_module, "_stream_order_confirmation", _fail_stream)
        monkeypatch.setattr(
            tradier_execution_module,
            "_polling_get_orders",
            lambda **kwargs: {"order_id": "POLL-1", "status": "accepted", "method_used": "polling_get_orders"},
        )
        result = tradier_execution_module._execute_with_fallback(
            client=client,
            session=session,
            payload={"symbol": "SPY", "side": "buy", "quantity": 1, "type": "market"},
            max_retries=3,
            timeout_sec=1,
        )
        assert result["order_id"] == "POLL-1"
        assert "falling back" in caplog.text.lower()

    def test_execute_with_fallback_all_fail(self, monkeypatch) -> None:
        client = _FakeClient()
        session = _FakeSession()
        monkeypatch.setattr(
            tradier_execution_module,
            "_stream_order_confirmation",
            lambda **kwargs: (_ for _ in ()).throw(TimeoutError("stream timeout")),
        )
        monkeypatch.setattr(
            tradier_execution_module,
            "_polling_get_orders",
            lambda **kwargs: (_ for _ in ()).throw(TimeoutError("polling timeout")),
        )
        with pytest.raises(RuntimeError, match="Failed to execute order with fallback strategies"):
            tradier_execution_module._execute_with_fallback(
                client=client,
                session=session,
                payload={"symbol": "SPY", "side": "buy", "quantity": 1, "type": "market"},
                max_retries=2,
                timeout_sec=1,
            )

    def test_route_order_to_tradier_guarantees_broker_order_id(self, monkeypatch) -> None:
        session = _FakeSession()
        client = _FakeClient()
        monkeypatch.setattr(tradier_execution_module, "resolve_account_session", lambda **kwargs: (client, session))
        monkeypatch.setattr(
            tradier_execution_module,
            "_execute_with_fallback",
            lambda **kwargs: {"order_id": "ABC123", "status": "accepted", "method_used": "stream_confirmation"},
        )
        monkeypatch.setattr(tradier_execution_module, "record_live_order_intent", lambda **kwargs: {"ok": True})
        result = tradier_execution_module.route_order_to_tradier(
            OrderRequest(
                symbol="SPY",
                side="buy",
                size=1,
                order_type="market",
                asset_class="equity",
                account_scope="paper",
            )
        )
        assert result["broker_order_ids_json"]["id"] == "ABC123"
        assert result["status"] == "sent"

    def test_route_order_to_tradier_raises_if_no_id(self, monkeypatch) -> None:
        session = _FakeSession()
        client = _FakeClient()
        monkeypatch.setattr(tradier_execution_module, "resolve_account_session", lambda **kwargs: (client, session))
        monkeypatch.setattr(
            tradier_execution_module,
            "_execute_with_fallback",
            lambda **kwargs: {"status": "accepted", "method_used": "stream_confirmation"},
        )
        monkeypatch.setattr(tradier_execution_module, "record_live_order_intent", lambda **kwargs: {"ok": True})
        with pytest.raises(ValueError, match="broker_order_id"):
            tradier_execution_module.route_order_to_tradier(
                OrderRequest(
                    symbol="SPY",
                    side="buy",
                    size=1,
                    order_type="market",
                    asset_class="equity",
                    account_scope="paper",
                )
            )
