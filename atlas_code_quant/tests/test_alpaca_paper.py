"""Tests unitarios para AlpacaPaperBroker.

Cubre:
  - is_configured: con y sin credenciales
  - health_check: OK, fallo de red, HTTP ≠ 200
  - place_order: éxito, rechazo HTTP, sin credenciales, error de red
  - cancel_order: OK, sin credenciales
  - get_account / get_positions: caminos felices y error
  - _tradier_error_is_fallback: keywords esperados
"""
from __future__ import annotations

from unittest.mock import MagicMock, patch

import pytest

from atlas_code_quant.execution.alpaca_paper import (
    AlpacaPaperBroker,
    _tradier_error_is_fallback,
    get_alpaca_paper_broker,
)


# ── Helpers ────────────────────────────────────────────────────────────────────

def make_broker(key="PKTEST", secret="STEST") -> AlpacaPaperBroker:
    return AlpacaPaperBroker(api_key=key, secret_key=secret)


def mock_response(status_code: int, json_data: dict):
    r = MagicMock()
    r.status_code = status_code
    r.content     = b"content"
    r.json.return_value = json_data
    return r


# ── TestIsConfigured ───────────────────────────────────────────────────────────

class TestIsConfigured:
    def test_configured_with_both_keys(self):
        b = make_broker()
        assert b.is_configured() is True

    def test_not_configured_empty_key(self):
        b = make_broker(key="", secret="SECRET")
        assert b.is_configured() is False

    def test_not_configured_empty_secret(self):
        b = make_broker(key="KEY", secret="")
        assert b.is_configured() is False

    def test_not_configured_both_empty(self):
        b = AlpacaPaperBroker()
        assert b.is_configured() is False


# ── TestHealthCheck ────────────────────────────────────────────────────────────

class TestHealthCheck:
    def test_health_ok(self):
        b = make_broker()
        with patch("requests.get", return_value=mock_response(200, {"status": "ACTIVE"})):
            assert b.health_check(force=True) is True

    def test_health_http_error(self):
        b = make_broker()
        with patch("requests.get", return_value=mock_response(403, {})):
            assert b.health_check(force=True) is False

    def test_health_connection_error(self):
        b = make_broker()
        with patch("requests.get", side_effect=ConnectionError("timeout")):
            assert b.health_check(force=True) is False

    def test_health_caches_result(self):
        b = make_broker()
        with patch("requests.get", return_value=mock_response(200, {})) as mock_get:
            b.health_check(force=True)
            b.health_check()          # segunda llamada sin force → no llama de nuevo
        mock_get.assert_called_once()


# ── TestPlaceOrder ─────────────────────────────────────────────────────────────

class TestPlaceOrder:
    def test_buy_accepted(self):
        b = make_broker()
        resp_data = {
            "id": "abc-123",
            "status": "accepted",
            "filled_avg_price": None,
        }
        with patch("requests.post", return_value=mock_response(201, resp_data)):
            r = b.place_order("AAPL", side="buy", qty=10)
        assert r.status   == "accepted"
        assert r.order_id == "abc-123"
        assert r.broker   == "alpaca_paper"

    def test_sell_accepted(self):
        b = make_broker()
        resp_data = {"id": "sell-456", "status": "accepted", "filled_avg_price": "195.5"}
        with patch("requests.post", return_value=mock_response(200, resp_data)):
            r = b.place_order("TSLA", side="sell", qty=5)
        assert r.status     == "accepted"
        assert r.fill_price == 195.5

    def test_order_rejected_http(self):
        b = make_broker()
        with patch("requests.post", return_value=mock_response(422, {"message": "insufficient qty"})):
            r = b.place_order("AAPL", side="buy", qty=0)
        assert r.status == "rejected"
        assert "insufficient" in r.error

    def test_no_credentials_returns_error(self):
        b = AlpacaPaperBroker()
        r = b.place_order("AAPL", side="buy", qty=10)
        assert r.status == "error"
        assert "credentials" in r.error.lower()

    def test_network_exception(self):
        b = make_broker()
        with patch("requests.post", side_effect=ConnectionError("network down")):
            r = b.place_order("NVDA", side="buy", qty=3)
        assert r.status == "error"
        assert "network" in r.error.lower()

    def test_limit_order_includes_price(self):
        b = make_broker()
        captured = {}
        def fake_post(url, json=None, headers=None, timeout=None):
            captured.update(json or {})
            return mock_response(201, {"id": "lim-1", "status": "accepted", "filled_avg_price": None})
        with patch("requests.post", side_effect=fake_post):
            b.place_order("AAPL", side="buy", qty=1, order_type="limit", limit_price=150.0)
        assert captured.get("limit_price") == "150.0"
        assert captured.get("type") == "limit"


# ── TestCancelOrder ────────────────────────────────────────────────────────────

class TestCancelOrder:
    def test_cancel_ok(self):
        b = make_broker()
        with patch("requests.delete", return_value=mock_response(204, {})):
            assert b.cancel_order("order-xyz") is True

    def test_cancel_not_found(self):
        b = make_broker()
        with patch("requests.delete", return_value=mock_response(404, {})):
            assert b.cancel_order("bad-id") is False

    def test_cancel_no_credentials(self):
        b = AlpacaPaperBroker()
        assert b.cancel_order("any") is False

    def test_cancel_empty_id(self):
        b = make_broker()
        assert b.cancel_order("") is False


# ── TestGetAccount ─────────────────────────────────────────────────────────────

class TestGetAccount:
    def test_returns_account_data(self):
        b = make_broker()
        data = {"equity": "10000.00", "currency": "USD"}
        with patch("requests.get", return_value=mock_response(200, data)):
            acc = b.get_account()
        assert acc["equity"] == "10000.00"

    def test_returns_empty_on_error(self):
        b = make_broker()
        with patch("requests.get", return_value=mock_response(401, {})):
            assert b.get_account() == {}

    def test_returns_empty_no_credentials(self):
        b = AlpacaPaperBroker()
        assert b.get_account() == {}


# ── TestGetPositions ───────────────────────────────────────────────────────────

class TestGetPositions:
    def test_returns_list(self):
        b = make_broker()
        positions = [{"symbol": "AAPL", "qty": "10"}, {"symbol": "TSLA", "qty": "5"}]
        with patch("requests.get", return_value=mock_response(200, positions)):
            result = b.get_positions()
        assert len(result) == 2
        assert result[0]["symbol"] == "AAPL"

    def test_returns_empty_on_failure(self):
        b = make_broker()
        with patch("requests.get", side_effect=ConnectionError):
            assert b.get_positions() == []


# ── TestTradierFallbackDetection ───────────────────────────────────────────────

class TestTradierFallbackDetection:
    @pytest.mark.parametrize("msg", [
        "504 Gateway Time-out",
        "503 Service Unavailable",
        "ConnectionError: sandbox.tradier.com",
        "Request timed out",
        "connection error occurred",
        "gateway timeout",
    ])
    def test_fallback_keywords(self, msg):
        exc = Exception(msg)
        assert _tradier_error_is_fallback(exc) is True

    @pytest.mark.parametrize("msg", [
        "401 Unauthorized",
        "invalid token",
        "PDT limit reached",
        "order blocked",
    ])
    def test_non_fallback_errors(self, msg):
        exc = Exception(msg)
        assert _tradier_error_is_fallback(exc) is False


# ── TestSingleton ──────────────────────────────────────────────────────────────

class TestSingleton:
    def test_same_instance(self):
        import atlas_code_quant.execution.alpaca_paper as mod
        mod._broker_instance = None   # reset
        b1 = get_alpaca_paper_broker()
        b2 = get_alpaca_paper_broker()
        assert b1 is b2

    def teardown_method(self, _):
        import atlas_code_quant.execution.alpaca_paper as mod
        mod._broker_instance = None
