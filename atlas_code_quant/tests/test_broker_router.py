"""Tests para execution.broker_router."""
from __future__ import annotations

from unittest.mock import MagicMock, patch

import pytest

from atlas_code_quant.execution.broker_router import BrokerRouter
from atlas_code_quant.api.schemas import OrderRequest


def _make_order(symbol="AAPL", side="buy", size=1) -> OrderRequest:
    return OrderRequest(symbol=symbol, side=side, size=size, order_type="market")


class TestBrokerRouterInit:
    def test_default_mode_paper(self):
        router = BrokerRouter()
        assert router.mode == "paper"

    def test_custom_mode(self):
        router = BrokerRouter(mode="live")
        assert router.mode == "live"

    def test_mode_stripped_lowered(self):
        router = BrokerRouter(mode=" LIVE ")
        assert router.mode == "live"


class TestRouteEquity:
    @patch("atlas_code_quant.execution.broker_router.route_order_to_tradier")
    def test_equity_stock_routes_to_tradier(self, mock_tradier):
        mock_tradier.return_value = {"ok": True, "order_id": "123"}
        router = BrokerRouter()
        result = router.route(_make_order("AAPL"), asset_class="equity_stock")
        assert result["broker"] == "tradier"
        assert result["ok"] is True
        mock_tradier.assert_called_once()

    @patch("atlas_code_quant.execution.broker_router.route_order_to_tradier")
    def test_equity_etf_routes_to_tradier(self, mock_tradier):
        mock_tradier.return_value = {"ok": True, "order_id": "456"}
        router = BrokerRouter()
        result = router.route(_make_order("SPY"), asset_class="equity_etf")
        assert result["broker"] == "tradier"
        mock_tradier.assert_called_once()

    @patch("atlas_code_quant.execution.broker_router.route_order_to_tradier")
    def test_index_option_routes_to_tradier(self, mock_tradier):
        mock_tradier.return_value = {"ok": True, "order_id": "789"}
        router = BrokerRouter()
        result = router.route(_make_order("SPX"), asset_class="index_option")
        assert result["broker"] == "tradier"
        mock_tradier.assert_called_once()

    @patch("atlas_code_quant.execution.broker_router.route_order_to_tradier")
    def test_unknown_asset_class_falls_back_to_tradier(self, mock_tradier):
        mock_tradier.return_value = {"ok": True, "order_id": "000"}
        router = BrokerRouter()
        result = router.route(_make_order("XYZ"), asset_class="unknown_type")
        assert result["broker"] == "tradier"

    @patch("atlas_code_quant.execution.broker_router.route_order_to_tradier")
    def test_tradier_exception_returns_error_dict(self, mock_tradier):
        mock_tradier.side_effect = RuntimeError("timeout")
        router = BrokerRouter()
        result = router.route(_make_order("AAPL"), asset_class="equity_stock")
        assert result["ok"] is False
        assert result["broker"] == "tradier"
        assert "timeout" in result["error"]


class TestRouteCrypto:
    def test_crypto_paper_no_credentials_simulates(self):
        router = BrokerRouter(mode="paper")
        with patch.dict("os.environ", {"CCXT_API_KEY": "", "CCXT_SECRET": ""}):
            result = router.route(_make_order("BTC/USDT"), asset_class="crypto")
        assert result["ok"] is True
        assert result["broker"] == "crypto_simulated"
        assert result["status"] == "simulated"

    def test_crypto_live_no_credentials_simulates(self):
        router = BrokerRouter(mode="live")
        with patch.dict("os.environ", {"CCXT_API_KEY": "", "CCXT_SECRET": ""}):
            result = router.route(_make_order("ETH/USDT"), asset_class="crypto")
        assert result["ok"] is True
        assert result["status"] == "simulated"

    def test_crypto_simulate_includes_symbol(self):
        router = BrokerRouter(mode="paper")
        with patch.dict("os.environ", {"CCXT_API_KEY": "", "CCXT_SECRET": ""}):
            result = router.route(_make_order("BTC/USDT"), asset_class="crypto")
        assert result["symbol"] == "BTC/USDT"

    def test_crypto_simulate_includes_side(self):
        router = BrokerRouter(mode="paper")
        with patch.dict("os.environ", {"CCXT_API_KEY": "", "CCXT_SECRET": ""}):
            result = router.route(_make_order("BTC/USDT", side="sell"), asset_class="crypto")
        assert result["side"] == "sell"

    def test_crypto_simulate_order_id_format(self):
        router = BrokerRouter(mode="live")
        result = router._crypto_simulate(_make_order("BTC/USDT"))
        assert result["ok"] is True
        assert "CRYPTO-SIM-" in result["order_id"]


class TestRouteFutures:
    def test_future_returns_simulated(self):
        router = BrokerRouter()
        result = router.route(_make_order("ES"), asset_class="future")
        assert result["ok"] is True
        assert "simulated" in result["broker"]
        assert result["status"] == "simulated"

    def test_forex_returns_simulated(self):
        router = BrokerRouter()
        result = router.route(_make_order("EUR/USD"), asset_class="forex")
        assert result["ok"] is True
        assert "simulated" in result["broker"]

    def test_future_includes_phase_note(self):
        router = BrokerRouter()
        result = router.route(_make_order("NQ"), asset_class="future")
        assert "note" in result
        assert "5" in result["note"]  # "Fase 5"


class TestBrokerBadge:
    @patch("atlas_code_quant.execution.broker_router.route_order_to_tradier")
    def test_tradier_result_has_broker_field(self, mock_tradier):
        mock_tradier.return_value = {"ok": True}
        router = BrokerRouter()
        result = router.route(_make_order(), asset_class="equity_stock")
        assert "broker" in result

    def test_crypto_result_has_broker_field(self):
        router = BrokerRouter()
        with patch.dict("os.environ", {"CCXT_API_KEY": "", "CCXT_SECRET": ""}):
            result = router.route(_make_order("BTC/USDT"), asset_class="crypto")
        assert "broker" in result
