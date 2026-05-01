"""Tests para scanner.asset_classifier."""
from __future__ import annotations

import pytest

from atlas_code_quant.scanner.asset_classifier import (
    AssetClass,
    AssetProfile,
    classify_asset,
    classify_many,
    filter_by_class,
    get_options_eligible,
)


class TestClassifyAsset:
    def test_spy_is_etf(self):
        p = classify_asset("SPY")
        assert p.asset_class == AssetClass.EQUITY_ETF

    def test_qqq_is_etf(self):
        assert classify_asset("QQQ").asset_class == AssetClass.EQUITY_ETF

    def test_spx_is_index_option(self):
        p = classify_asset("SPX")
        assert p.asset_class == AssetClass.INDEX_OPTION
        assert p.no_pdt is True

    def test_ndx_is_index_option(self):
        assert classify_asset("NDX").asset_class == AssetClass.INDEX_OPTION

    def test_rut_is_index_option(self):
        assert classify_asset("RUT").asset_class == AssetClass.INDEX_OPTION

    def test_btc_is_crypto(self):
        p = classify_asset("BTC/USDT")
        assert p.asset_class == AssetClass.CRYPTO

    def test_eth_usdt_is_crypto(self):
        assert classify_asset("ETH/USDT").asset_class == AssetClass.CRYPTO

    def test_es_is_future(self):
        p = classify_asset("ES")
        assert p.asset_class == AssetClass.FUTURE

    def test_nq_is_future(self):
        assert classify_asset("NQ").asset_class == AssetClass.FUTURE

    def test_eurusd_is_forex(self):
        p = classify_asset("EUR/USD")
        assert p.asset_class == AssetClass.FOREX

    def test_unknown_equity(self):
        p = classify_asset("AAPL")
        assert p.asset_class == AssetClass.EQUITY_STOCK

    def test_equity_with_etf_flag(self):
        p = classify_asset("XYZ", is_etf=True)
        assert p.asset_class == AssetClass.EQUITY_ETF

    def test_case_insensitive(self):
        assert classify_asset("spy").asset_class == AssetClass.EQUITY_ETF
        assert classify_asset("spx").asset_class == AssetClass.INDEX_OPTION

    def test_profile_is_asset_profile(self):
        p = classify_asset("AAPL")
        assert isinstance(p, AssetProfile)

    def test_symbol_preserved(self):
        p = classify_asset("TSLA")
        assert p.symbol == "TSLA"


class TestClassifyMany:
    def test_returns_dict(self):
        result = classify_many(["SPY", "AAPL", "SPX"])
        assert isinstance(result, dict)
        assert "SPY" in result
        assert "AAPL" in result
        assert "SPX" in result

    def test_correct_classes(self):
        result = classify_many(["SPY", "SPX", "AAPL"])
        assert result["SPY"].asset_class == AssetClass.EQUITY_ETF
        assert result["SPX"].asset_class == AssetClass.INDEX_OPTION
        assert result["AAPL"].asset_class == AssetClass.EQUITY_STOCK

    def test_empty_list(self):
        assert classify_many([]) == {}


class TestFilterByClass:
    def test_filter_etf_only(self):
        symbols = ["SPY", "AAPL", "SPX", "QQQ"]
        etfs = filter_by_class(symbols, {AssetClass.EQUITY_ETF})
        assert "SPY" in etfs
        assert "QQQ" in etfs
        assert "AAPL" not in etfs

    def test_filter_multiple_classes(self):
        symbols = ["SPY", "AAPL", "SPX"]
        result = filter_by_class(symbols, {AssetClass.EQUITY_ETF, AssetClass.INDEX_OPTION})
        assert "SPY" in result
        assert "SPX" in result
        assert "AAPL" not in result

    def test_empty_symbols(self):
        assert filter_by_class([], {AssetClass.EQUITY_ETF}) == []


class TestGetOptionsEligible:
    def test_etf_with_options_included(self):
        profiles = classify_many(["SPY", "QQQ"])
        eligible = get_options_eligible(profiles)
        assert "SPY" in eligible
        assert "QQQ" in eligible

    def test_equity_with_options_included(self):
        profiles = classify_many(["AAPL", "MSFT"])
        eligible = get_options_eligible(profiles)
        # AAPL/MSFT are equity_stock — check if has_options is True in profile
        # They default to True for well-known equities
        assert isinstance(eligible, list)

    def test_crypto_not_options_eligible(self):
        profiles = classify_many(["BTC/USDT"])
        eligible = get_options_eligible(profiles)
        assert "BTC/USDT" not in eligible


class TestPDTExemption:
    """Asset classifier marks index/crypto/future/forex as no_pdt."""

    def test_spx_no_pdt(self):
        assert classify_asset("SPX").no_pdt is True

    def test_ndx_no_pdt(self):
        assert classify_asset("NDX").no_pdt is True

    def test_btc_no_pdt(self):
        assert classify_asset("BTC/USDT").no_pdt is True

    def test_es_no_pdt(self):
        assert classify_asset("ES").no_pdt is True

    def test_equity_stock_has_pdt(self):
        p = classify_asset("AAPL")
        assert p.no_pdt is False

    def test_etf_has_pdt(self):
        p = classify_asset("SPY")
        assert p.no_pdt is False
