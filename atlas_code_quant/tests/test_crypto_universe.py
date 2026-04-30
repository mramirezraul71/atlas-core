"""Tests para scanner.crypto_universe."""
from __future__ import annotations

import pytest

from atlas_code_quant.scanner.crypto_universe import (
    CRYPTO_SPOT_UNIVERSE,
    CryptoProfile,
    get_crypto_profile,
    get_all_crypto_symbols,
    get_crypto_by_exchange,
)


class TestCryptoSpotUniverse:
    def test_not_empty(self):
        assert len(CRYPTO_SPOT_UNIVERSE) > 0

    def test_btc_present(self):
        assert "BTC/USDT" in CRYPTO_SPOT_UNIVERSE

    def test_eth_present(self):
        assert "ETH/USDT" in CRYPTO_SPOT_UNIVERSE

    def test_all_values_are_crypto_profiles(self):
        for v in CRYPTO_SPOT_UNIVERSE.values():
            assert isinstance(v, CryptoProfile)

    def test_all_have_base_and_quote(self):
        for p in CRYPTO_SPOT_UNIVERSE.values():
            assert p.base and p.quote

    def test_no_duplicates(self):
        assert len(CRYPTO_SPOT_UNIVERSE) == len(set(CRYPTO_SPOT_UNIVERSE.keys()))


class TestGetAllCryptoSymbols:
    def test_returns_list(self):
        syms = get_all_crypto_symbols()
        assert isinstance(syms, list)

    def test_btc_in_list(self):
        assert "BTC/USDT" in get_all_crypto_symbols()

    def test_no_duplicates(self):
        syms = get_all_crypto_symbols()
        assert len(syms) == len(set(syms))


class TestGetCryptoProfile:
    def test_btc_found(self):
        p = get_crypto_profile("BTC/USDT")
        assert p is not None
        assert p.symbol == "BTC/USDT"

    def test_unknown_returns_none(self):
        assert get_crypto_profile("UNKNOWN/USDT") is None

    def test_sol_found(self):
        p = get_crypto_profile("SOL/USDT")
        assert p is not None

    def test_profile_has_yfinance_ticker(self):
        p = get_crypto_profile("BTC/USDT")
        assert p is not None
        assert p.yfinance_ticker


class TestGetCryptoByExchange:
    def test_returns_list(self):
        result = get_crypto_by_exchange("binance")
        assert isinstance(result, list)

    def test_binance_not_empty(self):
        assert len(get_crypto_by_exchange("binance")) > 0

    def test_btc_in_binance(self):
        result = get_crypto_by_exchange("binance")
        syms = [p.symbol if hasattr(p, "symbol") else p for p in result]
        assert "BTC/USDT" in syms

    def test_unknown_exchange_empty(self):
        result = get_crypto_by_exchange("unknown_exchange_xyz")
        assert result == []
