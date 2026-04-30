"""Tests para execution.option_chain_cache."""
from __future__ import annotations

import time
from unittest.mock import MagicMock

import pytest

from atlas_code_quant.execution.option_chain_cache import OptionChainCache


def _mock_tradier():
    client = MagicMock()
    # get_option_chain is called as: tradier_client.get_option_chain(symbol, expiration, scope=scope)
    client.get_option_chain.return_value = {
        "options": {"option": [
            {"strike": 4500, "option_type": "call", "bid": 10, "ask": 11, "open_interest": 500, "volume": 200},
            {"strike": 4500, "option_type": "put",  "bid": 9,  "ask": 10, "open_interest": 400, "volume": 150},
        ]}
    }
    # get_expirations is called as: tradier_client.get_option_expirations(symbol, scope=scope)
    client.get_option_expirations.return_value = ["2024-01-19", "2024-02-16"]
    return client


class TestGetOrFetch:
    def test_first_call_fetches(self):
        cache = OptionChainCache(ttl_sec=60)
        client = _mock_tradier()
        result = cache.get_or_fetch("SPX", "2024-01-19", client, "paper")
        assert result is not None
        client.get_option_chain.assert_called_once()

    def test_second_call_uses_cache(self):
        cache = OptionChainCache(ttl_sec=60)
        client = _mock_tradier()
        cache.get_or_fetch("SPX", "2024-01-19", client, "paper")
        cache.get_or_fetch("SPX", "2024-01-19", client, "paper")
        # Should only have called tradier once
        assert client.get_option_chain.call_count == 1

    def test_expired_cache_refetches(self):
        cache = OptionChainCache(ttl_sec=0)  # TTL=0 → always expired
        client = _mock_tradier()
        cache.get_or_fetch("SPX", "2024-01-19", client, "paper")
        time.sleep(0.01)
        cache.get_or_fetch("SPX", "2024-01-19", client, "paper")
        assert client.get_option_chain.call_count >= 2

    def test_different_expirations_separate_cache(self):
        cache = OptionChainCache(ttl_sec=60)
        client = _mock_tradier()
        cache.get_or_fetch("SPX", "2024-01-19", client, "paper")
        cache.get_or_fetch("SPX", "2024-02-16", client, "paper")
        assert client.get_option_chain.call_count == 2

    def test_different_symbols_separate_cache(self):
        cache = OptionChainCache(ttl_sec=60)
        client = _mock_tradier()
        cache.get_or_fetch("SPX", "2024-01-19", client, "paper")
        cache.get_or_fetch("NDX", "2024-01-19", client, "paper")
        assert client.get_option_chain.call_count == 2


class TestGetExpirations:
    def test_returns_list(self):
        cache = OptionChainCache()
        client = _mock_tradier()
        result = cache.get_expirations("SPX", client, "paper")
        assert isinstance(result, list)

    def test_returns_cached_on_second_call(self):
        cache = OptionChainCache(ttl_sec=60)
        client = _mock_tradier()
        cache.get_expirations("SPX", client, "paper")
        cache.get_expirations("SPX", client, "paper")
        assert client.get_option_expirations.call_count == 1

    def test_tradier_exception_returns_empty(self):
        cache = OptionChainCache()
        client = MagicMock()
        client.get_option_expirations.side_effect = RuntimeError("connection error")
        result = cache.get_expirations("SPX", client, "paper")
        assert result == []


class TestInvalidate:
    def test_invalidate_forces_refetch(self):
        cache = OptionChainCache(ttl_sec=3600)
        client = _mock_tradier()
        cache.get_or_fetch("SPX", "2024-01-19", client, "paper")
        cache.invalidate("SPX")
        cache.get_or_fetch("SPX", "2024-01-19", client, "paper")
        assert client.get_option_chain.call_count == 2

    def test_invalidate_unknown_symbol_no_error(self):
        cache = OptionChainCache()
        cache.invalidate("UNKNOWN")  # should not raise


class TestStats:
    def test_stats_returns_dict(self):
        cache = OptionChainCache()
        assert isinstance(cache.stats(), dict)

    def test_stats_includes_total(self):
        cache = OptionChainCache()
        stats = cache.stats()
        assert "total" in stats

    def test_stats_empty_initially(self):
        cache = OptionChainCache()
        stats = cache.stats()
        assert stats.get("total", 0) == 0
