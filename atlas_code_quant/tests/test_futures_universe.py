"""Tests para scanner.futures_universe."""
from __future__ import annotations

import pytest

from atlas_code_quant.scanner.futures_universe import (
    FUTURES_UNIVERSE,
    FuturesProfile,
    get_futures_profile,
    get_all_futures_symbols,
    get_micro_futures,
    point_value,
)


class TestFuturesUniverse:
    def test_not_empty(self):
        assert len(FUTURES_UNIVERSE) > 0

    def test_es_present(self):
        assert "ES" in FUTURES_UNIVERSE

    def test_nq_present(self):
        assert "NQ" in FUTURES_UNIVERSE

    def test_all_values_are_futures_profiles(self):
        for v in FUTURES_UNIVERSE.values():
            assert isinstance(v, FuturesProfile)

    def test_multiplier_positive(self):
        for p in FUTURES_UNIVERSE.values():
            assert p.multiplier > 0

    def test_es_multiplier_50(self):
        assert FUTURES_UNIVERSE["ES"].multiplier == 50

    def test_no_duplicates(self):
        assert len(FUTURES_UNIVERSE) == len(set(FUTURES_UNIVERSE.keys()))


class TestGetAllFuturesSymbols:
    def test_returns_list(self):
        assert isinstance(get_all_futures_symbols(), list)

    def test_es_in_list(self):
        assert "ES" in get_all_futures_symbols()

    def test_no_duplicates(self):
        syms = get_all_futures_symbols()
        assert len(syms) == len(set(syms))


class TestGetFuturesProfile:
    def test_es_found(self):
        p = get_futures_profile("ES")
        assert p is not None

    def test_case_insensitive(self):
        p = get_futures_profile("es")
        assert p is not None

    def test_unknown_returns_none(self):
        assert get_futures_profile("UNKNOWN") is None

    def test_nq_has_yfinance_ticker(self):
        p = get_futures_profile("NQ")
        assert p is not None
        assert p.yfinance_ticker


class TestGetMicroFutures:
    def test_returns_list(self):
        assert isinstance(get_micro_futures(), list)

    def test_mes_in_micros(self):
        micros = get_micro_futures()
        # May return symbols or profiles
        syms = [p.symbol if hasattr(p, "symbol") else p for p in micros]
        assert "MES" in syms

    def test_micro_futures_not_empty(self):
        assert len(get_micro_futures()) > 0


class TestPointValue:
    def test_es_point_value(self):
        pv = point_value("ES")
        assert pv > 0

    def test_unknown_returns_zero_or_none(self):
        pv = point_value("UNKNOWN")
        assert pv == 0 or pv is None
