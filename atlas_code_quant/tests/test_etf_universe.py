"""Tests para scanner.etf_universe."""
from __future__ import annotations

import pytest

from atlas_code_quant.scanner.etf_universe import (
    ETF_OPTIONS_UNIVERSE,
    ETFProfile,
    get_etf_profile,
    get_all_etf_symbols,
    get_liquid_etfs,
)


class TestETFOptionsUniverse:
    def test_not_empty(self):
        assert len(ETF_OPTIONS_UNIVERSE) > 0

    def test_spy_present(self):
        assert "SPY" in ETF_OPTIONS_UNIVERSE

    def test_qqq_present(self):
        assert "QQQ" in ETF_OPTIONS_UNIVERSE

    def test_all_values_are_etf_profiles(self):
        for v in ETF_OPTIONS_UNIVERSE.values():
            assert isinstance(v, ETFProfile)

    def test_all_have_symbol(self):
        for symbol, profile in ETF_OPTIONS_UNIVERSE.items():
            assert profile.symbol == symbol

    def test_no_duplicates(self):
        assert len(ETF_OPTIONS_UNIVERSE) == len(set(ETF_OPTIONS_UNIVERSE.keys()))


class TestGetAllETFSymbols:
    def test_returns_list(self):
        syms = get_all_etf_symbols()
        assert isinstance(syms, list)

    def test_spy_in_result(self):
        assert "SPY" in get_all_etf_symbols()

    def test_no_duplicates(self):
        syms = get_all_etf_symbols()
        assert len(syms) == len(set(syms))


class TestGetETFProfile:
    def test_spy_found(self):
        p = get_etf_profile("SPY")
        assert p is not None
        assert p.symbol == "SPY"

    def test_case_insensitive(self):
        p = get_etf_profile("spy")
        assert p is not None

    def test_unknown_returns_none(self):
        assert get_etf_profile("NOTANETF") is None

    def test_qqq_has_options_fields(self):
        p = get_etf_profile("QQQ")
        assert p is not None
        assert isinstance(p.has_0dte, bool)


class TestGetLiquidETFs:
    def test_returns_list(self):
        result = get_liquid_etfs()
        assert isinstance(result, list)

    def test_spy_is_liquid(self):
        liquid = get_liquid_etfs()
        # May return strings or profiles — handle both
        symbols = [p.symbol if hasattr(p, "symbol") else p for p in liquid]
        assert "SPY" in symbols
