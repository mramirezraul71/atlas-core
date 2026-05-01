"""Tests para scanner.index_universe."""
from __future__ import annotations

from datetime import date

import pytest

from atlas_code_quant.scanner.index_universe import (
    INDEX_PROFILES,
    IndexProfile,
    get_index_profile,
    get_all_index_symbols,
    build_index_option_symbol,
    estimate_bpr,
)


class TestIndexProfiles:
    def test_not_empty(self):
        assert len(INDEX_PROFILES) > 0

    def test_spx_present(self):
        assert "SPX" in INDEX_PROFILES

    def test_ndx_present(self):
        assert "NDX" in INDEX_PROFILES

    def test_rut_present(self):
        assert "RUT" in INDEX_PROFILES

    def test_all_values_are_index_profiles(self):
        for v in INDEX_PROFILES.values():
            assert isinstance(v, IndexProfile)

    def test_all_no_pdt_true(self):
        for p in INDEX_PROFILES.values():
            assert p.no_pdt is True

    def test_spx_cash_settled(self):
        assert INDEX_PROFILES["SPX"].is_cash_settled is True

    def test_spx_european(self):
        assert INDEX_PROFILES["SPX"].is_european is True

    def test_multiplier_100(self):
        for p in INDEX_PROFILES.values():
            assert p.multiplier == 100


class TestGetAllIndexSymbols:
    def test_returns_list(self):
        assert isinstance(get_all_index_symbols(), list)

    def test_spx_in_list(self):
        assert "SPX" in get_all_index_symbols()

    def test_no_duplicates(self):
        syms = get_all_index_symbols()
        assert len(syms) == len(set(syms))


class TestGetIndexProfile:
    def test_spx_found(self):
        p = get_index_profile("SPX")
        assert p is not None
        assert p.root == "SPX"

    def test_case_insensitive(self):
        p = get_index_profile("spx")
        assert p is not None

    def test_unknown_returns_none(self):
        assert get_index_profile("UNKNOWN") is None

    def test_ndx_has_yfinance_ticker(self):
        p = get_index_profile("NDX")
        assert p is not None
        assert p.yfinance_ticker


class TestBuildIndexOptionSymbol:
    def test_basic_call(self):
        exp = date(2024, 1, 19)
        sym = build_index_option_symbol("SPX", exp, "C", 4500)
        assert "SPX" in sym
        assert "C" in sym

    def test_occ_format_length(self):
        exp = date(2024, 1, 19)
        sym = build_index_option_symbol("SPX", exp, "C", 4500)
        # "SPX" (3) + "240119" (6) + "C" (1) + "04500000" (8) = 18 chars
        assert len(sym) == 18

    def test_put_symbol(self):
        exp = date(2024, 1, 19)
        sym = build_index_option_symbol("NDX", exp, "P", 17000)
        assert "P" in sym

    def test_strike_zero_padded(self):
        exp = date(2024, 1, 19)
        sym = build_index_option_symbol("SPX", exp, "C", 4500)
        assert sym.endswith("04500000")


class TestEstimateBPR:
    def test_credit_spread_bpr_is_width_times_multiplier(self):
        # Credit spread BPR = width * multiplier
        bpr = estimate_bpr(width_points=10, multiplier=100, is_credit_spread=True)
        assert bpr == 1000.0

    def test_debit_spread_bpr_is_half(self):
        # Debit BPR = width * multiplier * 0.5
        bpr = estimate_bpr(width_points=10, multiplier=100, is_credit_spread=False)
        assert bpr == 500.0

    def test_positive_result(self):
        bpr = estimate_bpr(5, 100, True)
        assert bpr > 0

    def test_default_multiplier_100(self):
        bpr = estimate_bpr(width_points=5)
        assert bpr == 500.0  # 5 * 100 for credit spread
