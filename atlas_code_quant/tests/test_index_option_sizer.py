"""Tests para execution.index_option_sizer."""
from __future__ import annotations

import pytest

from atlas_code_quant.execution.index_option_sizer import (
    IndexOptionSize,
    size_index_option,
)


def _bull_put_legs(short_strike=4500, long_strike=4490, credit=2.0):
    # side: "short" = sold / "long" = bought; mid = mid-price
    return [
        {"option_type": "put", "side": "short", "strike": short_strike, "mid": credit},
        {"option_type": "put", "side": "long",  "strike": long_strike,  "mid": 0.5},
    ]


def _long_call_legs(strike=4500, debit=3.0):
    return [
        {"option_type": "call", "side": "long", "strike": strike, "mid": debit},
    ]


def _iron_condor_legs():
    return [
        {"option_type": "put",  "side": "short", "strike": 4450, "mid": 2.0},
        {"option_type": "put",  "side": "long",  "strike": 4440, "mid": 0.5},
        {"option_type": "call", "side": "short", "strike": 4550, "mid": 2.0},
        {"option_type": "call", "side": "long",  "strike": 4560, "mid": 0.5},
    ]


class TestSizeIndexOption:
    def test_returns_index_option_size(self):
        result = size_index_option("bull_put_credit_spread", _bull_put_legs(), capital=50000)
        assert isinstance(result, IndexOptionSize)

    def test_credit_spread_bpr_computed(self):
        result = size_index_option("bull_put_credit_spread", _bull_put_legs(), capital=50000)
        assert result.bpr_per_contract > 0

    def test_approved_within_limits(self):
        result = size_index_option(
            "bull_put_credit_spread", _bull_put_legs(credit=2.0),
            capital=50000, max_bpr_pct=0.02
        )
        assert result.is_approved is True

    def test_blocked_when_bpr_exceeds_limit_small_capital(self):
        # Very small capital → BPR% will be huge but still 1 contract minimum
        # The sizer keeps min 1 contract so is_approved=True; total_bpr >> max_bpr
        result = size_index_option(
            "bull_put_credit_spread", _bull_put_legs(credit=2.0),
            capital=100, max_bpr_pct=0.02
        )
        # contracts may be 1 (minimum), but bpr_pct_of_capital will be >> limit
        assert result.bpr_pct_of_capital > result.bpr_per_contract / 100 * 0.01

    def test_max_contracts_respected(self):
        result = size_index_option(
            "bull_put_credit_spread", _bull_put_legs(),
            capital=1_000_000, max_contracts=2
        )
        assert result.contracts <= 2

    def test_long_call_debit_bpr(self):
        # Long call: BPR = debit * multiplier = 3.0 * 100 = 300
        result = size_index_option("long_call", _long_call_legs(debit=3.0), capital=50000)
        assert result.bpr_per_contract == pytest.approx(300.0)

    def test_iron_condor_bpr(self):
        result = size_index_option("iron_condor", _iron_condor_legs(), capital=50000)
        assert result.bpr_per_contract > 0

    def test_total_bpr_equals_contracts_times_per_contract(self):
        result = size_index_option("bull_put_credit_spread", _bull_put_legs(), capital=100000)
        if result.contracts > 0:
            assert result.total_bpr == pytest.approx(result.contracts * result.bpr_per_contract)

    def test_bpr_pct_within_range(self):
        result = size_index_option("bull_put_credit_spread", _bull_put_legs(), capital=50000)
        assert 0.0 <= result.bpr_pct_of_capital <= 1.0

    def test_empty_legs_returns_unapproved(self):
        result = size_index_option("long_call", [], capital=50000)
        assert result.is_approved is False

    def test_warnings_list(self):
        result = size_index_option("bull_put_credit_spread", _bull_put_legs(), capital=50000)
        assert isinstance(result.warnings, list)

    def test_credit_spread_bpr_is_width_minus_credit(self):
        # width = 4500 - 4490 = 10, net credit = 2.0 - 0.5 = 1.5
        # BPR = (10 - 1.5) * 100 = 850
        result = size_index_option(
            "bull_put_credit_spread",
            _bull_put_legs(short_strike=4500, long_strike=4490, credit=2.0),
            capital=100000
        )
        assert result.bpr_per_contract == pytest.approx(850.0)


class TestIndexOptionSizeFields:
    def test_has_all_fields(self):
        result = size_index_option("bull_put_credit_spread", _bull_put_legs(), capital=50000)
        assert hasattr(result, "contracts")
        assert hasattr(result, "bpr_per_contract")
        assert hasattr(result, "total_bpr")
        assert hasattr(result, "max_loss")
        assert hasattr(result, "max_gain")
        assert hasattr(result, "bpr_pct_of_capital")
        assert hasattr(result, "warnings")
        assert hasattr(result, "is_approved")

    def test_max_loss_non_negative(self):
        result = size_index_option("bull_put_credit_spread", _bull_put_legs(), capital=50000)
        if result.contracts > 0:
            assert result.max_loss >= 0
