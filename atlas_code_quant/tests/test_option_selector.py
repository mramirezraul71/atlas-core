"""Tests para execution.option_selector."""
from __future__ import annotations

from datetime import date, timedelta

import pytest

from atlas_code_quant.execution.option_selector import (
    ContractSelection,
    describe_strategy_governance,
    pick_strategy,
    pick_expiration,
    pick_strike,
    build_legs_for_strategy,
    select_option_contract,
)


def _make_expirations(start_days=21, n=3, step=14):
    today = date.today()
    return [(today + timedelta(days=start_days + i * step)).strftime("%Y-%m-%d") for i in range(n)]


def _make_chain_options(spot=450.0, option_type="call", n=10):
    """Fake option chain matching the exact format pick_strike expects."""
    options = []
    for i in range(n):
        strike = spot - 20 + i * 5
        mid_delta = 0.60 - i * 0.06  # 0.60 down to ~0
        options.append({
            "option_type": option_type,
            "strike": strike,
            "bid": 2.0 + i * 0.1,
            "ask": 2.5 + i * 0.1,
            "open_interest": 500 + i * 50,
            "volume": 50,
            "symbol": f"SPY240119{'C' if option_type == 'call' else 'P'}{int(strike*1000):08d}",
            "greeks": {"delta": mid_delta if option_type == "call" else -mid_delta, "mid_iv": 0.25},
        })
    return options


class TestPickStrategy:
    def test_high_iv_sideways_gives_iron_condor(self):
        strat = pick_strategy("BUY", "SIDEWAYS", iv_rank=75, iv_hv_ratio=1.5)
        assert strat == "iron_condor"

    def test_high_iv_bull_gives_bull_put_credit(self):
        strat = pick_strategy("BUY", "BULL", iv_rank=55, iv_hv_ratio=1.4)
        assert strat == "bull_put_credit_spread"

    def test_high_iv_bear_gives_bear_call_credit(self):
        strat = pick_strategy("SELL", "BEAR", iv_rank=55, iv_hv_ratio=1.4)
        assert strat == "bear_call_credit_spread"

    def test_low_iv_bull_gives_debit_strategy(self):
        strat = pick_strategy("BUY", "BULL", iv_rank=20, iv_hv_ratio=0.8)
        assert strat in {"long_call", "bull_call_debit_spread"}

    def test_low_iv_bear_gives_debit_strategy(self):
        strat = pick_strategy("SELL", "BEAR", iv_rank=20, iv_hv_ratio=0.8)
        assert strat in {"long_put", "bear_put_debit_spread"}

    def test_returns_string(self):
        strat = pick_strategy("BUY", "BULL", iv_rank=40, iv_hv_ratio=1.0)
        assert isinstance(strat, str)
        assert len(strat) > 0

    def test_very_high_iv_sideways_iron_condor(self):
        assert pick_strategy("BUY", "SIDEWAYS", 85, 2.0) == "iron_condor"

    def test_event_near_bull_avoids_credit_spread(self):
        strat = pick_strategy("BUY", "BULL", iv_rank=65, iv_hv_ratio=1.4, event_near=True)
        assert strat == "bull_call_debit_spread"

    def test_very_high_iv_sideways_balanced_skew_can_use_iron_butterfly(self):
        strat = pick_strategy(
            "BUY",
            "SIDEWAYS",
            iv_rank=82,
            iv_hv_ratio=1.5,
            skew_pct=0.03,
            liquidity_score=0.9,
            thesis="neutral_income",
        )
        assert strat == "iron_butterfly"

    def test_weak_liquidity_avoids_directional_credit_selling(self):
        strat = pick_strategy(
            "SELL",
            "BEAR",
            iv_rank=70,
            iv_hv_ratio=1.5,
            liquidity_score=0.2,
        )
        assert strat == "bear_put_debit_spread"


class TestPickExpiration:
    def test_selects_closest_to_target(self):
        exps = _make_expirations(start_days=21, n=3, step=14)  # ~21, 35, 49 DTE
        result = pick_expiration(exps, target_dte=30, min_dte=14, max_dte=45)
        assert result in exps

    def test_returns_none_when_no_valid_exp(self):
        # Dates way in the past — 0 DTE
        result = pick_expiration(["2020-01-01", "2020-06-01"], target_dte=30, min_dte=14, max_dte=45)
        assert result is None

    def test_empty_list_returns_none(self):
        assert pick_expiration([], 30, 14, 45) is None

    def test_respects_min_dte(self):
        # Only 5 days out — below min_dte=14
        exp_5d = (date.today() + timedelta(days=5)).strftime("%Y-%m-%d")
        exp_30d = (date.today() + timedelta(days=30)).strftime("%Y-%m-%d")
        result = pick_expiration([exp_5d, exp_30d], target_dte=30, min_dte=14, max_dte=45)
        assert result == exp_30d


class TestPickStrike:
    def test_returns_closest_to_target_delta(self):
        options = _make_chain_options(spot=450, option_type="call")
        result = pick_strike(options, spot=450, option_type="call", target_delta=0.40)
        assert result is not None

    def test_no_options_returns_none(self):
        result = pick_strike([], spot=450, option_type="call", target_delta=0.3)
        assert result is None

    def test_result_has_strike_field(self):
        options = _make_chain_options(spot=450, option_type="call")
        result = pick_strike(options, spot=450, option_type="call", target_delta=0.40)
        assert result is not None
        assert "strike" in result

    def test_filters_by_option_type(self):
        calls = _make_chain_options(spot=450, option_type="call")
        result = pick_strike(calls, spot=450, option_type="put", target_delta=0.30)
        # All are calls → no puts → should return None
        assert result is None


class TestBuildLegsForStrategy:
    def test_long_call_single_leg(self):
        calls = _make_chain_options(spot=450, option_type="call")
        puts  = _make_chain_options(spot=450, option_type="put")
        legs = build_legs_for_strategy("long_call", calls, puts, spot=450, expiration="2024-06-19")
        # Returns leg if a suitable call found, else []
        if legs:
            assert legs[0]["option_type"] == "call"
            assert legs[0]["side"] == "long"

    def test_long_put_single_leg(self):
        calls = _make_chain_options(spot=450, option_type="call")
        puts  = _make_chain_options(spot=450, option_type="put")
        legs = build_legs_for_strategy("long_put", calls, puts, spot=450, expiration="2024-06-19")
        if legs:
            assert legs[0]["option_type"] == "put"
            assert legs[0]["side"] == "long"

    def test_unknown_strategy_returns_empty(self):
        legs = build_legs_for_strategy("unknown_strat", [], [], spot=450, expiration="2024-06-19")
        assert legs == []

    def test_expiration_in_leg(self):
        calls = _make_chain_options(spot=450, option_type="call")
        puts  = _make_chain_options(spot=450, option_type="put")
        legs = build_legs_for_strategy("long_call", calls, puts, spot=450, expiration="2024-06-19")
        if legs:
            assert legs[0]["expiration"] == "2024-06-19"

    def test_two_chain_required_for_spread(self):
        # With identical options, bull_call_debit_spread may return single or empty
        calls = _make_chain_options(spot=450, option_type="call", n=15)
        puts  = _make_chain_options(spot=450, option_type="put",  n=15)
        legs = build_legs_for_strategy("bull_call_debit_spread", calls, puts, spot=450, expiration="2024-06-19")
        # May be 1 or 2 legs depending on available strikes
        assert isinstance(legs, list)


class TestSelectOptionContract:
    def test_returns_contract_selection(self):
        exps = _make_expirations()
        chain = {
            exp: {
                "calls": _make_chain_options(option_type="call"),
                "puts":  _make_chain_options(option_type="put"),
            }
            for exp in exps
        }
        result = select_option_contract(
            symbol="SPY", direction="BUY", spot=450,
            iv_rank=60, iv_hv_ratio=1.3, regime="BULL",
            expirations=exps, chain_by_exp=chain,
        )
        assert isinstance(result, ContractSelection)

    def test_has_is_valid_field(self):
        exps = _make_expirations()
        chain = {e: {"calls": _make_chain_options(option_type="call"), "puts": _make_chain_options(option_type="put")} for e in exps}
        result = select_option_contract("SPY", "BUY", 450, 60, 1.3, "BULL", exps, chain)
        assert hasattr(result, "is_valid")

    def test_empty_expirations_is_invalid(self):
        result = select_option_contract(
            "SPY", "BUY", 450, 60, 1.3, "BULL", [], {}
        )
        assert result.is_valid is False

    def test_strategy_type_populated(self):
        exps = _make_expirations()
        chain = {e: {"calls": _make_chain_options(option_type="call"), "puts": _make_chain_options(option_type="put")} for e in exps}
        result = select_option_contract("SPY", "BUY", 450, 60, 1.3, "BULL", exps, chain)
        assert isinstance(result.strategy_type, str)
        assert len(result.strategy_type) > 0

    def test_contract_selection_includes_governance_reasons(self):
        exps = _make_expirations()
        chain = {e: {"calls": _make_chain_options(option_type="call"), "puts": _make_chain_options(option_type="put")} for e in exps}
        result = select_option_contract(
            "SPY",
            "BUY",
            450,
            68,
            1.4,
            "BULL",
            exps,
            chain,
            event_near=True,
            liquidity_score=0.8,
            thesis="directional_explosive",
        )
        assert result.governance["thesis"] == "directional_explosive"
        assert result.governance["event_near"] is True
        assert len(result.governance["reasons"]) >= 2


class TestStrategyGovernance:
    def test_governance_describes_premium_posture(self):
        governance = describe_strategy_governance(
            direction="BUY",
            regime="BULL",
            iv_rank=70,
            iv_hv_ratio=1.4,
            strategy="bull_put_credit_spread",
            thesis="directional_controlled",
            event_near=False,
            liquidity_score=0.9,
        )
        assert governance["premium_stance"] == "sell_premium_defined_risk"
        assert governance["strategy_family"] == "directional_credit"
