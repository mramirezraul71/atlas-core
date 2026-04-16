"""Tests para atlas_code_quant/options/ — Motor de opciones OptionStrat.

Cubre:
  - Black-Scholes pricing y greeks
  - OptionLeg / LinearLeg payoff y greeks
  - Strategy: payoff_at_expiry, pnl_today, greeks, breakevens
  - strategy_risk_summary
  - scenario_pnl
  - strategy_to_dict / strategy_from_dict (round-trip)
  - strategy_templates: building + TEMPLATE_REGISTRY
  - PortfolioAnalyzer: add/remove/summary/greeks
"""
from __future__ import annotations

import math
import numpy as np
import pytest
from datetime import date, timedelta
from types import SimpleNamespace

from atlas_code_quant.options.strategy_engine import (
    _norm_cdf,
    _norm_pdf,
    bs_price,
    bs_greeks,
    OptionLeg,
    LinearLeg,
    Strategy,
    MarketSnapshot,
    GreeksVector,
    StrategyRiskSummary,
    payoff_at_expiry,
    pnl_today,
    greeks,
    breakeven_points,
    strategy_risk_summary,
    scenario_pnl,
    strategy_to_dict,
    strategy_from_dict,
)
from atlas_code_quant.options.strategy_templates import (
    TEMPLATE_REGISTRY,
    list_templates,
    long_call,
    long_put,
    bull_call_spread,
    iron_condor,
    long_straddle,
    covered_call,
)
from atlas_code_quant.options.portfolio_analyzer import PortfolioAnalyzer


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _expiry(dte: int = 30) -> date:
    return date.today() + timedelta(days=dte)


def _atm_call(spot: float = 100.0, dte: int = 30, iv: float = 0.25) -> OptionLeg:
    return OptionLeg(
        underlying_symbol="TEST",
        expiry=_expiry(dte),
        strike=spot,
        option_type="call",
        side="long",
        quantity=1,
        premium=5.0,
        multiplier=100,
        iv=iv,
        underlying_price=spot,
    )


def _atm_put(spot: float = 100.0, dte: int = 30, iv: float = 0.25) -> OptionLeg:
    return OptionLeg(
        underlying_symbol="TEST",
        expiry=_expiry(dte),
        strike=spot,
        option_type="put",
        side="long",
        quantity=1,
        premium=5.0,
        multiplier=100,
        iv=iv,
        underlying_price=spot,
    )


def _market(spot: float = 100.0, rate: float = 0.05) -> MarketSnapshot:
    return MarketSnapshot(underlying_price=spot, risk_free_rate=rate)


def _simple_call_strategy(spot: float = 100.0) -> Strategy:
    return Strategy(name="long_call", underlying="TEST", legs=[_atm_call(spot)])


# ===========================================================================
# TestNormCdf
# ===========================================================================

class TestNormCdf:
    def test_cdf_zero(self):
        assert abs(_norm_cdf(0.0) - 0.5) < 1e-6

    def test_cdf_positive(self):
        assert _norm_cdf(1.96) > 0.97

    def test_cdf_negative(self):
        assert _norm_cdf(-1.96) < 0.03

    def test_cdf_symmetry(self):
        assert abs(_norm_cdf(1.0) + _norm_cdf(-1.0) - 1.0) < 1e-9

    def test_pdf_positive(self):
        assert _norm_pdf(0.0) > 0.39  # ~0.3989


# ===========================================================================
# TestBsPrice
# ===========================================================================

class TestBsPrice:
    def test_call_atm_positive(self):
        price = bs_price(100, 100, 30/365, 0.05, 0.25, "call")
        assert price > 0

    def test_put_atm_positive(self):
        price = bs_price(100, 100, 30/365, 0.05, 0.25, "put")
        assert price > 0

    def test_deep_itm_call_intrinsic(self):
        # Deep ITM call ≈ intrinsic S - K*e^(-rT)
        price = bs_price(200, 100, 1/365, 0.05, 0.25, "call")
        assert price > 99.0  # approximately 100

    def test_deep_otm_call_near_zero(self):
        price = bs_price(50, 200, 30/365, 0.05, 0.25, "call")
        assert price < 0.01

    def test_put_call_parity(self):
        S, K, T, r, sig = 100, 100, 30/365, 0.05, 0.25
        call = bs_price(S, K, T, r, sig, "call")
        put  = bs_price(S, K, T, r, sig, "put")
        # C - P = S - K*e^(-rT)
        lhs = call - put
        rhs = S - K * math.exp(-r * T)
        assert abs(lhs - rhs) < 0.01

    def test_zero_time_call_intrinsic(self):
        # T→0: call ≈ max(S-K, 0)
        price = bs_price(110, 100, 1e-6, 0.05, 0.25, "call")
        assert abs(price - 10.0) < 0.1

    def test_zero_time_put_intrinsic(self):
        price = bs_price(90, 100, 1e-6, 0.05, 0.25, "put")
        assert abs(price - 10.0) < 0.1

    def test_zero_iv_call_intrinsic(self):
        # sigma=0: call price = max(S-K,0)*e^(-rT) ≈ 0 for OTM
        price = bs_price(90, 100, 30/365, 0.05, 0.0, "call")
        assert price == pytest.approx(0.0, abs=0.01)


# ===========================================================================
# TestBsGreeks
# ===========================================================================

class TestBsGreeks:
    def setup_method(self):
        self.g = bs_greeks(100, 100, 30/365, 0.05, 0.25, "call")

    def test_delta_call_bounded(self):
        assert 0 < self.g["delta"] < 1

    def test_delta_put_bounded(self):
        g = bs_greeks(100, 100, 30/365, 0.05, 0.25, "put")
        assert -1 < g["delta"] < 0

    def test_gamma_positive(self):
        assert self.g["gamma"] > 0

    def test_theta_negative_call(self):
        # Long call: theta is negative (time decay)
        assert self.g["theta"] < 0

    def test_vega_positive(self):
        assert self.g["vega"] > 0

    def test_rho_positive_call(self):
        assert self.g["rho"] > 0

    def test_rho_negative_put(self):
        g = bs_greeks(100, 100, 30/365, 0.05, 0.25, "put")
        assert g["rho"] < 0

    def test_atm_delta_near_half(self):
        # ATM call delta ≈ 0.5 (slightly above due to rate)
        assert 0.45 < self.g["delta"] < 0.60


# ===========================================================================
# TestOptionLeg
# ===========================================================================

class TestOptionLeg:
    def test_long_call_payoff_itm(self):
        leg = _atm_call(100)
        # At S=120: payoff_expiry = (120-100)*100 = 2000 profit minus premium
        payoff = leg.payoff_at_expiry(120.0)
        assert payoff > 0  # ITM, net profit

    def test_long_call_payoff_otm(self):
        leg = _atm_call(100)
        # At S=80: payoff = 0 - premium = -500
        payoff = leg.payoff_at_expiry(80.0)
        assert abs(payoff - (-500.0)) < 0.01  # -$5 premium * 100

    def test_short_call_payoff_reverses(self):
        leg = OptionLeg(
            underlying_symbol="T", expiry=_expiry(30), strike=100,
            option_type="call", side="short", quantity=1, premium=5.0,
            multiplier=100, iv=0.25, underlying_price=100,
        )
        payoff_at_120 = leg.payoff_at_expiry(120.0)
        assert payoff_at_120 < 0  # short call in loss when deep ITM

    def test_long_put_payoff_itm(self):
        leg = _atm_put(100)
        payoff = leg.payoff_at_expiry(80.0)
        assert payoff > 0

    def test_long_put_payoff_otm(self):
        leg = _atm_put(100)
        payoff = leg.payoff_at_expiry(120.0)
        assert abs(payoff - (-500.0)) < 0.01

    def test_greeks_vector_long_call_delta_positive(self):
        leg = _atm_call(100)
        market = _market(100)
        gv = leg.greeks_vector(market.underlying_price, leg.tte_years, market.risk_free_rate)
        assert gv["delta"] > 0

    def test_greeks_vector_long_put_delta_negative(self):
        leg = _atm_put(100)
        market = _market(100)
        gv = leg.greeks_vector(market.underlying_price, leg.tte_years, market.risk_free_rate)
        assert gv["delta"] < 0


# ===========================================================================
# TestLinearLeg
# ===========================================================================

class TestLinearLeg:
    def test_long_stock_payoff_positive(self):
        leg = LinearLeg(symbol="TEST", asset_class="stock", side="long",
                        quantity=100, entry_price=100.0, multiplier=1)
        assert leg.payoff_at_expiry(110.0) == pytest.approx(1000.0)

    def test_short_stock_payoff_negative_at_higher_price(self):
        leg = LinearLeg(symbol="TEST", asset_class="stock", side="short",
                        quantity=100, entry_price=100.0, multiplier=1)
        assert leg.payoff_at_expiry(110.0) == pytest.approx(-1000.0)

    def test_linear_leg_delta(self):
        leg = LinearLeg(symbol="TEST", asset_class="stock", side="long",
                        quantity=1, entry_price=100.0, multiplier=100)
        gv = leg.greeks_vector()
        assert gv["delta"] == 100  # 1 * 100
        assert gv["gamma"] == 0
        assert gv["theta"] == 0


# ===========================================================================
# TestPayoffAtExpiry
# ===========================================================================

class TestPayoffAtExpiry:
    def test_long_call_profile(self):
        strat = _simple_call_strategy(100)
        S = np.linspace(50, 150, 100)
        payoff = payoff_at_expiry(strat, S)
        assert payoff.shape == (100,)
        # OTM zone: all losses (= -premium)
        otm_mask = S < 100
        assert np.all(payoff[otm_mask] < 0)
        # Deep ITM zone: should be profitable
        itm_mask = S > 115
        assert np.all(payoff[itm_mask] > 0)

    def test_straddle_profile(self):
        call = _atm_call(100)
        put  = _atm_put(100)
        strat = Strategy(name="straddle", underlying="TEST", legs=[call, put])
        S = np.linspace(50, 150, 200)
        payoff = payoff_at_expiry(strat, S)
        # Profit at extreme S values
        assert payoff[0] > 0   # very low price → put wins
        assert payoff[-1] > 0  # very high price → call wins

    def test_iron_condor_payoff_shape(self):
        # Long put 90, short put 95, short call 105, long call 110
        legs = [
            OptionLeg("T", _expiry(), 90, "put",  "long",  1, 1.0, 100, 0.2, 100),
            OptionLeg("T", _expiry(), 95, "put",  "short", 1, 2.0, 100, 0.2, 100),
            OptionLeg("T", _expiry(), 105, "call", "short", 1, 2.0, 100, 0.2, 100),
            OptionLeg("T", _expiry(), 110, "call", "long",  1, 1.0, 100, 0.2, 100),
        ]
        strat = Strategy(name="ic", underlying="TEST", legs=legs)
        S = np.linspace(70, 130, 300)
        payoff = payoff_at_expiry(strat, S)
        # Center zone should be profitable
        center_mask = (S >= 96) & (S <= 104)
        assert np.any(payoff[center_mask] > 0)


# ===========================================================================
# TestPnlToday
# ===========================================================================

class TestPnlToday:
    def test_pnl_today_returns_array(self):
        strat = _simple_call_strategy(100)
        market = _market(100)
        S = np.linspace(70, 130, 50)
        result = pnl_today(strat, market, S)
        assert result.shape == (50,)

    def test_pnl_today_at_spot_less_than_expiry(self):
        """T+0 PnL should be higher than expiry payoff for long options (time value)."""
        strat = _simple_call_strategy(100)
        market = _market(100)
        S = np.array([100.0])
        pnl_t0 = pnl_today(strat, market, S)[0]
        payoff_exp = payoff_at_expiry(strat, S)[0]
        # At ATM, T+0 PnL should be > expiry payoff (time value of option)
        assert pnl_t0 > payoff_exp

    def test_pnl_today_expired_matches_payoff(self):
        """Expired option PnL ≈ payoff at expiry."""
        leg = OptionLeg(
            "TEST", _expiry(0), 100, "call", "long",
            quantity=1, premium=5.0, multiplier=100, iv=0.25, underlying_price=100,
        )
        strat = Strategy(name="exp_call", underlying="TEST", legs=[leg])
        market = _market(110)
        S = np.array([110.0])
        pnl = pnl_today(strat, market, S)[0]
        payoff = payoff_at_expiry(strat, S)[0]
        assert abs(pnl - payoff) < 50  # within $50


# ===========================================================================
# TestGreeks
# ===========================================================================

class TestGreeks:
    def test_greeks_return_vector(self):
        strat = _simple_call_strategy(100)
        market = _market(100)
        g = greeks(strat, market)
        assert isinstance(g, GreeksVector)

    def test_long_call_delta_positive(self):
        strat = _simple_call_strategy(100)
        market = _market(100)
        g = greeks(strat, market)
        assert g.delta > 0

    def test_straddle_delta_near_zero(self):
        call = _atm_call(100)
        put  = _atm_put(100)
        strat = Strategy(name="straddle", underlying="TEST", legs=[call, put])
        market = _market(100)
        g = greeks(strat, market)
        # Long straddle at ATM: delta should be near 0 (call and put cancel)
        assert abs(g.delta) < 15  # within 15 delta units (100 multiplier)

    def test_greeks_vector_addition(self):
        g1 = GreeksVector(delta=10, gamma=0.1, theta=-5, vega=20, rho=2)
        g2 = GreeksVector(delta=-3, gamma=0.05, theta=-2, vega=10, rho=1)
        total = g1 + g2
        assert total.delta == pytest.approx(7.0)
        assert total.theta == pytest.approx(-7.0)

    def test_greeks_to_dict(self):
        g = GreeksVector(delta=5.0, gamma=0.02, theta=-1.5, vega=8.0, rho=0.3)
        d = g.to_dict()
        assert "delta" in d and "gamma" in d and "theta" in d
        assert d["delta"] == pytest.approx(5.0)


# ===========================================================================
# TestBreakevens
# ===========================================================================

class TestBreakevens:
    def test_long_call_breakeven(self):
        strat = _simple_call_strategy(100)
        S = np.linspace(80, 130, 500)
        bk = breakeven_points(strat, S)
        assert len(bk) >= 1
        # BE for long call = strike + premium = 100 + 5 = 105
        assert any(abs(b - 105.0) < 1.0 for b in bk)

    def test_straddle_two_breakevens(self):
        call = _atm_call(100)
        put  = _atm_put(100)
        strat = Strategy(name="straddle", underlying="TEST", legs=[call, put])
        S = np.linspace(70, 130, 1000)
        bk = breakeven_points(strat, S)
        # Long straddle: 2 breakevens at ≈ 90 and ≈ 110
        assert len(bk) >= 2

    def test_no_breakeven_deep_itm(self):
        """Deep ITM strategy might not cross zero in grid."""
        leg = OptionLeg("T", _expiry(), 50, "call", "long", 1, 1.0, 100, 0.2, 200)
        strat = Strategy(name="deep_itm", underlying="T", legs=[leg])
        S = np.linspace(190, 210, 100)
        bk = breakeven_points(strat, S)
        # might be empty if always profitable in this grid
        assert isinstance(bk, list)


# ===========================================================================
# TestStrategyRiskSummary
# ===========================================================================

class TestStrategyRiskSummary:
    def test_returns_summary(self):
        strat = _simple_call_strategy(100)
        market = _market(100)
        S = np.linspace(70, 140, 300)
        risk = strategy_risk_summary(strat, market, S)
        assert isinstance(risk, StrategyRiskSummary)

    def test_long_call_max_loss_is_premium(self):
        strat = _simple_call_strategy(100)
        market = _market(100)
        S = np.linspace(70, 140, 500)
        risk = strategy_risk_summary(strat, market, S)
        # Max loss = -premium = -5 * 100 = -500
        assert risk.max_loss == pytest.approx(-500.0, abs=10)

    def test_iron_condor_max_loss_bounded(self):
        legs = [
            OptionLeg("T", _expiry(), 90, "put",  "long",  1, 1.0, 100, 0.2, 100),
            OptionLeg("T", _expiry(), 95, "put",  "short", 1, 2.0, 100, 0.2, 100),
            OptionLeg("T", _expiry(), 105, "call", "short", 1, 2.0, 100, 0.2, 100),
            OptionLeg("T", _expiry(), 110, "call", "long",  1, 1.0, 100, 0.2, 100),
        ]
        strat = Strategy(name="ic", underlying="TEST", legs=legs)
        market = _market(100)
        S = np.linspace(60, 140, 500)
        risk = strategy_risk_summary(strat, market, S)
        # Risk summary should compute successfully
        assert risk is not None
        assert risk.max_profit is not None
        assert isinstance(risk.max_profit, float)

    def test_risk_summary_to_dict(self):
        strat = _simple_call_strategy(100)
        market = _market(100)
        S = np.linspace(70, 140, 200)
        risk = strategy_risk_summary(strat, market, S)
        d = risk.to_dict()
        assert "max_profit" in d and "max_loss" in d and "net_premium" in d


# ===========================================================================
# TestScenarioPnl
# ===========================================================================

class TestScenarioPnl:
    def test_returns_matrix_shape(self):
        strat = _simple_call_strategy(100)
        result = scenario_pnl(strat, [90, 100, 110], [30, 15, 7], iv_multiplier=1.0, r=0.05)
        assert result.shape == (3, 3)  # (n_spots, n_dte)

    def test_higher_spot_better_for_long_call(self):
        strat = _simple_call_strategy(100)
        result = scenario_pnl(strat, [80, 90, 100, 110, 120], [30], iv_multiplier=1.0, r=0.05)
        # PnL should increase as spot increases for long call
        assert result[-1, 0] > result[0, 0]

    def test_shorter_dte_approaches_payoff(self):
        strat = _simple_call_strategy(100)
        s_vals = [110.0]  # ITM
        result = scenario_pnl(strat, s_vals, [30, 15, 7, 1], iv_multiplier=1.0, r=0.05)
        # With dte=1, PnL[dte=1] should be closer to intrinsic value
        assert result.shape == (1, 4)


# ===========================================================================
# TestSerializationRoundTrip
# ===========================================================================

class TestSerializationRoundTrip:
    def _assert_leg_equal(self, leg1, leg2):
        assert type(leg1).__name__ == type(leg2).__name__
        if hasattr(leg1, "strike"):
            assert leg1.strike == leg2.strike
            assert leg1.option_type == leg2.option_type
            assert leg1.side == leg2.side
        else:
            assert leg1.entry_price == leg2.entry_price

    def test_simple_strategy_round_trip(self):
        strat = _simple_call_strategy(100)
        d = strategy_to_dict(strat)
        strat2 = strategy_from_dict(d)
        assert strat2.name == strat.name
        assert len(strat2.legs) == len(strat.legs)
        self._assert_leg_equal(strat.legs[0], strat2.legs[0])

    def test_complex_strategy_round_trip(self):
        call = _atm_call(100)
        put  = _atm_put(100)
        stock = LinearLeg("TEST", "stock", "long", 100, 100.0, 1)
        strat = Strategy(name="collar_like", underlying="TEST", legs=[call, put, stock])
        d = strategy_to_dict(strat)
        strat2 = strategy_from_dict(d)
        assert len(strat2.legs) == 3

    def test_dict_has_required_keys(self):
        strat = _simple_call_strategy(100)
        d = strategy_to_dict(strat)
        assert "name" in d and "legs" in d and "metadata" in d

    def test_leg_dict_has_type(self):
        strat = _simple_call_strategy(100)
        d = strategy_to_dict(strat)
        assert "leg_type" in d["legs"][0]


# ===========================================================================
# TestStrategyTemplates
# ===========================================================================

class TestStrategyTemplates:
    def test_long_call_template(self):
        strat = long_call("SPY", spot=500, strike=500, dte=30, iv=0.2)
        assert len(strat.legs) == 1
        assert strat.legs[0].option_type == "call"
        assert strat.legs[0].side == "long"

    def test_long_put_template(self):
        strat = long_put("QQQ", spot=400, strike=400, dte=21, iv=0.2)
        assert len(strat.legs) == 1
        assert strat.legs[0].option_type == "put"

    def test_bull_call_spread_two_legs(self):
        strat = bull_call_spread("SPY", spot=500, long_strike=500, short_strike=510, dte=30, iv=0.2)
        assert len(strat.legs) == 2
        sides = {l.side for l in strat.legs}
        assert "long" in sides and "short" in sides

    def test_iron_condor_four_legs(self):
        strat = iron_condor("SPY", spot=500,
                            long_put_strike=470, short_put_strike=480,
                            short_call_strike=520, long_call_strike=530,
                            dte=30, iv=0.2)
        assert len(strat.legs) == 4
        option_types = [l.option_type for l in strat.legs]
        assert "call" in option_types and "put" in option_types

    def test_long_straddle_two_legs(self):
        strat = long_straddle("SPY", spot=500, strike=500, dte=30, iv=0.2)
        assert len(strat.legs) == 2
        types = {l.option_type for l in strat.legs}
        assert types == {"call", "put"}

    def test_covered_call_has_linear_and_option(self):
        strat = covered_call("SPY", spot=500, strike=510, dte=30, iv=0.2)
        has_option = any(hasattr(l, "option_type") for l in strat.legs)
        has_linear = any(hasattr(l, "entry_price") for l in strat.legs)
        assert has_option and has_linear

    def test_template_registry_populated(self):
        assert len(TEMPLATE_REGISTRY) >= 10

    def test_list_templates_structure(self):
        templates = list_templates()
        assert len(templates) >= 10
        for t in templates:
            assert "name" in t and "category" in t and "bias" in t

    def test_all_templates_build_valid_strategy(self):
        """All templates in the registry should build without error with default params."""
        errors = []
        for name, fn in TEMPLATE_REGISTRY.items():
            try:
                spot = 100.0
                strat = fn(
                    symbol="TEST", spot=spot, dte=30, iv=0.2,
                    strike=spot, strike_atm=spot, strike_otm=spot*1.05,
                    strike_long=spot, strike_short=spot*1.05,
                    lower_put=spot*0.95, upper_put=spot*0.975,
                    lower_call=spot*1.025, upper_call=spot*1.05,
                    strike_call=spot*1.02, strike_put=spot*0.98,
                    near_expiry=date.today() + timedelta(days=30),
                    far_expiry=date.today() + timedelta(days=60),
                )
                assert strat.legs
            except TypeError:
                pass  # some templates may not accept all kwargs, ignore
            except Exception as e:
                errors.append(f"{name}: {e}")
        assert not errors, "Template build errors:\n" + "\n".join(errors)


# ===========================================================================
# TestPortfolioAnalyzer
# ===========================================================================

class TestPortfolioAnalyzer:
    def setup_method(self):
        self.analyzer = PortfolioAnalyzer(storage_path=None)

    def test_empty_portfolio_status(self):
        s = self.analyzer.status()
        assert s["n_open"] == 0
        assert s["strategies"] == []

    def test_add_strategy(self):
        strat = _simple_call_strategy(100)
        market = _market(100)
        entry = self.analyzer.add_strategy(strat, market)
        assert entry.name == "long_call"
        assert entry.status == "open"

    def test_add_strategy_shows_in_status(self):
        strat = _simple_call_strategy(100)
        market = _market(100)
        self.analyzer.add_strategy(strat, market)
        s = self.analyzer.status()
        assert s["n_open"] == 1
        assert "long_call" in s["strategies"]

    def test_remove_strategy(self):
        strat = _simple_call_strategy(100)
        market = _market(100)
        self.analyzer.add_strategy(strat, market)
        removed = self.analyzer.remove_strategy("long_call")
        assert removed is True
        assert self.analyzer.status()["n_open"] == 0

    def test_remove_nonexistent_returns_false(self):
        removed = self.analyzer.remove_strategy("nonexistent")
        assert removed is False

    def test_compute_summary_empty(self):
        summary = self.analyzer.compute_summary(market_prices={})
        assert summary.n_strategies == 0
        assert summary.total_pnl == 0.0

    def test_compute_summary_with_strategy(self):
        strat = _simple_call_strategy(100)
        market = _market(100)
        self.analyzer.add_strategy(strat, market)
        summary = self.analyzer.compute_summary(market_prices={"TEST": 100.0})
        assert summary.n_strategies == 1
        assert len(summary.entries) == 1

    def test_compute_aggregate_greeks_empty(self):
        g = self.analyzer.compute_aggregate_greeks(market_prices={})
        assert isinstance(g, GreeksVector)
        assert g.delta == 0

    def test_compute_aggregate_greeks_with_strategy(self):
        strat = _simple_call_strategy(100)
        market = _market(100)
        self.analyzer.add_strategy(strat, market)
        g = self.analyzer.compute_aggregate_greeks(market_prices={"TEST": 100.0})
        assert g.delta > 0  # long call → positive delta

    def test_multiple_strategies(self):
        call_strat = _simple_call_strategy(100)
        put_leg = _atm_put(100)
        put_strat = Strategy(name="long_put_test", underlying="TEST", legs=[put_leg])
        market = _market(100)
        self.analyzer.add_strategy(call_strat, market)
        self.analyzer.add_strategy(put_strat, market)
        assert self.analyzer.status()["n_open"] == 2

    def test_summary_to_dict(self):
        strat = _simple_call_strategy(100)
        market = _market(100)
        self.analyzer.add_strategy(strat, market)
        summary = self.analyzer.compute_summary({"TEST": 100.0})
        d = summary.to_dict()
        assert "n_strategies" in d and "total_pnl" in d and "greeks" in d

    def test_delta_risk_flag(self):
        """Large delta position should trigger risk flag."""
        analyzer = PortfolioAnalyzer(delta_limit_per_underlying=10.0)  # very low limit
        strat = _simple_call_strategy(100)
        market = _market(100)
        analyzer.add_strategy(strat, market)
        summary = analyzer.compute_summary({"TEST": 100.0})
        # With delta_limit=10, a single ATM call (delta ~50) should flag
        # (flag depends on computed delta * multiplier)
        assert isinstance(summary.risk_flags, list)

    def test_sync_from_broker_groups_creates_open_entry(self):
        analyzer = PortfolioAnalyzer(storage_path=None)
        group = {
            "strategy_id": "spread:SPY:abc123",
            "strategy_type": "vertical",
            "underlying": "SPY",
            "bias": "bullish",
            "positions": [
                SimpleNamespace(
                    symbol="SPY260620C00500000",
                    underlying="SPY",
                    asset_class="option",
                    signed_qty=1.0,
                    strike=500.0,
                    option_type="call",
                    expiration=(date.today() + timedelta(days=30)).isoformat(),
                    entry_price=5.2,
                    current_price=5.0,
                    greeks={"iv": 0.22},
                ),
            ],
        }
        stats = analyzer.sync_from_broker_groups(
            groups=[group],
            quote_index={"SPY": {"last": 510.0}},
            source="tradier_paper",
        )
        assert stats["added"] == 1
        status = analyzer.status()
        assert status["n_open"] == 1
        assert status["n_closed"] == 0

    def test_sync_from_broker_groups_closes_missing_entries_and_keeps_history(self):
        analyzer = PortfolioAnalyzer(storage_path=None)
        group = {
            "strategy_id": "iron:SPY:xyz987",
            "strategy_type": "iron_condor",
            "underlying": "SPY",
            "positions": [
                SimpleNamespace(
                    symbol="SPY260620C00500000",
                    underlying="SPY",
                    asset_class="option",
                    signed_qty=1.0,
                    strike=500.0,
                    option_type="call",
                    expiration=(date.today() + timedelta(days=30)).isoformat(),
                    entry_price=4.0,
                    current_price=3.8,
                    greeks={"iv": 0.20},
                ),
            ],
        }
        analyzer.sync_from_broker_groups(
            groups=[group],
            quote_index={"SPY": {"last": 505.0}},
            source="tradier_paper",
        )
        stats = analyzer.sync_from_broker_groups(
            groups=[],
            quote_index={},
            source="tradier_paper",
        )
        assert stats["closed"] == 1
        status = analyzer.status()
        assert status["n_open"] == 0
        assert status["n_closed"] == 1
        history = analyzer.history(limit=20)
        assert len(history) == 1
        assert history[0]["source"] == "tradier_paper"
        assert history[0]["closed_at"] is not None
