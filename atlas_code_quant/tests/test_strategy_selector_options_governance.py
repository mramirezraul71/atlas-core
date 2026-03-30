from __future__ import annotations

from atlas_code_quant.selector.strategy_selector import StrategySelectorService
from atlas_code_quant.selector import strategy_selector as selector_module


class _TrackerStub:
    def build_summary(self, account_scope: str = "paper", account_id: str | None = None) -> dict:
        return {
            "balances": {
                "cash": 100000.0,
                "option_buying_power": 100000.0,
            },
            "account_session": {
                "scope": account_scope,
                "account_id": account_id,
            },
        }


class _LearningStub:
    def context(self, **_: object) -> dict:
        return {
            "symbol_bias": 0.0,
            "directional_symbol_bias": 0.0,
        }

    def strategy_bias(self, strategy_type: str, account_scope: str = "paper") -> float:
        return 0.0

    def risk_multiplier(self, account_scope: str = "paper") -> float:
        return 1.0


def _service() -> StrategySelectorService:
    return StrategySelectorService(_TrackerStub(), _LearningStub())  # type: ignore[arg-type]


def test_candidate_structures_promotes_call_diagonal_for_bull_time_spread() -> None:
    service = _service()
    candidate = {
        "symbol": "AAPL",
        "direction": "alcista",
        "timeframe": "4h",
        "strategy_key": "trend_ema_stack",
        "selection_score": 82.0,
        "local_win_rate_pct": 61.0,
        "predicted_move_pct": 2.4,
        "confirmation": {"direction": "alcista", "higher_timeframe": "1d"},
        "regime": "BULL",
        "iv_rank": 38.0,
        "iv_hv_ratio": 1.0,
        "liquidity_score": 0.92,
        "term_structure_slope": 1.07,
        "skew_pct": 0.04,
        "options_thesis": "calendar",
        "order_flow": {
            "direction": "alcista",
            "confidence_pct": 77.0,
            "score_pct": 74.0,
        },
    }

    structures = service._candidate_structures(
        candidate,
        account_scope="paper",
        allow_equity=False,
        allow_credit=True,
        prefer_defined_risk=True,
    )

    assert structures[0]["strategy_type"] == "call_diagonal_debit_spread"
    assert structures[0]["governance_alignment"]["recommended_family"] == "term_structure_time_spread"
    assert any(
        "gobernanza de volatilidad y regimen favorece esta estructura" in note
        for note in structures[0]["why"]
    )


def test_option_first_session_promotes_governed_option_over_equity() -> None:
    service = _service()
    candidate = {
        "symbol": "AAPL",
        "direction": "alcista",
        "timeframe": "4h",
        "strategy_key": "trend_ema_stack",
        "selection_score": 82.0,
        "local_win_rate_pct": 61.0,
        "predicted_move_pct": 2.4,
        "confirmation": {"direction": "alcista", "higher_timeframe": "1d"},
        "regime": "BULL",
        "iv_rank": 38.0,
        "iv_hv_ratio": 1.0,
        "liquidity_score": 0.92,
        "term_structure_slope": 1.07,
        "skew_pct": 0.04,
        "options_thesis": "calendar",
        "has_options": True,
        "order_flow": {
            "direction": "alcista",
            "confidence_pct": 77.0,
            "score_pct": 74.0,
        },
    }

    balanced = service._candidate_structures(
        candidate,
        account_scope="paper",
        allow_equity=True,
        allow_credit=True,
        prefer_defined_risk=True,
        options_session_mode="balanced",
    )
    option_first = service._candidate_structures(
        candidate,
        account_scope="paper",
        allow_equity=True,
        allow_credit=True,
        prefer_defined_risk=True,
        options_session_mode="option_first",
    )

    assert balanced[0]["strategy_type"] == "equity_long"
    assert option_first[0]["strategy_type"] == "call_diagonal_debit_spread"


def test_options_only_session_removes_equity_candidates() -> None:
    service = _service()
    candidate = {
        "symbol": "AAPL",
        "direction": "alcista",
        "timeframe": "4h",
        "strategy_key": "trend_ema_stack",
        "selection_score": 82.0,
        "local_win_rate_pct": 61.0,
        "predicted_move_pct": 2.4,
        "confirmation": {"direction": "alcista", "higher_timeframe": "1d"},
        "regime": "BULL",
        "iv_rank": 38.0,
        "iv_hv_ratio": 1.0,
        "liquidity_score": 0.92,
        "term_structure_slope": 1.07,
        "skew_pct": 0.04,
        "options_thesis": "calendar",
        "has_options": True,
        "order_flow": {
            "direction": "alcista",
            "confidence_pct": 77.0,
            "score_pct": 74.0,
        },
    }

    structures = service._candidate_structures(
        candidate,
        account_scope="paper",
        allow_equity=True,
        allow_credit=True,
        prefer_defined_risk=True,
        options_session_mode="options_only",
    )

    assert all(not row["strategy_type"].startswith("equity_") for row in structures)
    assert structures[0]["strategy_type"] == "call_diagonal_debit_spread"


def test_candidate_structures_promotes_iron_butterfly_for_high_iv_sideways() -> None:
    service = _service()
    candidate = {
        "symbol": "SPY",
        "direction": "alcista",
        "timeframe": "1d",
        "strategy_key": "range_compression",
        "selection_score": 74.0,
        "local_win_rate_pct": 58.0,
        "predicted_move_pct": 1.3,
        "confirmation": {"direction": "alcista", "higher_timeframe": "1d"},
        "regime": "SIDEWAYS",
        "iv_rank": 82.0,
        "iv_hv_ratio": 1.5,
        "liquidity_score": 0.95,
        "term_structure_slope": 1.01,
        "skew_pct": 0.03,
        "options_thesis": "neutral_income",
        "order_flow": {
            "direction": "neutral",
            "confidence_pct": 30.0,
            "score_pct": 48.0,
        },
    }

    structures = service._candidate_structures(
        candidate,
        account_scope="paper",
        allow_equity=False,
        allow_credit=True,
        prefer_defined_risk=True,
    )

    assert structures[0]["strategy_type"] == "iron_butterfly"
    assert structures[0]["governance_alignment"]["recommended_strategy"] == "iron_butterfly"
    assert structures[0]["vehicle_type"] == "neutral_theta"


def test_candidate_structures_keep_governance_driven_alternatives_visible() -> None:
    service = _service()
    candidate = {
        "symbol": "QQQ",
        "direction": "bajista",
        "timeframe": "4h",
        "strategy_key": "pullback_rejection",
        "selection_score": 79.0,
        "local_win_rate_pct": 57.0,
        "predicted_move_pct": 2.1,
        "confirmation": {"direction": "bajista", "higher_timeframe": "1d"},
        "regime": "BEAR",
        "iv_rank": 42.0,
        "iv_hv_ratio": 1.0,
        "liquidity_score": 0.9,
        "term_structure_slope": 1.06,
        "skew_pct": 0.02,
        "options_thesis": "diagonal",
        "order_flow": {
            "direction": "bajista",
            "confidence_pct": 70.0,
            "score_pct": 73.0,
        },
    }

    structures = service._candidate_structures(
        candidate,
        account_scope="paper",
        allow_equity=False,
        allow_credit=True,
        prefer_defined_risk=True,
    )

    top_types = [row["strategy_type"] for row in structures[:3]]
    assert "put_diagonal_debit_spread" in top_types
    assert "bear_put_debit_spread" in top_types


class _ProbabilityResult:
    def __init__(self, payload: dict) -> None:
        self._payload = payload

    def to_dict(self) -> dict:
        return dict(self._payload)


def _fake_probability(symbol: str, strategy_type: str, account_scope: str = "paper", account_id: str | None = None):
    common = {
        "symbol": symbol,
        "strategy_type": strategy_type,
        "win_rate_pct": 68.0,
        "expected_pnl": 1.2,
        "expected_roi_pct": 12.0,
        "market_snapshot": {"spot": 100.0},
        "markov_snapshot": {},
        "assumptions": {},
        "selected_contract": {"symbol": f"{symbol}_OPT"},
    }
    if strategy_type == "call_diagonal_debit_spread":
        return _ProbabilityResult(
            {
                **common,
                "net_premium": 2.35,
                "selected_legs": [
                    {
                        "side": "long",
                        "option_type": "call",
                        "strike": 100.0,
                        "premium_mid": 4.0,
                        "expiration": "2026-05-15",
                        "dte": 45,
                        "symbol": f"{symbol}260515C00100000",
                        "bid": 3.9,
                        "ask": 4.1,
                        "volume": 100,
                        "open_interest": 200,
                        "implied_volatility": 0.24,
                    },
                    {
                        "side": "short",
                        "option_type": "call",
                        "strike": 105.0,
                        "premium_mid": 1.65,
                        "expiration": "2026-04-17",
                        "dte": 17,
                        "symbol": f"{symbol}260417C00105000",
                        "bid": 1.6,
                        "ask": 1.7,
                        "volume": 120,
                        "open_interest": 250,
                        "implied_volatility": 0.22,
                    },
                ],
            }
        )
    return _ProbabilityResult(
        {
            **common,
            "net_premium": 1.8,
            "selected_legs": [
                {
                    "side": "long",
                    "option_type": "call",
                    "strike": 100.0,
                    "premium_mid": 1.8,
                    "expiration": "2026-04-17",
                    "dte": 17,
                    "symbol": f"{symbol}260417C00100000",
                    "bid": 1.7,
                    "ask": 1.9,
                    "volume": 100,
                    "open_interest": 200,
                    "implied_volatility": 0.2,
                }
            ],
        }
    )


def test_proposal_builds_multileg_order_seed_for_time_spread(monkeypatch) -> None:
    monkeypatch.setattr(selector_module, "get_winning_probability", _fake_probability)
    service = _service()
    candidate = {
        "symbol": "AAPL",
        "direction": "alcista",
        "timeframe": "4h",
        "strategy_key": "trend_ema_stack",
        "selection_score": 82.0,
        "local_win_rate_pct": 61.0,
        "predicted_move_pct": 2.4,
        "confirmation": {"direction": "alcista", "higher_timeframe": "1d"},
        "regime": "BULL",
        "iv_rank": 38.0,
        "iv_hv_ratio": 1.0,
        "liquidity_score": 0.92,
        "term_structure_slope": 1.07,
        "skew_pct": 0.04,
        "options_thesis": "calendar",
        "price": 100.0,
        "order_flow": {"direction": "alcista", "confidence_pct": 77.0, "score_pct": 74.0},
    }

    proposal = service.proposal(
        candidate=candidate,
        account_scope="paper",
        allow_equity=False,
        allow_credit=True,
    )

    order_seed = proposal["order_seed"]
    assert order_seed["strategy_type"] == "call_diagonal_debit_spread"
    assert order_seed["tradier_class"] == "multileg"
    assert len(order_seed["legs"]) == 2
    assert order_seed["price"] == 2.35


def test_proposal_builds_single_option_order_seed(monkeypatch) -> None:
    monkeypatch.setattr(selector_module, "get_winning_probability", _fake_probability)
    service = _service()
    candidate = {
        "symbol": "NVDA",
        "direction": "alcista",
        "timeframe": "15m",
        "strategy_key": "breakout_event",
        "selection_score": 80.0,
        "local_win_rate_pct": 59.0,
        "predicted_move_pct": 3.5,
        "confirmation": {"direction": "alcista", "higher_timeframe": "1h"},
        "regime": "BULL",
        "iv_rank": 18.0,
        "iv_hv_ratio": 0.9,
        "liquidity_score": 0.95,
        "term_structure_slope": 1.0,
        "skew_pct": 0.01,
        "event_near": True,
        "options_thesis": "event_driven",
        "price": 100.0,
        "order_flow": {"direction": "alcista", "confidence_pct": 80.0, "score_pct": 79.0},
    }

    proposal = service.proposal(
        candidate=candidate,
        account_scope="paper",
        allow_equity=False,
        allow_credit=True,
    )

    order_seed = proposal["order_seed"]
    assert order_seed["strategy_type"] == "long_call"
    assert order_seed["tradier_class"] == "option"
    assert order_seed["option_symbol"] == "NVDA260417C00100000"
    assert "legs" not in order_seed or not order_seed["legs"]


def test_proposal_includes_market_context_and_degrades_when_context_is_weak(monkeypatch) -> None:
    monkeypatch.setattr(selector_module, "get_winning_probability", _fake_probability)
    service = _service()
    candidate = {
        "symbol": "TSLA",
        "direction": "alcista",
        "timeframe": "15m",
        "strategy_key": "breakout_event",
        "selection_score": 58.0,
        "local_win_rate_pct": 50.0,
        "predicted_move_pct": 3.3,
        "confirmation": {"direction": "alcista", "higher_timeframe": "1h"},
        "regime": "BULL",
        "iv_rank": 80.0,
        "iv_hv_ratio": 1.4,
        "liquidity_score": 0.42,
        "event_near": True,
        "price": 100.0,
        "order_flow": {"direction": "alcista", "confidence_pct": 45.0, "score_pct": 44.0},
    }

    proposal = service.proposal(
        candidate=candidate,
        account_scope="paper",
        allow_equity=False,
        allow_credit=True,
    )

    assert proposal["market_context"]["decision_gate"]["action"] in {"block", "degrade"}
    assert proposal["order_seed"]["market_context"]["symbol"] == "TSLA"
    assert proposal["automation_ready"]["context_gate"] in {"blocked", "degraded"}
