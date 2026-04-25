from __future__ import annotations

from datetime import datetime, timezone

from atlas_scanner.models.domain_models import (
    CandidateOpportunity,
    GammaFeatures,
    MacroEvent,
    MacroFeatures,
    MarketContextSnapshot,
    OIFlowFeatures,
    PriceFeatures,
    PriceLevel,
    SessionParams,
    StrategyCandidate,
    StrikeLevel,
    VolFeatures,
)


def test_market_context_snapshot_minimal_instantiation() -> None:
    context = MarketContextSnapshot(
        symbol="SPX",
        as_of=datetime(2026, 1, 1, tzinfo=timezone.utc),
        spot=5100.0,
        session_tags=("0DTE Monday",),
        call_walls=(StrikeLevel(strike=5150.0, kind="CALL_WALL"),),
        put_walls=(StrikeLevel(strike=5050.0, kind="PUT_WALL"),),
        key_levels=(PriceLevel(level=5120.0, role="RESISTANCE", label="R1"),),
        macro_events=(
            MacroEvent(
                name="Retail Sales",
                time=datetime(2026, 1, 1, 13, 30, tzinfo=timezone.utc),
                impact="HIGH",
                category="MACRO",
                symbol_scope="INDEX",
            ),
        ),
    )

    assert context.symbol == "SPX"
    assert context.call_walls[0].kind == "CALL_WALL"
    assert context.macro_events[0].impact == "HIGH"


def test_candidate_opportunity_instantiation_with_market_context() -> None:
    session = SessionParams(
        market_hours_gate=datetime(2026, 1, 1, 14, 45, tzinfo=timezone.utc),
        max_open_positions_recommended=4,
        score_min=0.72,
        bias="DEFENSIVE",
        stop_structural_intraday=5075.0,
        auton_mode="paper",
        executor_mode="disabled",
        fail_safe_state="safe_default_bootstrap",
        notes=("wait post-macro",),
    )
    context = MarketContextSnapshot(
        symbol="SPY",
        as_of=datetime(2026, 1, 1, tzinfo=timezone.utc),
        spot=510.0,
        session_params=session,
    )
    candidate = CandidateOpportunity(
        symbol="SPY",
        as_of=datetime(2026, 1, 1, tzinfo=timezone.utc),
        asset_type="ETF",
        dte_candidates=(0, 7, 14),
        dte_preferred=7,
        strikes_relevant=(StrikeLevel(strike=510.0, kind="OTHER"),),
        vol_features=VolFeatures(iv_rank_20d=0.7),
        gamma_features=GammaFeatures(net_gex=1_000_000.0),
        oi_flow_features=OIFlowFeatures(oi_change_1d_pct=0.15),
        price_features=PriceFeatures(trend_state="RANGING"),
        macro_features=MacroFeatures(regime_id="long_gamma_deep"),
        total_score=0.78,
        component_scores={"vol": 0.8, "gamma": 0.75},
        weights_effective={"vol": 0.4, "gamma": 0.2},
        strategy_candidates=(
            StrategyCandidate(
                strategy_type="IRON_CONDOR",
                score=0.8,
                context_match=True,
            ),
        ),
        strengths=("high liquidity",),
        penalties=("elevated event risk",),
        explanation="Pros: high liquidity. Risks: elevated event risk.",
        market_context=context,
    )

    assert candidate.market_context is context
    assert candidate.market_context.session_params is session
    assert candidate.strategy_candidates[0].strategy_type == "IRON_CONDOR"
    assert candidate.price_features.trend_state == "RANGING"


def test_domain_models_tuple_and_literal_fields_hold_valid_values() -> None:
    level = StrikeLevel(strike=100.0, kind="STRUCTURAL")
    candidate = CandidateOpportunity(
        symbol="QQQ",
        as_of=datetime(2026, 1, 1, tzinfo=timezone.utc),
        asset_type="ETF",
        strikes_relevant=(level,),
        strategy_candidates=(
            StrategyCandidate(strategy_type="CALENDAR", score=0.5, context_match=True),
        ),
    )

    assert isinstance(candidate.strikes_relevant, tuple)
    assert candidate.strikes_relevant[0].kind == "STRUCTURAL"
    assert candidate.strategy_candidates[0].strategy_type == "CALENDAR"


def test_session_params_minimal_construction() -> None:
    params = SessionParams()
    assert params.bias == "UNKNOWN"
    assert params.market_hours_gate is None
    assert params.notes == ()


def test_session_params_accepts_valid_bias() -> None:
    params = SessionParams(bias="LONG")
    assert params.bias == "LONG"


def test_market_context_snapshot_can_include_session_params() -> None:
    params = SessionParams(
        market_hours_gate=datetime(2026, 1, 1, 14, 45, tzinfo=timezone.utc),
        bias="NEUTRAL",
    )
    context = MarketContextSnapshot(
        symbol="NDX",
        as_of=datetime(2026, 1, 1, tzinfo=timezone.utc),
        spot=18000.0,
        session_params=params,
    )
    assert context.session_params is params

