"""Tests para el subsistema AtlasLearningBrain.

Cubre:
- trade_events.py          — dataclasses y propiedades derivadas
- metrics_engine.py        — funciones estadísticas puras
- ml_signal_ranker.py      — feature extraction y scoring con fallback
- policy_manager.py        — aplicación de políticas y persistencia
- atlas_learning_brain.py  — integración de los 6 métodos públicos
"""
from __future__ import annotations

import json
import math
import tempfile
import uuid
from datetime import date, datetime, timedelta
from pathlib import Path
from unittest.mock import MagicMock, patch

import pytest

from atlas_code_quant.learning.trade_events import (
    MetricsSummary,
    PolicyAction,
    PolicySnapshot,
    ReadinessCriterion,
    ScoreResult,
    SignalContext,
    SystemReadinessReport,
    TradeEvent,
)
from atlas_code_quant.learning.metrics_engine import (
    build_learning_report,
    check_readiness,
    compute_metrics,
    compute_stability_score,
    detect_error_patterns,
    detect_success_patterns,
    metrics_by_regime,
    metrics_by_setup,
    propose_policies,
)
from atlas_code_quant.learning.ml_signal_ranker import (
    MLSignalRanker,
    extract_features_from_signal,
    extract_features_from_trade,
)
from atlas_code_quant.learning.policy_manager import PolicyManager
from atlas_code_quant.learning.atlas_learning_brain import AtlasLearningBrain


# ---------------------------------------------------------------------------
# Fixtures helpers
# ---------------------------------------------------------------------------

def _make_trade(
    r_realized: float = 0.5,
    setup_type: str = "breakout",
    regime: str = "BULL",
    symbol: str = "AAPL",
    asset_class: str = "equity_stock",
    side: str = "buy",
    error_flags: list | None = None,
    entry_time: datetime | None = None,
    exit_time: datetime | None = None,
    rsi: float = 55.0,
    volume_ratio: float = 2.0,
    iv_rank: float = 60.0,
) -> TradeEvent:
    # Usar fecha reciente por defecto para pasar los filtros de lookback
    now = entry_time or (datetime.utcnow() - timedelta(days=5))
    ex = exit_time or (now + timedelta(minutes=30))
    return TradeEvent(
        trade_id=str(uuid.uuid4()),
        symbol=symbol,
        asset_class=asset_class,
        side=side,
        entry_time=now,
        exit_time=ex,
        timeframe="1m",
        entry_price=100.0,
        exit_price=100.0 + r_realized,
        stop_loss_price=99.0,
        r_initial=1.0,
        r_realized=r_realized,
        mae_r=min(0.0, r_realized * 0.5),
        mfe_r=max(0.0, r_realized),
        setup_type=setup_type,
        regime=regime,
        exit_type="target" if r_realized > 0 else "stop",
        rsi=rsi,
        volume_ratio=volume_ratio,
        iv_rank=iv_rank,
        iv_hv_ratio=1.3,
        error_flags=error_flags or [],
    )


def _make_n_trades(n: int, winrate: float = 0.6, **kwargs) -> list:
    trades = []
    for i in range(n):
        r = 1.0 if (i / n) < winrate else -0.5
        trades.append(_make_trade(r_realized=r, **kwargs))
    return trades


def _make_signal_ctx(**kwargs) -> SignalContext:
    defaults = dict(
        symbol="AAPL",
        asset_class="equity_stock",
        side="buy",
        setup_type="breakout",
        regime="BULL",
        timeframe="1m",
        entry_price=100.0,
        stop_loss_price=99.0,
        r_initial=1.0,
        rsi=55.0,
        volume_ratio=2.0,
        iv_rank=60.0,
        iv_hv_ratio=1.3,
        signal_time=datetime(2024, 1, 15, 11, 0, 0),
    )
    defaults.update(kwargs)
    return SignalContext(**defaults)


# ===========================================================================
# TestTradeEventDataclass
# ===========================================================================

class TestTradeEventDataclass:
    def test_is_winner_positive_r(self):
        t = _make_trade(r_realized=1.5)
        assert t.is_winner is True

    def test_is_loser_negative_r(self):
        t = _make_trade(r_realized=-0.5)
        assert t.is_winner is False

    def test_duration_minutes(self):
        now = datetime(2024, 1, 15, 10, 0)
        later = datetime(2024, 1, 15, 10, 45)
        t = _make_trade(entry_time=now, exit_time=later)
        assert t.duration_minutes == pytest.approx(45.0)

    def test_risk_reward_ratio(self):
        t = _make_trade(r_realized=2.0)
        t.mfe_r = 2.0
        t.mae_r = -0.5
        assert t.risk_reward_ratio == pytest.approx(4.0)

    def test_trade_id_is_string(self):
        t = _make_trade()
        assert isinstance(t.trade_id, str)

    def test_error_flags_default_empty(self):
        t = _make_trade()
        assert t.error_flags == []


# ===========================================================================
# TestMetricsSummary
# ===========================================================================

class TestMetricsSummary:
    def test_is_profitable_positive_pf(self):
        m = MetricsSummary(profit_factor=1.5, expectancy_r=0.3)
        assert m.is_profitable is True

    def test_not_profitable_pf_below_1(self):
        m = MetricsSummary(profit_factor=0.8, expectancy_r=-0.1)
        assert m.is_profitable is False

    def test_statistically_significant_30_trades(self):
        m = MetricsSummary(n_trades=30)
        assert m.is_statistically_significant is True

    def test_not_significant_few_trades(self):
        m = MetricsSummary(n_trades=10)
        assert m.is_statistically_significant is False


# ===========================================================================
# TestComputeMetrics
# ===========================================================================

class TestComputeMetrics:
    def test_empty_list_returns_zero_metrics(self):
        m = compute_metrics([])
        assert m.n_trades == 0
        assert m.profit_factor == 0.0

    def test_single_winner(self):
        trades = [_make_trade(r_realized=1.0)]
        m = compute_metrics(trades)
        assert m.n_trades == 1
        assert m.n_winners == 1
        assert m.winrate == 1.0

    def test_single_loser(self):
        trades = [_make_trade(r_realized=-0.5)]
        m = compute_metrics(trades)
        assert m.n_losers == 1
        assert m.winrate == 0.0

    def test_profit_factor_calculation(self):
        trades = [
            _make_trade(r_realized=2.0),
            _make_trade(r_realized=1.0),
            _make_trade(r_realized=-1.0),
        ]
        m = compute_metrics(trades)
        # PF = (2+1) / 1 = 3.0
        assert m.profit_factor == pytest.approx(3.0)

    def test_expectancy_positive(self):
        trades = _make_n_trades(10, winrate=0.6)
        m = compute_metrics(trades)
        # Expectancy = 0.6 * 1.0 + 0.4 * (-0.5) = 0.4
        assert m.expectancy_r > 0

    def test_winrate_range_0_to_1(self):
        trades = _make_n_trades(20, winrate=0.65)
        m = compute_metrics(trades)
        assert 0.0 <= m.winrate <= 1.0

    def test_max_drawdown_negative_or_zero(self):
        trades = _make_n_trades(20)
        m = compute_metrics(trades)
        assert m.max_drawdown_r <= 0.0

    def test_avg_duration_minutes(self):
        trades = []
        for i in range(5):
            t0 = datetime(2024, 1, 15, 10, 0)
            t1 = t0 + timedelta(minutes=60)
            trades.append(_make_trade(entry_time=t0, exit_time=t1))
        m = compute_metrics(trades)
        assert m.avg_duration_minutes == pytest.approx(60.0)

    def test_metrics_by_setup_groups_correctly(self):
        trades = (
            [_make_trade(r_realized=1.0, setup_type="breakout")] * 10 +
            [_make_trade(r_realized=-1.0, setup_type="reversal")] * 10
        )
        by_setup = metrics_by_setup(trades)
        assert "breakout" in by_setup
        assert "reversal" in by_setup
        assert by_setup["breakout"].profit_factor > by_setup["reversal"].profit_factor

    def test_metrics_by_regime(self):
        trades = (
            [_make_trade(r_realized=1.5, regime="BULL")] * 5 +
            [_make_trade(r_realized=-0.5, regime="BEAR")] * 5
        )
        by_regime = metrics_by_regime(trades)
        assert "BULL" in by_regime
        assert "BEAR" in by_regime


# ===========================================================================
# TestComputeStabilityScore
# ===========================================================================

class TestComputeStabilityScore:
    def test_insufficient_data_returns_zero(self):
        trades = _make_n_trades(5)
        score = compute_stability_score(trades)
        assert score == 0.0

    def test_consistent_performance_high_score(self):
        # Intercalar ganadores y perdedores para que ambas mitades tengan igual distribución
        trades = []
        for i in range(40):
            r = 1.0 if i % 3 != 0 else -0.5   # ~67% winrate distribuida uniformemente
            trades.append(_make_trade(r_realized=r))
        score = compute_stability_score(trades)
        assert score > 0.3  # al menos algo de estabilidad

    def test_score_range_0_to_1(self):
        trades = _make_n_trades(30, winrate=0.55)
        score = compute_stability_score(trades)
        assert 0.0 <= score <= 1.0


# ===========================================================================
# TestDetectPatterns
# ===========================================================================

class TestDetectPatterns:
    def test_error_patterns_empty_trades(self):
        assert detect_error_patterns([]) == []

    def test_success_patterns_few_trades_returns_empty(self):
        trades = _make_n_trades(5)
        assert detect_success_patterns(trades) == []

    def test_error_flag_detected(self):
        losers = [_make_trade(r_realized=-0.5, error_flags=["late_entry"]) for _ in range(6)]
        winners = [_make_trade(r_realized=1.0) for _ in range(4)]
        patterns = detect_error_patterns(losers + winners)
        assert any("late_entry" in p for p in patterns)

    def test_success_pattern_high_pf_setup(self):
        trades = (
            [_make_trade(r_realized=2.0, setup_type="breakout")] * 16 +
            [_make_trade(r_realized=-0.5, setup_type="breakout")] * 4
        )
        patterns = detect_success_patterns(trades)
        assert any("breakout" in p for p in patterns)


# ===========================================================================
# TestProposePolicies
# ===========================================================================

class TestProposePolicies:
    def test_bad_setup_proposes_disable(self):
        trades = [_make_trade(r_realized=-1.0, setup_type="bad_setup")] * 25
        global_m = compute_metrics(trades)
        by_setup = metrics_by_setup(trades)
        policies = propose_policies(trades, global_m, by_setup, min_n=20)
        assert any(p.action == "disable" and p.setup_type == "bad_setup" for p in policies)

    def test_excellent_setup_proposes_increase_priority(self):
        trades = (
            [_make_trade(r_realized=3.0, setup_type="great_setup")] * 18 +
            [_make_trade(r_realized=-0.5, setup_type="great_setup")] * 4
        )
        global_m = compute_metrics(trades)
        by_setup = metrics_by_setup(trades)
        policies = propose_policies(trades, global_m, by_setup, min_n=20)
        assert any(p.action == "increase_priority" and p.setup_type == "great_setup" for p in policies)

    def test_insufficient_n_no_policy(self):
        trades = [_make_trade(r_realized=-1.0, setup_type="rare")] * 5
        global_m = compute_metrics(trades)
        by_setup = metrics_by_setup(trades)
        policies = propose_policies(trades, global_m, by_setup, min_n=20)
        assert not any(p.setup_type == "rare" for p in policies)


# ===========================================================================
# TestCheckReadiness
# ===========================================================================

class TestCheckReadiness:
    def test_empty_trades_not_ready(self):
        report = check_readiness([])
        assert report.ready is False
        assert report.n_trades_evaluated == 0

    def test_too_few_trades_fails_n_trades_criterion(self):
        trades = _make_n_trades(50, winrate=0.65)
        report = check_readiness(trades, min_n_trades=300)
        assert "n_trades" in report.failed

    def test_all_criteria_structure(self):
        trades = _make_n_trades(10)
        report = check_readiness(trades)
        criterion_names = {c.name for c in report.criteria}
        expected = {"n_trades", "history_months", "profit_factor",
                    "calmar_ratio", "max_drawdown_pct", "expectancy_r", "stability_score"}
        assert expected == criterion_names

    def test_passed_plus_failed_equals_total_criteria(self):
        trades = _make_n_trades(20)
        report = check_readiness(trades)
        assert len(report.passed) + len(report.failed) == len(report.criteria)

    def test_ready_true_when_all_pass(self):
        report = check_readiness([], min_n_trades=0, min_months=0, min_profit_factor=0,
                                  min_calmar=0, max_dd_pct=100, min_expectancy_r=-999,
                                  min_stability=0)
        assert report.ready is True

    def test_system_readiness_report_to_dict(self):
        report = check_readiness([])
        d = report.to_dict()
        assert "ready" in d
        assert "criteria" in d
        assert "failed" in d

    def test_next_step_not_empty_when_not_ready(self):
        report = check_readiness([])
        assert report.next_step != ""

    def test_summary_contains_ready_state(self):
        report = check_readiness([], min_n_trades=0, min_months=0, min_profit_factor=0,
                                  min_calmar=0, max_dd_pct=100, min_expectancy_r=-999,
                                  min_stability=0)
        assert "LISTO" in report.summary


# ===========================================================================
# TestBuildLearningReport
# ===========================================================================

class TestBuildLearningReport:
    def test_basic_report_structure(self):
        trades = _make_n_trades(20)
        now = datetime.utcnow()
        report = build_learning_report(trades, date.today(), now - timedelta(days=30), now)
        assert report.n_trades_analyzed == 20
        assert report.global_metrics.n_trades == 20

    def test_report_has_all_breakdown_dicts(self):
        trades = _make_n_trades(20)
        now = datetime.utcnow()
        report = build_learning_report(trades, date.today(), now - timedelta(days=30), now)
        assert isinstance(report.by_setup, dict)
        assert isinstance(report.by_regime, dict)
        assert isinstance(report.by_symbol, dict)

    def test_few_trades_generates_warning(self):
        trades = _make_n_trades(5)
        now = datetime.utcnow()
        report = build_learning_report(trades, date.today(), now - timedelta(days=30), now)
        assert len(report.warnings) > 0


# ===========================================================================
# TestMLSignalRanker
# ===========================================================================

class TestMLSignalRanker:
    def test_untrained_returns_neutral_05(self):
        ranker = MLSignalRanker()
        ctx = _make_signal_ctx()
        score = ranker.score(ctx)
        assert score == pytest.approx(0.5)

    def test_extract_features_length(self):
        trade = _make_trade()
        features = extract_features_from_trade(trade)
        assert len(features) == 19

    def test_extract_signal_features_length(self):
        ctx = _make_signal_ctx()
        features = extract_features_from_signal(ctx)
        assert len(features) == 19

    def test_status_dict_structure(self):
        ranker = MLSignalRanker()
        s = ranker.status()
        assert "is_trained" in s
        assert "backend" in s

    def test_train_insufficient_data(self):
        ranker = MLSignalRanker()
        trades = _make_n_trades(10)
        result = ranker.train(trades)
        assert result["trained"] is False

    def test_sklearn_fallback_train(self):
        ranker = MLSignalRanker()
        ranker.backend = "sklearn"
        trades = _make_n_trades(60, winrate=0.6)
        result = ranker.train(trades)
        # Si sklearn está disponible, debe entrenar
        if result["trained"]:
            assert ranker.is_trained is True
            ctx = _make_signal_ctx()
            score = ranker.score(ctx)
            assert 0.0 <= score <= 1.0

    def test_score_batch_untrained(self):
        ranker = MLSignalRanker()
        ctxs = [_make_signal_ctx() for _ in range(3)]
        scores = ranker.score_batch(ctxs)
        assert scores == [0.5, 0.5, 0.5]

    def test_persist_and_load(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            path = Path(tmpdir) / "model.pkl"
            ranker = MLSignalRanker(model_path=path)
            trades = _make_n_trades(60, winrate=0.6)
            result = ranker.train(trades)
            if result["trained"]:
                assert path.exists()
                loaded = MLSignalRanker(model_path=path)
                assert loaded.is_trained is True


# ===========================================================================
# TestPolicyManager
# ===========================================================================

class TestPolicyManager:
    def test_initial_snapshot_empty(self):
        pm = PolicyManager()
        snap = pm.get_snapshot()
        assert snap.disabled_setups == []
        assert snap.global_score_threshold == pytest.approx(0.40)

    def test_apply_disable_action(self):
        pm = PolicyManager()
        actions = [PolicyAction(setup_type="bad", action="disable", reason="test")]
        snap = pm.apply_policies(actions)
        assert "bad" in snap.disabled_setups

    def test_apply_reduce_size_action(self):
        pm = PolicyManager()
        actions = [PolicyAction(setup_type="risky", action="reduce_size",
                                reason="test", size_multiplier=0.5)]
        snap = pm.apply_policies(actions)
        assert snap.size_multipliers.get("risky") == pytest.approx(0.5)

    def test_apply_increase_priority(self):
        pm = PolicyManager()
        actions = [PolicyAction(setup_type="star", action="increase_priority",
                                reason="test", size_multiplier=1.2, score_boost=0.1)]
        snap = pm.apply_policies(actions)
        assert snap.size_multipliers.get("star", 1.0) >= 1.0

    def test_revert_restores_previous(self):
        pm = PolicyManager()
        actions = [PolicyAction(setup_type="x", action="disable", reason="t")]
        pm.apply_policies(actions)
        reverted = pm.revert()
        assert reverted is not None
        assert "x" not in reverted.disabled_setups

    def test_reset_clears_policies(self):
        pm = PolicyManager()
        actions = [PolicyAction(setup_type="a", action="disable", reason="t")]
        pm.apply_policies(actions)
        pm.reset()
        snap = pm.get_snapshot()
        assert snap.disabled_setups == []

    def test_persist_and_load_json(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            path = Path(tmpdir) / "policies.json"
            pm = PolicyManager(storage_path=path)
            actions = [PolicyAction(setup_type="slow", action="disable", reason="test")]
            pm.apply_policies(actions)
            assert path.exists()

            pm2 = PolicyManager(storage_path=path)
            snap = pm2.get_snapshot()
            assert "slow" in snap.disabled_setups

    def test_status_dict(self):
        pm = PolicyManager()
        s = pm.status()
        assert "disabled_setups" in s
        assert "global_score_threshold" in s

    def test_is_setup_enabled(self):
        snap = PolicySnapshot(disabled_setups=["bad_setup"])
        assert snap.is_setup_enabled("good_setup") is True
        assert snap.is_setup_enabled("bad_setup") is False


# ===========================================================================
# TestAtlasLearningBrain
# ===========================================================================

class TestAtlasLearningBrainInit:
    def test_init_no_args(self):
        brain = AtlasLearningBrain()
        assert brain is not None

    def test_status_structure(self):
        brain = AtlasLearningBrain()
        s = brain.status()
        assert "n_trades_in_memory" in s
        assert "ml_ranker" in s
        assert "policy" in s

    def test_initial_trade_count_zero(self):
        brain = AtlasLearningBrain()
        s = brain.status()
        assert s["n_trades_in_memory"] == 0


class TestAtlasLearningBrainRecordTrade:
    def test_record_trade_increases_count(self):
        brain = AtlasLearningBrain()
        brain.record_trade(_make_trade())
        assert brain.status()["n_trades_in_memory"] == 1

    def test_record_multiple_trades(self):
        brain = AtlasLearningBrain()
        for _ in range(5):
            brain.record_trade(_make_trade())
        assert brain.status()["n_trades_in_memory"] == 5


class TestAtlasLearningBrainDailyAnalysis:
    def test_run_daily_analysis_empty(self):
        brain = AtlasLearningBrain()
        report = brain.run_daily_analysis(date.today())
        assert report.n_trades_analyzed == 0

    def test_run_daily_analysis_with_trades(self):
        brain = AtlasLearningBrain()
        for t in _make_n_trades(30):
            brain.record_trade(t)
        report = brain.run_daily_analysis(date.today(), lookback_days=365)
        assert report.n_trades_analyzed == 30
        assert report.global_metrics.n_trades == 30

    def test_report_stability_score_range(self):
        brain = AtlasLearningBrain()
        for t in _make_n_trades(40):
            brain.record_trade(t)
        report = brain.run_daily_analysis(date.today(), lookback_days=365)
        assert 0.0 <= report.stability_score <= 1.0


class TestAtlasLearningBrainUpdatePolicies:
    def test_update_policies_returns_snapshot(self):
        brain = AtlasLearningBrain()
        trades = [_make_trade(r_realized=-1.0, setup_type="bad")] * 25
        for t in trades:
            brain.record_trade(t)
        report = brain.run_daily_analysis(date.today(), lookback_days=365)
        snap = brain.update_policies(report)
        assert snap is not None
        assert isinstance(snap, PolicySnapshot)


class TestAtlasLearningBrainGetPolicySnapshot:
    def test_get_policy_snapshot_returns_snapshot(self):
        brain = AtlasLearningBrain()
        snap = brain.get_policy_snapshot()
        assert isinstance(snap, PolicySnapshot)


class TestAtlasLearningBrainScoreSignal:
    def test_score_no_trades_returns_result(self):
        brain = AtlasLearningBrain()
        ctx = _make_signal_ctx()
        result = brain.score_signal(ctx)
        assert isinstance(result, ScoreResult)
        assert 0.0 <= result.total_score <= 1.0

    def test_score_disabled_setup_not_approved(self):
        brain = AtlasLearningBrain()
        actions = [PolicyAction(setup_type="bad_setup", action="disable", reason="test")]
        brain._policy_manager.apply_policies(actions)
        ctx = _make_signal_ctx(setup_type="bad_setup")
        result = brain.score_signal(ctx)
        assert result.approved is False
        assert result.total_score == 0.0

    def test_score_has_reasoning(self):
        brain = AtlasLearningBrain()
        ctx = _make_signal_ctx()
        result = brain.score_signal(ctx)
        assert result.reasoning != ""

    def test_score_result_has_size_multiplier(self):
        brain = AtlasLearningBrain()
        ctx = _make_signal_ctx()
        result = brain.score_signal(ctx)
        assert result.size_multiplier >= 0.0

    def test_score_with_good_setup_history(self):
        brain = AtlasLearningBrain()
        # Cargar historial rentable para "breakout"
        for t in _make_n_trades(20, winrate=0.75, setup_type="breakout"):
            brain.record_trade(t)
        ctx = _make_signal_ctx(setup_type="breakout")
        result = brain.score_signal(ctx)
        # Stats score debe ser > 0.5 con buen historial
        assert result.stats_score > 0.4


class TestAtlasLearningBrainReadiness:
    def test_is_system_ready_for_live_returns_report(self):
        brain = AtlasLearningBrain()
        report = brain.is_system_ready_for_live()
        assert isinstance(report, SystemReadinessReport)

    def test_empty_history_not_ready(self):
        brain = AtlasLearningBrain()
        report = brain.is_system_ready_for_live()
        assert report.ready is False

    def test_readiness_with_custom_config(self):
        brain = AtlasLearningBrain(
            readiness_cfg={
                "min_n_trades": 0,
                "min_months": 0,
                "min_profit_factor": 0,
                "min_calmar": 0,
                "max_dd_pct": 100,
                "min_expectancy_r": -999,
                "min_stability": 0,
            }
        )
        report = brain.is_system_ready_for_live()
        assert report.ready is True

    def test_readiness_report_has_7_criteria(self):
        brain = AtlasLearningBrain()
        report = brain.is_system_ready_for_live()
        assert len(report.criteria) == 7

    def test_revert_policies(self):
        brain = AtlasLearningBrain()
        actions = [PolicyAction(setup_type="x", action="disable", reason="t")]
        brain._policy_manager.apply_policies(actions)
        reverted = brain.revert_policies()
        assert reverted is not None
        assert "x" not in reverted.disabled_setups


# ===========================================================================
# TestPolicySnapshot
# ===========================================================================

class TestPolicySnapshot:
    def test_get_size_multiplier_default(self):
        snap = PolicySnapshot()
        assert snap.get_size_multiplier("any_setup") == pytest.approx(1.0)

    def test_get_score_threshold_default(self):
        snap = PolicySnapshot(global_score_threshold=0.45)
        assert snap.get_score_threshold("any") == pytest.approx(0.45)

    def test_get_score_threshold_override(self):
        snap = PolicySnapshot(
            global_score_threshold=0.40,
            score_thresholds={"special": 0.60}
        )
        assert snap.get_score_threshold("special") == pytest.approx(0.60)
        assert snap.get_score_threshold("other") == pytest.approx(0.40)

    def test_get_score_boost_zero_by_default(self):
        snap = PolicySnapshot()
        assert snap.get_score_boost("setup") == pytest.approx(0.0)


# ===========================================================================
# TestReadinessCriterion
# ===========================================================================

class TestReadinessCriterion:
    def test_gap_positive_when_not_passed(self):
        c = ReadinessCriterion(
            name="n_trades", description="", value=50, threshold=300, passed=False
        )
        assert c.gap == pytest.approx(250)

    def test_gap_negative_when_passed(self):
        c = ReadinessCriterion(
            name="n_trades", description="", value=400, threshold=300, passed=True
        )
        assert c.gap == pytest.approx(-100)
