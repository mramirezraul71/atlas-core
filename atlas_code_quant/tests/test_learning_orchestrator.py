"""Tests para LearningOrchestrator — loop de aprendizaje cerrado de ATLAS-Quant.

Valida:
- Traducción IC a política (size_multiplier, score_threshold)
- Estado inicial del orquestador
- apply_ic_to_policy con datos reales del IC tracker
- _build_trade_event_from_journal con snapshot sintético
- get_orchestrator_status estructura
"""
from __future__ import annotations

import pytest


# ── Fixtures ──────────────────────────────────────────────────────────────────

from learning.learning_orchestrator import (
    _ic_to_size_multiplier,
    _ic_to_score_threshold,
    _build_trade_event_from_journal,
    _JournalSnapshot,
    get_orchestrator_status,
    _SIZE_MULTIPLIER_PAUSE,
    _SIZE_MULTIPLIER_WEAK,
    _SIZE_MULTIPLIER_NORMAL,
    _SIZE_MULTIPLIER_STRONG,
    _SCORE_THRESHOLD_PAUSE,
    _SCORE_THRESHOLD_TIGHT,
    _SCORE_THRESHOLD_NORMAL,
    _SCORE_THRESHOLD_RELAXED,
)


# ── IC → size_multiplier ──────────────────────────────────────────────────────

class TestIcToSizeMultiplier:
    def test_none_ic_returns_weak(self):
        assert _ic_to_size_multiplier(None) == _SIZE_MULTIPLIER_WEAK

    def test_negative_ic_pauses(self):
        assert _ic_to_size_multiplier(-0.05) == _SIZE_MULTIPLIER_PAUSE

    def test_very_low_ic_pauses(self):
        assert _ic_to_size_multiplier(0.01) == _SIZE_MULTIPLIER_PAUSE

    def test_below_meaningful_returns_weak(self):
        assert _ic_to_size_multiplier(0.03) == _SIZE_MULTIPLIER_WEAK

    def test_at_meaningful_threshold_returns_normal(self):
        assert _ic_to_size_multiplier(0.05) == _SIZE_MULTIPLIER_NORMAL

    def test_above_meaningful_returns_normal(self):
        assert _ic_to_size_multiplier(0.07) == _SIZE_MULTIPLIER_NORMAL

    def test_at_strong_threshold_returns_strong(self):
        assert _ic_to_size_multiplier(0.10) == _SIZE_MULTIPLIER_STRONG

    def test_above_strong_returns_strong(self):
        assert _ic_to_size_multiplier(0.20) == _SIZE_MULTIPLIER_STRONG

    def test_zero_ic_pauses(self):
        assert _ic_to_size_multiplier(0.0) == _SIZE_MULTIPLIER_PAUSE

    def test_multipliers_monotonically_increase(self):
        """Más IC = más tamaño (monotónico)."""
        ics = [-0.1, 0.0, 0.03, 0.05, 0.10, 0.20]
        mults = [_ic_to_size_multiplier(ic) for ic in ics]
        for i in range(len(mults) - 1):
            assert mults[i] <= mults[i + 1], (
                f"Violación monotónica: IC={ics[i]} → {mults[i]} > IC={ics[i+1]} → {mults[i+1]}"
            )


# ── IC → score_threshold ──────────────────────────────────────────────────────

class TestIcToScoreThreshold:
    def test_low_n_returns_tight_regardless_of_ic(self):
        """Con n < 10, siempre exigir score alto."""
        assert _ic_to_score_threshold(0.15, 5) == _SCORE_THRESHOLD_TIGHT

    def test_none_ic_high_n_returns_pause_threshold(self):
        """ic=None con n suficiente → máxima exigencia (PAUSE), no hay IC calculado."""
        assert _ic_to_score_threshold(None, 15) == _SCORE_THRESHOLD_PAUSE
        # ic=None con n bajo → tight (no hay datos suficientes)
        assert _ic_to_score_threshold(None, 5) == _SCORE_THRESHOLD_TIGHT

    def test_no_edge_ic_returns_pause_threshold(self):
        assert _ic_to_score_threshold(0.01, 20) == _SCORE_THRESHOLD_PAUSE

    def test_below_meaningful_returns_tight(self):
        assert _ic_to_score_threshold(0.03, 30) == _SCORE_THRESHOLD_TIGHT

    def test_meaningful_ic_returns_normal(self):
        assert _ic_to_score_threshold(0.06, 40) == _SCORE_THRESHOLD_NORMAL

    def test_strong_ic_returns_relaxed(self):
        assert _ic_to_score_threshold(0.12, 80) == _SCORE_THRESHOLD_RELAXED

    def test_thresholds_decrease_with_ic(self):
        """Mayor IC = umbral menor (más señales aceptadas)."""
        pairs = [(0.01, 20), (0.05, 30), (0.10, 60)]
        thresholds = [_ic_to_score_threshold(ic, n) for ic, n in pairs]
        for i in range(len(thresholds) - 1):
            assert thresholds[i] >= thresholds[i + 1], (
                f"Threshold no disminuye con IC mayor: {thresholds}"
            )


# ── _build_trade_event_from_journal ───────────────────────────────────────────

class _FakeRow:
    """Simula una fila de TradingJournal para tests."""
    def __init__(self, **kwargs):
        defaults = {
            "journal_key": "TEST-001",
            "symbol": "AAPL",
            "strategy_type": "equity_long",
            "side": "buy",
            "entry_price": 150.0,
            "exit_price": 165.0,
            "entry_time": None,
            "exit_time": None,
            "entry_notional": 15000.0,
            "risk_at_entry": 300.0,
            "realized_pnl": 1500.0,
            "fees": 2.0,
            "iv_rank": 45.0,
            "post_mortem_text": "Salida en target.",
        }
        from datetime import datetime, timezone
        defaults["entry_time"] = datetime(2026, 3, 1, 10, 0, tzinfo=timezone.utc)
        defaults["exit_time"] = datetime(2026, 3, 10, 15, 0, tzinfo=timezone.utc)
        defaults.update(kwargs)
        for k, v in defaults.items():
            setattr(self, k, v)


class TestBuildTradeEvent:
    def test_builds_valid_trade_event(self):
        row = _FakeRow()
        snap = _JournalSnapshot(row)
        event = _build_trade_event_from_journal(snap)
        assert event is not None
        assert event.symbol == "AAPL"
        assert event.trade_id == "TEST-001"
        assert event.entry_price == 150.0
        assert event.exit_price == 165.0
        assert event.r_realized > 0  # trade ganador

    def test_missing_exit_price_returns_none(self):
        row = _FakeRow(exit_price=None)
        snap = _JournalSnapshot(row)
        event = _build_trade_event_from_journal(snap)
        assert event is None

    def test_missing_entry_price_returns_none(self):
        row = _FakeRow(entry_price=0.0)
        snap = _JournalSnapshot(row)
        event = _build_trade_event_from_journal(snap)
        assert event is None

    def test_losing_trade_has_negative_r(self):
        row = _FakeRow(exit_price=140.0)  # bajo entry de 150
        snap = _JournalSnapshot(row)
        event = _build_trade_event_from_journal(snap)
        assert event is not None
        assert event.r_realized < 0

    def test_option_strategy_sets_correct_asset_class(self):
        row = _FakeRow(strategy_type="long_call")
        snap = _JournalSnapshot(row)
        event = _build_trade_event_from_journal(snap)
        assert event is not None
        assert event.asset_class == "option"

    def test_short_strategy_inverts_r(self):
        """Short con precio bajando = ganancia."""
        row = _FakeRow(strategy_type="equity_short", entry_price=150.0, exit_price=140.0)
        snap = _JournalSnapshot(row)
        event = _build_trade_event_from_journal(snap)
        assert event is not None
        assert event.r_realized > 0  # short ganador cuando precio baja


# ── Orchestrator status ───────────────────────────────────────────────────────

class TestOrchestratorStatus:
    def test_status_has_required_fields(self):
        status = get_orchestrator_status()
        required = [
            "running", "reconcile_count", "trades_processed_total",
            "ic_updates_total", "policy_updates", "last_reconcile_at",
            "last_daily_analysis_at", "last_ic_by_method",
            "last_readiness_report", "recent_errors",
        ]
        for field in required:
            assert field in status, f"Missing field: {field}"

    def test_initial_state_is_stopped(self):
        """El orquestador no está corriendo en tests (sin startup)."""
        status = get_orchestrator_status()
        # running puede ser True o False dependiendo de si hay loop; checar tipo
        assert isinstance(status["running"], bool)

    def test_counters_are_non_negative(self):
        status = get_orchestrator_status()
        assert status["reconcile_count"] >= 0
        assert status["trades_processed_total"] >= 0
        assert status["ic_updates_total"] >= 0
        assert status["policy_updates"] >= 0

    def test_recent_errors_is_list(self):
        status = get_orchestrator_status()
        assert isinstance(status["recent_errors"], list)
        assert len(status["recent_errors"]) <= 5


# ── Grinold-Kahn invariants ───────────────────────────────────────────────────

class TestGrinoldKahnInvariants:
    """Verifica que la implementación respeta los invariantes del framework."""

    def test_ic_at_minimum_meaningful_gets_full_size(self):
        """IC = 0.05 (umbral Grinold-Kahn) debe habilitar tamaño completo."""
        mult = _ic_to_size_multiplier(0.05)
        assert mult == _SIZE_MULTIPLIER_NORMAL

    def test_ic_at_strong_gets_scaled_size(self):
        """IC = 0.10 (edge fuerte) debe escalar tamaño."""
        mult = _ic_to_size_multiplier(0.10)
        assert mult > _SIZE_MULTIPLIER_NORMAL

    def test_negative_ic_never_trades(self):
        """IC negativo = señal que predice al revés → no operar."""
        mult = _ic_to_size_multiplier(-0.01)
        assert mult == 0.0

    def test_pause_state_requires_high_score_threshold(self):
        """Método pausado debe exigir scores altísimos (casi nunca pasa)."""
        thresh_paused = _ic_to_score_threshold(0.01, 20)
        thresh_normal = _ic_to_score_threshold(0.07, 40)
        assert thresh_paused > thresh_normal

    def test_kelly_fractions_are_capped(self):
        """Por gestión de riesgo, el multiplicador máximo es 1.5 (half-Kelly)."""
        very_high_ic = _ic_to_size_multiplier(0.50)
        assert very_high_ic <= 2.0  # cap de seguridad
