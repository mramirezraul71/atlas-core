"""Tests for ICSignalTracker — Information Coefficient tracking per scanner method."""
from __future__ import annotations

import json
import tempfile
import uuid
from pathlib import Path

import pytest

from atlas_code_quant.learning.ic_signal_tracker import (
    ICSignalTracker,
    _ic_status_label,
    _spearman_r,
)


# ── _spearman_r ────────────────────────────────────────────────────────────────

class TestSpearmanR:
    def test_empty_returns_zero(self):
        rho, t = _spearman_r([], [])
        assert rho == 0.0
        assert t == 0.0

    def test_single_element_returns_zero(self):
        rho, t = _spearman_r([1.0], [2.0])
        assert rho == 0.0

    def test_perfect_positive_correlation(self):
        x = [1.0, 2.0, 3.0, 4.0, 5.0]
        y = [1.0, 2.0, 3.0, 4.0, 5.0]
        rho, t = _spearman_r(x, y)
        assert abs(rho - 1.0) < 1e-6

    def test_perfect_negative_correlation(self):
        x = [1.0, 2.0, 3.0, 4.0, 5.0]
        y = [5.0, 4.0, 3.0, 2.0, 1.0]
        rho, t = _spearman_r(x, y)
        assert abs(rho - (-1.0)) < 1e-6

    def test_no_correlation(self):
        x = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
        y = [3.0, 1.0, 4.0, 1.0, 5.0, 9.0]
        rho, _ = _spearman_r(x, y)
        # Just verify result is bounded
        assert -1.0 <= rho <= 1.0

    def test_t_stat_grows_with_n(self):
        """Higher n → higher t-stat for same rho."""
        # Perfect correlation: rho=1, t=inf conceptually, but with finite n: t grows
        x_small = [1.0, 2.0, 3.0, 4.0]
        x_large = list(range(1, 21))
        _, t_small = _spearman_r(x_small, x_small)
        _, t_large = _spearman_r(x_large, x_large)
        assert t_large > t_small

    def test_mismatched_lengths_returns_zero(self):
        rho, t = _spearman_r([1.0, 2.0], [1.0])
        assert rho == 0.0
        assert t == 0.0

    def test_ties_handled(self):
        x = [1.0, 1.0, 2.0, 2.0]
        y = [1.0, 2.0, 3.0, 4.0]
        rho, _ = _spearman_r(x, y)
        assert -1.0 <= rho <= 1.0

    def test_constant_x_returns_zero(self):
        x = [5.0, 5.0, 5.0]
        y = [1.0, 2.0, 3.0]
        rho, t = _spearman_r(x, y)
        assert rho == 0.0
        assert t == 0.0


# ── _ic_status_label ──────────────────────────────────────────────────────────

class TestIcStatusLabel:
    def test_insufficient_data(self):
        assert _ic_status_label(0.08, 2.5, 3) == "insufficient_data"

    def test_not_significant(self):
        assert _ic_status_label(0.08, 1.5, 10) == "not_significant"

    def test_negative(self):
        assert _ic_status_label(-0.08, 2.5, 10) == "negative"

    def test_weak(self):
        assert _ic_status_label(0.03, 2.5, 10) == "weak"

    def test_meaningful(self):
        assert _ic_status_label(0.07, 2.5, 10) == "meaningful"

    def test_strong(self):
        assert _ic_status_label(0.12, 3.5, 20) == "strong"


# ── ICSignalTracker ────────────────────────────────────────────────────────────

@pytest.fixture
def tmp_tracker(tmp_path):
    return ICSignalTracker(tracker_path=tmp_path / "ic_tracker.json")


class TestICSignalTracker:
    def test_record_signal_returns_sid(self, tmp_tracker):
        sid = tmp_tracker.record_signal(
            symbol="AAPL", method="trend_ema_stack",
            predicted_move_pct=2.5, entry_price=175.0,
        )
        assert isinstance(sid, str) and len(sid) > 0

    def test_record_signal_persists(self, tmp_tracker):
        sid = tmp_tracker.record_signal(
            symbol="AAPL", method="trend_ema_stack",
            predicted_move_pct=2.5, entry_price=175.0,
        )
        # Reload from disk
        t2 = ICSignalTracker(tracker_path=tmp_tracker._path)
        assert sid in t2._state["signals"]

    def test_update_outcome_true(self, tmp_tracker):
        sid = tmp_tracker.record_signal(
            symbol="TSLA", method="breakout_donchian",
            predicted_move_pct=3.0, entry_price=200.0,
        )
        ok = tmp_tracker.update_outcome(signal_id=sid, exit_price=206.0)
        assert ok is True
        sig = tmp_tracker._state["signals"][sid]
        assert sig["outcome_available"] is True
        assert abs(sig["actual_return_pct"] - 3.0) < 0.01

    def test_update_outcome_false_unknown_id(self, tmp_tracker):
        ok = tmp_tracker.update_outcome(signal_id="nonexistent", exit_price=100.0)
        assert ok is False

    def test_update_outcome_zero_entry(self, tmp_tracker):
        sid = tmp_tracker.record_signal(
            symbol="SPY", method="momentum",
            predicted_move_pct=1.0, entry_price=0.0,  # bad entry
        )
        ok = tmp_tracker.update_outcome(signal_id=sid, exit_price=100.0)
        assert ok is False

    def test_compute_ic_insufficient_data(self, tmp_tracker):
        result = tmp_tracker.compute_ic()
        assert result["ic_status"] == "insufficient_data"
        assert result["ic"] is None

    def test_compute_ic_with_enough_data(self, tmp_tracker):
        symbols = ["A", "B", "C", "D", "E", "F"]
        predicted = [3.0, 2.0, 1.0, -1.0, -2.0, -3.0]
        actual_prices = [103.0, 102.0, 101.0, 99.0, 98.0, 97.0]  # perfectly aligned
        entries = [100.0] * 6
        sids = []
        for sym, pred, entry in zip(symbols, predicted, entries):
            sid = tmp_tracker.record_signal(
                symbol=sym, method="test_method",
                predicted_move_pct=pred, entry_price=entry,
            )
            sids.append(sid)
        for sid, exit_p in zip(sids, actual_prices):
            tmp_tracker.update_outcome(signal_id=sid, exit_price=exit_p)

        result = tmp_tracker.compute_ic(method="test_method")
        assert result["ic"] is not None
        assert result["ic"] > 0.9  # near-perfect correlation
        assert result["n_observations"] == 6

    def test_summary_structure(self, tmp_tracker):
        summary = tmp_tracker.summary()
        assert "overall" in summary
        assert "by_method" in summary
        assert "total_signals" in summary
        assert "signals_with_outcome" in summary
        assert "benchmark_reference" in summary

    def test_pending_outcome_signals_all(self, tmp_tracker):
        tmp_tracker.record_signal(
            symbol="NVDA", method="trend_ema_stack",
            predicted_move_pct=1.5, entry_price=500.0,
        )
        pending = tmp_tracker.pending_outcome_signals()
        assert len(pending) == 1
        assert pending[0]["symbol"] == "NVDA"

    def test_pending_outcome_signals_empty_after_update(self, tmp_tracker):
        sid = tmp_tracker.record_signal(
            symbol="MSFT", method="trend_ema_stack",
            predicted_move_pct=1.0, entry_price=300.0,
        )
        tmp_tracker.update_outcome(signal_id=sid, exit_price=303.0)
        pending = tmp_tracker.pending_outcome_signals()
        assert len(pending) == 0

    def test_purge_old_signals(self, tmp_path):
        tracker = ICSignalTracker(tracker_path=tmp_path / "purge_test.json")
        # Inject a signal with old recorded_at
        sid = str(uuid.uuid4())
        tracker._state["signals"][sid] = {
            "signal_id": sid,
            "symbol": "OLD",
            "method": "old_method",
            "predicted_move_pct": 1.0,
            "entry_price": 100.0,
            "timeframe": "1d",
            "selection_score": 0.5,
            "recorded_at": "2020-01-01T00:00:00+00:00",
            "actual_return_pct": None,
            "measured_at": None,
            "outcome_available": False,
        }
        tracker._save()
        deleted = tracker.purge_old_signals(keep_days=30)
        assert deleted == 1
        assert sid not in tracker._state["signals"]

    def test_custom_signal_id(self, tmp_tracker):
        custom_id = "my-custom-id-001"
        sid = tmp_tracker.record_signal(
            symbol="META", method="breakout_donchian",
            predicted_move_pct=2.0, entry_price=400.0,
            signal_id=custom_id,
        )
        assert sid == custom_id
        assert custom_id in tmp_tracker._state["signals"]

    def test_compute_ic_method_filter(self, tmp_tracker):
        """IC para método específico no debe mezclarse con otros."""
        for sym, method, pred in [
            ("A", "method_a", 2.0),
            ("B", "method_a", -2.0),
            ("C", "method_b", 1.0),
            ("D", "method_b", -1.0),
        ]:
            sid = tmp_tracker.record_signal(
                symbol=sym, method=method,
                predicted_move_pct=pred, entry_price=100.0,
            )
            exit_p = 102.0 if pred > 0 else 98.0
            tmp_tracker.update_outcome(signal_id=sid, exit_price=exit_p)

        # Need min_sample=2 to test with small sets
        result_a = tmp_tracker.compute_ic(method="method_a", min_sample=2)
        result_b = tmp_tracker.compute_ic(method="method_b", min_sample=2)
        assert result_a["n_observations"] == 2
        assert result_b["n_observations"] == 2

    def test_state_reloads_correctly(self, tmp_path):
        """Datos persistidos en disco deben cargarse en nueva instancia."""
        path = tmp_path / "reload_test.json"
        t1 = ICSignalTracker(tracker_path=path)
        sid = t1.record_signal(
            symbol="AMD", method="trend_ema_stack",
            predicted_move_pct=1.0, entry_price=100.0,
        )
        t1.update_outcome(signal_id=sid, exit_price=101.0)

        t2 = ICSignalTracker(tracker_path=path)
        assert sid in t2._state["signals"]
        assert t2._state["signals"][sid]["outcome_available"] is True
