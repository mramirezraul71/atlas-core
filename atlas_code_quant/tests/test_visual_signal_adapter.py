"""Tests para signals.visual_signal_adapter."""
from __future__ import annotations

from atlas_code_quant.signals.visual_signal_adapter import VisualSignalAdapter


class TestVisualSignalAdapter:
    def test_short_put_breach_0dte(self):
        adapter = VisualSignalAdapter(breach_tolerance=0.001)  # 0.1%
        ctx = adapter.analyze_price_context(
            spot=4980.0,
            short_put_strike=5000.0,
            short_call_strike=5100.0,
            is_0dte=True,
        )
        assert ctx["short_strike_breached"] is True
        assert ctx["critical_breach"] is True
        assert ctx["risk_break"] is True
        assert ctx["breach_detected"] is True
        assert ctx["details"]["breached_side"] == "put"
        assert ctx["details"]["breached_level_type"] == "short_strike"

    def test_short_call_breach_0dte(self):
        adapter = VisualSignalAdapter(breach_tolerance=0.0)
        ctx = adapter.analyze_price_context(
            spot=5125.0,
            short_put_strike=5000.0,
            short_call_strike=5100.0,
            is_0dte=True,
        )
        assert ctx["short_strike_breached"] is True
        assert ctx["critical_breach"] is True
        assert ctx["risk_break"] is True
        assert ctx["breach_detected"] is True
        assert ctx["details"]["breached_side"] == "call"

    def test_support_breach_without_short_strikes(self):
        adapter = VisualSignalAdapter()
        ctx = adapter.analyze_price_context(
            spot=4880.0,
            support_levels=[4900.0, 4950.0],
            resistance_levels=[5050.0, 5100.0],
            is_0dte=False,
        )
        assert ctx["breach_detected"] is True
        assert ctx["details"]["breached_level_type"] == "support"
        assert ctx["level_broken"] == 4900.0
        assert ctx["critical_breach"] is False

    def test_resistance_breach_without_short_strikes(self):
        adapter = VisualSignalAdapter()
        ctx = adapter.analyze_price_context(
            spot=5125.0,
            support_levels=[4900.0, 4950.0],
            resistance_levels=[5050.0, 5100.0],
            is_0dte=False,
        )
        assert ctx["breach_detected"] is True
        assert ctx["details"]["breached_level_type"] == "resistance"
        assert ctx["level_broken"] == 5100.0
        assert ctx["critical_breach"] is False

    def test_no_breach_case(self):
        adapter = VisualSignalAdapter(breach_tolerance=0.001)
        ctx = adapter.analyze_price_context(
            spot=5030.0,
            short_put_strike=5000.0,
            short_call_strike=5100.0,
            support_levels=[4980.0, 4990.0],
            resistance_levels=[5110.0, 5120.0],
            is_0dte=True,
        )
        assert ctx["breach_detected"] is False
        assert ctx["short_strike_breached"] is False
        assert ctx["critical_breach"] is False
        assert ctx["risk_break"] is False
        assert ctx["level_broken"] is None

    def test_output_schema_integrity(self):
        adapter = VisualSignalAdapter()
        ctx = adapter.analyze_price_context(spot=5000.0, is_0dte=False)
        for key in ("critical_breach", "short_strike_breached", "breach_detected", "level_broken", "risk_break", "details"):
            assert key in ctx
        for key in (
            "spot",
            "short_put_strike",
            "short_call_strike",
            "breached_side",
            "breached_level_type",
            "breached_level",
            "breach_magnitude_pct",
            "is_0dte",
            "notes",
        ):
            assert key in ctx["details"]
