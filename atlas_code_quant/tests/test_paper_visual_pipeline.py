from __future__ import annotations

import numpy as np

from atlas_code_quant.operations.paper_visual_pipeline import (
    build_visual_review_contract,
    collect_visual_evidence,
)


def test_visual_review_contract_required_fields() -> None:
    frame = type(
        "VF",
        (),
        {
            "ok": True,
            "frame": np.zeros((40, 60, 3), dtype=np.uint8),
            "provider_used": "desktop_capture",
            "used_fallback": True,
            "fallback_reason": "chart_launcher_unavailable",
            "screenshot_path": "x.png",
        },
    )()
    out = build_visual_review_contract(
        symbol="SPY",
        timeframe="5m",
        opportunity_id="opp_test",
        trace_id="tr_test",
        frame=frame,
        provider_payload={"confidence": 0.66, "summary": "breakout", "tags": ["volume"]},
        candidate={"direction": "long", "strategy_type": "iron_condor", "spread_pct": 0.12},
    )
    assert out.symbol == "SPY"
    assert out.timeframe == "5m"
    assert out.chart_bias in {"bullish", "bearish", "neutral", "mixed"}
    assert isinstance(out.support_levels, list)
    assert isinstance(out.resistance_levels, list)
    assert out.estimated_slippage_bps >= 0
    assert 0.0 <= out.partial_fill_risk <= 1.0


def test_collect_visual_evidence_includes_visual_decision_context(monkeypatch, tmp_path) -> None:
    class _FakeFrame:
        def __init__(self):
            self.ok = True
            self.frame = np.zeros((32, 48, 3), dtype=np.uint8)
            self.provider_used = "tradingview_chart_launcher"
            self.used_fallback = False
            self.fallback_reason = ""
            self.screenshot_path = str(tmp_path / "chart.png")
            self.metadata = {"symbol": "SPY", "timeframe": "5m"}

    class _FakeProvider:
        def get_chart_frame(self, **kwargs):
            return _FakeFrame()

        def get_screen_frame(self):
            return _FakeFrame()

    monkeypatch.setattr(
        "atlas_code_quant.operations.paper_visual_pipeline.VisualInputProvider",
        lambda: _FakeProvider(),
    )
    monkeypatch.setattr(
        "atlas_code_quant.operations.paper_visual_pipeline.analyze_dashboard_screenshot",
        lambda path: {"confidence": 0.8, "summary": "breakout", "tags": ["volume"]},
    )
    out = collect_visual_evidence(
        symbol="SPY",
        timeframe="5m",
        opportunity_id="opp_x",
        trace_id="tr_x",
        chart_source="tradingview",
        candidate={"direction": "long", "strategy_type": "iron_condor", "spread_pct": 0.12},
        chart_plan={"provider": "tradingview"},
        camera_plan={"required": True},
    )
    assert out["capture_ok"] is True
    assert out["symbol_verified"] is True
    assert out["timeframe_verified"] is True
    assert isinstance(out["visual_decision_context"], dict)
    assert out["visual_decision_context"]["confluence_bucket"] in {"high", "medium", "low", "unknown"}
    assert "fusion_decision_reason" in out
    assert out["recommended_action"] in {
        "execute",
        "reduce_aggressiveness",
        "postpone",
        "discard",
        "manual_review",
    }
    assert isinstance(out["seasonality_context"], dict)
    assert isinstance(out["multi_timeframe_view"], dict)
    assert isinstance(out["operational_context"], dict)
