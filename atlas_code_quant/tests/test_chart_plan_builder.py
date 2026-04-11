from __future__ import annotations

from atlas_code_quant.operations.chart_plan_builder import build_selector_chart_plan, chart_interval_for_timeframe


def test_chart_interval_defaults() -> None:
    assert chart_interval_for_timeframe("1h") == "60"
    assert chart_interval_for_timeframe("unknown") == "60"


def test_build_selector_chart_plan_tradingview_targets() -> None:
    plan = build_selector_chart_plan("SPY", "1h", "4h", "tradingview")
    assert plan["provider"] == "tradingview"
    assert len(plan["targets"]) == 3
    assert all("url" in t for t in plan["targets"])
    assert "SPY" in plan["targets"][0]["url"].upper() or "SPY" in plan["targets"][0]["title"].upper()
