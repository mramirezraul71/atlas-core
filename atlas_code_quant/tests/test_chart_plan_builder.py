from __future__ import annotations

from atlas_code_quant.operations.chart_plan_builder import (
    build_selector_chart_plan,
    chart_interval_for_timeframe,
    chart_plan_probe_ok,
)


def test_chart_interval_defaults() -> None:
    assert chart_interval_for_timeframe("1h") == "60"
    assert chart_interval_for_timeframe("unknown") == "60"


def test_build_selector_chart_plan_tradingview_targets() -> None:
    plan = build_selector_chart_plan("SPY", "1h", "4h", "tradingview")
    assert plan["provider"] == "tradingview"
    assert len(plan["targets"]) == 3
    assert all("url" in t for t in plan["targets"])
    assert "SPY" in plan["targets"][0]["url"].upper() or "SPY" in plan["targets"][0]["title"].upper()
    ok, detail = chart_plan_probe_ok("SPY", "1h", "tradingview")
    assert ok is True
    assert detail == ""


def test_build_selector_chart_plan_yahoo_and_higher_tf_default() -> None:
    plan = build_selector_chart_plan("QQQ", "5m", None, "yahoo")
    assert plan["provider"] == "yahoo"
    assert plan["targets"][1]["timeframe"] == "1h"
    assert all(t["url"].startswith("https://finance.yahoo.com/") for t in plan["targets"])
    ok, _ = chart_plan_probe_ok("QQQ", "5m", "yahoo")
    assert ok is True


def test_chart_plan_probe_rejects_bad_symbol() -> None:
    ok, detail = chart_plan_probe_ok("", "1h", "tradingview")
    assert ok is False
    assert "símbolo" in detail.lower() or "vacío" in detail.lower()


def test_chart_plan_stable_with_whitespace_symbol() -> None:
    plan = build_selector_chart_plan("  spy  ", "15m", "4h", "tradingview")
    assert "SPY" in plan["targets"][0]["url"].upper()
    ok, _ = chart_plan_probe_ok("  spy  ", "15m", "tradingview")
    assert ok is True
