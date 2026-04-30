from __future__ import annotations

from atlas_code_quant.options.paper_entry_planner import PaperEntryPlanner


def test_planner_returns_executable_size_with_sufficient_capital():
    planner = PaperEntryPlanner()
    intent = {
        "symbol": "IWM",
        "allow_entry": True,
        "force_no_trade": False,
        "recommended_strategy": "bull_call_debit_spread",
        "selector_inputs": {},
        "strategy_candidates": ["bull_call_debit_spread"],
    }
    out = planner.build_entry_plan(intent, capital=10_000.0, briefing={})
    assert out["entry"] == "proposed"
    assert out["position_size_units"] is not None
    assert int(out["position_size_units"]) >= 1
    assert out["size_blocked_reason"] is None


def test_planner_returns_blocked_reason_when_capital_too_low():
    planner = PaperEntryPlanner()
    intent = {
        "symbol": "IWM",
        "allow_entry": True,
        "force_no_trade": False,
        "recommended_strategy": "iron_condor",
        "selector_inputs": {},
        "strategy_candidates": ["iron_condor"],
    }
    out = planner.build_entry_plan(
        intent,
        capital=100.0,
        briefing={},
        manual_entry_overrides={"risk_per_unit_dollars": 250.0},
    )
    assert out["entry"] == "blocked"
    assert out["position_size_units"] is None
    assert out["size_blocked_reason"] in {"min_contract_not_reached", "insufficient_capital"}
