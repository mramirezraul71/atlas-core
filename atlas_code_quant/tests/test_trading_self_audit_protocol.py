from atlas_code_quant.learning.trading_self_audit_protocol import (
    TRADING_ANALYSIS_SCHEMA,
    TRADING_PROCESS_GUARDRAILS,
    build_stage_audit_blueprint,
    build_trading_self_audit_note,
)


def test_trading_self_audit_note_contains_shared_schema_and_stage_map() -> None:
    note = build_trading_self_audit_note()

    assert note["current_focus"] == "post_trade_learning"
    assert len(note["analysis_schema"]) >= 9
    assert len(note["implementation_scorecard_metrics"]) >= 6
    assert len(note["external_benchmark_scan_flow"]) >= 6
    assert len(note["external_benchmark_sources"]) >= 17
    assert note["process_guardrails"] == TRADING_PROCESS_GUARDRAILS
    assert len(note["stage_map"]) == len(note["lifecycle"])
    assert note["entry_validation_focus"]["current_focus"] == "entry_validation"
    assert note["visual_entry_benchmark_focus"]["current_focus"] == "visual_entry_optimization"
    assert note["execution_quality_focus"]["current_focus"] == "execution_quality"
    assert note["position_management_focus"]["current_focus"] == "position_management"
    assert note["exit_governance_focus"]["current_focus"] == "exit_governance"
    assert note["post_trade_learning_focus"]["current_focus"] == "post_trade_learning"
    assert "breakout confirmation" in " ".join(note["visual_entry_benchmark_focus"]["external_benchmark_focus"]).lower()

    stages = {stage["stage"] for stage in note["stage_map"]}
    assert {
        "scanner_selection",
        "entry_validation",
        "execution_quality",
        "position_management",
        "exit_governance",
        "post_trade_learning",
    }.issubset(stages)


def test_stage_blueprint_reuses_common_analysis_schema() -> None:
    blueprint = build_stage_audit_blueprint("post_trade_learning")

    assert blueprint["stage"] == "post_trade_learning"
    assert blueprint["analysis_schema"] == TRADING_ANALYSIS_SCHEMA
    assert "policy promotion" in " ".join(blueprint["external_benchmark_focus"]).lower()
    assert len(blueprint["guardrails"]) >= 3
