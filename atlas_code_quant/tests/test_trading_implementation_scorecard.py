import json
import sqlite3
from datetime import datetime, timezone, timedelta
from pathlib import Path

import pytest

from atlas_code_quant.learning.trading_implementation_scorecard import build_trading_implementation_scorecard


def _fresh_checked_at() -> str:
    """Returns a checked_at timestamp always within the 6-hour freshness window."""
    return (datetime.now(timezone.utc) - timedelta(hours=1)).isoformat()


def _write_text(path: Path, content: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(content, encoding="utf-8")


def _build_minimal_repo(root: Path) -> tuple[Path, Path, Path]:
    _write_text(root / "atlas_code_quant/learning/trading_self_audit_protocol.py", "# protocol\n")
    _write_text(root / "atlas_code_quant/scripts/record_trading_self_audit_note.py", "# script\n")
    _write_text(root / "reports/atlas_quant_trading_process_audit_framework_20260328.md", "# framework\n")
    _write_text(root / "reports/atlas_quant_scanner_metric_audit_20260328.md", "# scanner\n")
    _write_text(root / "reports/atlas_quant_entry_validation_audit_20260328.md", "# entry\n")
    _write_text(root / "reports/atlas_quant_execution_quality_audit_20260328.md", "# execution\n")
    _write_text(root / "reports/atlas_quant_position_management_audit_20260328.md", "# position management\n")
    _write_text(root / "reports/atlas_quant_exit_governance_audit_20260328.md", "# exit governance\n")
    _write_text(root / "reports/atlas_quant_post_trade_learning_audit_20260328.md", "# post trade learning\n")
    _write_text(root / "reports/atlas_quant_external_benchmark_confrontation_20260328.md", "# external\n")
    _write_text(root / "reports/atlas_quant_visual_benchmark_confrontation_20260329.md", "# visual benchmark\n")
    _write_text(root / "reports/atlas_quant_options_strategy_governance_20260329.md", "# options governance\n")
    _write_text(root / "atlas_code_quant/tests/test_trading_self_audit_protocol.py", "def test_placeholder():\n    assert True\n")
    _write_text(root / "atlas_code_quant/tests/test_trading_implementation_scorecard.py", "def test_placeholder():\n    assert True\n")
    _write_text(root / "atlas_code_quant/tests/test_position_management_snapshot.py", "def test_placeholder():\n    assert True\n")
    _write_text(root / "atlas_code_quant/tests/test_scanner_metric_recalibration.py", "def test_placeholder():\n    assert True\n")
    _write_text(root / "atlas_code_quant/tests/test_operation_center_status.py", "def test_placeholder():\n    assert True\n")
    _write_text(root / "atlas_code_quant/tests/test_operation_center_autonomous_guards.py", "def test_placeholder():\n    assert True\n")

    protocol = {
        "lifecycle": [
            {"stage": "scanner_selection", "status": "baseline_hardened"},
            {"stage": "entry_validation", "status": "baseline_hardened"},
            {"stage": "execution_quality", "status": "baseline_hardened"},
            {"stage": "position_management", "status": "baseline_hardened"},
            {"stage": "exit_governance", "status": "baseline_hardened"},
            {"stage": "post_trade_learning", "status": "active_focus"},
        ],
        "external_benchmark_sources": [
            {"used_for": ["scanner_selection"]},
            {"used_for": ["entry_validation"]},
            {"used_for": ["execution_quality"]},
            {"used_for": ["position_management"]},
            {"used_for": ["exit_governance"]},
            {"used_for": ["post_trade_learning"]},
            {"title": "visual-a", "used_for": ["visual_entry_optimization"]},
            {"title": "visual-b", "used_for": ["visual_entry_optimization"]},
            {"title": "visual-c", "used_for": ["visual_entry_optimization"]},
            {"title": "visual-d", "used_for": ["visual_entry_optimization"]},
            {"title": "visual-e", "used_for": ["visual_entry_optimization"]},
            {"title": "options-a", "used_for": ["options_strategy_governance"]},
            {"title": "options-b", "used_for": ["options_strategy_governance"]},
            {"title": "options-c", "used_for": ["options_strategy_governance"]},
        ],
        "visual_entry_benchmark_focus": {
            "current_focus": "visual_entry_optimization",
            "human_best_practice": ["contexto", "nivel", "trigger", "invalidacion"],
            "automation_translation": ["chart_plan", "expected_visual", "ocr", "gate"],
            "recommended_metrics": ["a", "b", "c", "d", "e", "f"],
            "web_feedback_loop": ["detectar", "buscar", "comparar", "persistir"],
        },
        "options_strategy_governance_focus": {
            "current_focus": "options_strategy_governance",
            "human_best_practice": ["thesis", "iv", "term structure", "hedge context"],
            "automation_translation": ["family", "governance", "constraints", "fallbacks"],
            "recommended_metrics": ["a", "b", "c", "d", "e", "f"],
            "web_feedback_loop": ["detectar", "buscar", "comparar", "persistir"],
        },
    }
    protocol_path = root / "reports/trading_self_audit_protocol.json"
    protocol_path.write_text(json.dumps(protocol), encoding="utf-8")

    brain_state = {
        "total_events": 10,
        "delivered_events": 6,
        "queued_local_only_events": 4,
        "last_memory_ok": True,
        "last_bitacora_ok": False,
        "events_path": str(root / "atlas_code_quant/logs/quant_brain_bridge.jsonl"),
    }
    brain_state_path = root / "atlas_code_quant/data/operation/quant_brain_bridge_state.json"
    brain_state_path.parent.mkdir(parents=True, exist_ok=True)
    brain_state_path.write_text(json.dumps(brain_state), encoding="utf-8")
    _write_text(
        root / "atlas_code_quant/logs/quant_brain_bridge.jsonl",
        json.dumps({"kind": "trading_self_audit_protocol"}) + "\n",
    )
    _write_text(
        root / "reports/atlas_grafana_provisioning_check_latest.json",
        json.dumps(
            {
                "checked_at": _fresh_checked_at(),
                "grafana_version": "10.4.2",
                "overall_status": "ready",
                "telegram_env_ready": False,
                "provisioning_rebuild": {"ok": True},
                "reload": {
                    "datasources": {"ok": True},
                    "dashboards": {"ok": False},
                    "alerting": {"ok": True},
                },
                "verification": {
                    "datasource_ok": True,
                    "dashboards_ok": True,
                    "alerting_ready": True,
                },
                "raw": {
                    "health": {"ok": True},
                },
            }
        ),
    )

    journal_path = root / "atlas_code_quant/data/journal/trading_journal.sqlite3"
    journal_path.parent.mkdir(parents=True, exist_ok=True)
    with sqlite3.connect(journal_path) as conn:
        conn.execute(
            "create table trading_journal (status text, strategy_type text, unrealized_pnl real, realized_pnl real)"
        )
        conn.executemany(
            "insert into trading_journal(status, strategy_type, unrealized_pnl, realized_pnl) values (?, ?, ?, ?)",
            [
                ("open", "equity_long", 10.0, 0.0),
                ("open", "untracked", -5.0, 0.0),
                ("closed", "equity_long", 0.0, 2.0),
                ("closed", "equity_short", 0.0, -1.0),
                ("closed", "equity_short", 0.0, 3.0),
                ("closed", "equity_long", 0.0, 1.0),
            ],
        )

    return protocol_path, journal_path, brain_state_path


def test_build_trading_implementation_scorecard_computes_headline_and_indicators(tmp_path: Path) -> None:
    protocol_path, journal_path, brain_state_path = _build_minimal_repo(tmp_path)

    payload = build_trading_implementation_scorecard(
        root=tmp_path,
        protocol_path=protocol_path,
        journal_db_path=journal_path,
        brain_state_path=brain_state_path,
        grafana_check_path=tmp_path / "reports/atlas_grafana_provisioning_check_latest.json",
        pytest_result=None,
    )

    assert payload["headline"]["atlas_process_compliance_score"] == pytest.approx(97.5)
    assert payload["metrics"]["artifact_coverage_score"]["value"] == 100.0
    assert payload["metrics"]["external_benchmark_coverage_score"]["value"] == 100.0
    assert payload["metrics"]["observability_feedback_score"]["value"] == pytest.approx(93.33)
    assert payload["metrics"]["visual_benchmark_feedback_score"]["value"] == 100.0
    assert payload["metrics"]["options_strategy_governance_feedback_score"]["value"] == 100.0
    assert payload["metrics"]["implementation_usefulness_score"]["value"] == pytest.approx(23.0)
    assert payload["supporting_indicators"]["attributed_open_positions_pct"] == 50.0
    assert payload["supporting_indicators"]["evidence_sufficiency_score"] == 20.0
    assert payload["supporting_indicators"]["grafana_alerting_ready_pct"] == 100.0
    assert payload["supporting_indicators"]["visual_benchmark_source_count"] == 5
    assert payload["supporting_indicators"]["options_governance_source_count"] == 3
    assert payload["metrics"]["implementation_usefulness_score"]["details"]["signal_ic_quality_score"] == 0.0
    assert payload["metrics"]["implementation_usefulness_score"]["details"]["paper_outcome_quality_score"] == 0.0
    assert payload["metrics"]["implementation_usefulness_score"]["details"]["usefulness_cap_reason"] == "paper_outcome_quality_weak"
    assert payload["next_actions"]
