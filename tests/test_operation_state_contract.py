from __future__ import annotations

import json
from pathlib import Path

from atlas_code_quant.operations.operation_state_contract import (
    combined_operation_state,
    update_quant_state,
)


def test_update_quant_state_publishes_core_contract(tmp_path: Path) -> None:
    quant_path = tmp_path / "quant" / "operation_center_state.json"
    core_path = tmp_path / "core" / "operation_center_state.json"
    quant_path.parent.mkdir(parents=True, exist_ok=True)

    quant_path.write_text(
        json.dumps(
            {
                "account_scope": "paper",
                "paper_only": True,
                "auton_mode": "paper_supervised",
                "executor_mode": "paper_api",
                "vision_mode": "off",
                "operator_present": True,
                "screen_integrity_ok": True,
                "fail_safe_active": False,
                "fail_safe_reason": None,
                "autonomy_mode": "semi",
                "max_risk_per_trade_pct": 0.02,
            },
            indent=2,
        ),
        encoding="utf-8",
    )

    result = update_quant_state(
        {
            "account_scope": "paper",
            "paper_only": True,
            "auton_mode": "paper_autonomous",
            "executor_mode": "paper_api",
            "vision_mode": "off",
            "operator_present": True,
            "screen_integrity_ok": True,
            "fail_safe_active": True,
            "fail_safe_reason": "test_pause",
            "autonomy_mode": "safe",
            "max_risk_per_trade_pct": 0.01,
        },
        quant_state_path=quant_path,
        core_state_path=core_path,
        source_module="test_harness",
    )

    payload = json.loads(core_path.read_text(encoding="utf-8"))
    assert result["state"]["auton_mode"] == "paper_autonomous"
    assert payload["autonomy_mode"] == "safe"
    assert payload["fail_safe_reason"] == "test_pause"
    assert payload["source_module"] == "test_harness"
    assert payload["contract_version"] == 1
    assert payload["updated_at"].endswith("Z")


def test_combined_operation_state_reads_both_views(tmp_path: Path) -> None:
    quant_path = tmp_path / "quant" / "operation_center_state.json"
    core_path = tmp_path / "core" / "operation_center_state.json"
    quant_path.parent.mkdir(parents=True, exist_ok=True)
    core_path.parent.mkdir(parents=True, exist_ok=True)
    quant_path.write_text(json.dumps({"auton_mode": "paper_autonomous"}), encoding="utf-8")
    core_path.write_text(json.dumps({"autonomy_mode": "safe"}), encoding="utf-8")

    bundle = combined_operation_state(quant_state_path=quant_path, core_state_path=core_path)

    assert bundle["path"] == str(quant_path)
    assert bundle["state"]["auton_mode"] == "paper_autonomous"
    assert bundle["core_path"] == str(core_path)
    assert bundle["core_state"]["autonomy_mode"] == "safe"
