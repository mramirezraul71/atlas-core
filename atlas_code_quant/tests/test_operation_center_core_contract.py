from __future__ import annotations

import json
import sys
from pathlib import Path

QUANT_ROOT = Path(__file__).resolve().parents[1]
if str(QUANT_ROOT) not in sys.path:
    sys.path.insert(0, str(QUANT_ROOT))

from operations.operation_center import OperationCenter


def test_operation_center_publishes_core_contract(tmp_path: Path) -> None:
    quant_state = tmp_path / "quant" / "operation_center_state.json"
    core_state = tmp_path / "core" / "operation_center_state.json"

    center = OperationCenter(state_path=quant_state, core_state_path=core_state)
    center.update_config(
        {
            "account_scope": "paper",
            "auton_mode": "paper_autonomous",
            "executor_mode": "paper_api",
            "vision_mode": "off",
            "autonomy_mode": "safe",
            "max_risk_per_trade_pct": 0.015,
            "reset_fail_safe": True,
        }
    )

    payload = json.loads(core_state.read_text(encoding="utf-8"))
    assert payload["source_module"] == "atlas_code_quant"
    assert payload["contract_version"] == 1
    assert payload["auton_mode"] == "paper_autonomous"
    assert payload["autonomy_mode"] == "safe"
    assert payload["max_risk_per_trade_pct"] == 0.015
    assert payload["fail_safe_active"] is False


def test_operation_center_contract_accepts_paper_aggressive(tmp_path: Path) -> None:
    quant_state = tmp_path / "quant" / "operation_center_state.json"
    core_state = tmp_path / "core" / "operation_center_state.json"
    center = OperationCenter(state_path=quant_state, core_state_path=core_state)
    center.update_config({"auton_mode": "paper_aggressive", "reset_fail_safe": True})
    payload = json.loads(core_state.read_text(encoding="utf-8"))
    assert payload["auton_mode"] == "paper_aggressive"


def test_operation_center_imports_core_contract_on_load(tmp_path: Path) -> None:
    quant_state = tmp_path / "quant" / "operation_center_state.json"
    core_state = tmp_path / "core" / "operation_center_state.json"
    quant_state.parent.mkdir(parents=True, exist_ok=True)
    core_state.parent.mkdir(parents=True, exist_ok=True)

    quant_state.write_text(
        json.dumps(
            {
                "auton_mode": "paper_supervised",
                "autonomy_mode": "semi",
                "max_risk_per_trade_pct": 0.02,
                "fail_safe_active": False,
                "fail_safe_reason": None,
            },
            indent=2,
        ),
        encoding="utf-8",
    )
    core_state.write_text(
        json.dumps(
            {
                "contract_version": 1,
                "source_module": "atlas_core",
                "auton_mode": "off",
                "autonomy_mode": "safe",
                "max_risk_per_trade_pct": 0.005,
                "fail_safe_active": True,
                "fail_safe_reason": "brain_pause_trading",
            },
            indent=2,
        ),
        encoding="utf-8",
    )

    center = OperationCenter(state_path=quant_state, core_state_path=core_state)
    config = center.get_config()

    assert config["auton_mode"] == "off"
    assert config["autonomy_mode"] == "safe"
    assert config["max_risk_per_trade_pct"] == 0.005
    assert config["fail_safe_active"] is True
    assert config["fail_safe_reason"] == "brain_pause_trading"


def test_operation_center_ignores_quant_mirror_on_load(tmp_path: Path) -> None:
    quant_state = tmp_path / "quant" / "operation_center_state.json"
    core_state = tmp_path / "core" / "operation_center_state.json"
    quant_state.parent.mkdir(parents=True, exist_ok=True)
    core_state.parent.mkdir(parents=True, exist_ok=True)

    quant_state.write_text(
        json.dumps(
            {
                "auton_mode": "paper_autonomous",
                "autonomy_mode": "safe",
                "max_risk_per_trade_pct": 0.02,
            },
            indent=2,
        ),
        encoding="utf-8",
    )
    core_state.write_text(
        json.dumps(
            {
                "contract_version": 1,
                "source_module": "atlas_code_quant",
                "auton_mode": "off",
                "autonomy_mode": "semi",
                "max_risk_per_trade_pct": 0.005,
            },
            indent=2,
        ),
        encoding="utf-8",
    )

    center = OperationCenter(state_path=quant_state, core_state_path=core_state)
    config = center.get_config()

    assert config["auton_mode"] == "paper_autonomous"
    assert config["autonomy_mode"] == "safe"
    assert config["max_risk_per_trade_pct"] == 0.02
