from __future__ import annotations

import json
from pathlib import Path

import atlas_mcp_server as mcp


def test_get_state_returns_quant_and_core(monkeypatch, tmp_path: Path) -> None:
    quant_path = tmp_path / "quant" / "operation_center_state.json"
    core_path = tmp_path / "core" / "operation_center_state.json"
    quant_path.parent.mkdir(parents=True, exist_ok=True)
    core_path.parent.mkdir(parents=True, exist_ok=True)
    quant_path.write_text(json.dumps({"auton_mode": "paper_autonomous"}, indent=2), encoding="utf-8")
    core_path.write_text(json.dumps({"autonomy_mode": "safe"}, indent=2), encoding="utf-8")

    monkeypatch.setattr(mcp, "QUANT_STATE_PATH", quant_path)
    monkeypatch.setattr(mcp, "CORE_STATE_PATH", core_path)

    payload = mcp._get_state()

    assert payload["path"] == str(quant_path)
    assert payload["state"]["auton_mode"] == "paper_autonomous"
    assert payload["core_path"] == str(core_path)
    assert payload["core_state"]["autonomy_mode"] == "safe"


def test_set_state_updates_quant_and_core_contract(monkeypatch, tmp_path: Path) -> None:
    quant_path = tmp_path / "quant" / "operation_center_state.json"
    core_path = tmp_path / "core" / "operation_center_state.json"
    quant_path.parent.mkdir(parents=True, exist_ok=True)
    core_path.parent.mkdir(parents=True, exist_ok=True)
    quant_path.write_text(
        json.dumps(
            {
                "account_scope": "paper",
                "paper_only": True,
                "auton_mode": "paper_autonomous",
                "executor_mode": "paper_api",
                "vision_mode": "off",
                "operator_present": True,
                "screen_integrity_ok": True,
                "fail_safe_active": False,
                "fail_safe_reason": None,
                "autonomy_mode": "safe",
                "max_risk_per_trade_pct": 0.02,
            },
            indent=2,
        ),
        encoding="utf-8",
    )

    monkeypatch.setattr(mcp, "QUANT_STATE_PATH", quant_path)
    monkeypatch.setattr(mcp, "CORE_STATE_PATH", core_path)

    result = mcp.execute_action("set_state", {"key": "fail_safe_active", "value": True, "reason": "telegram_pause"})

    quant_state = json.loads(quant_path.read_text(encoding="utf-8"))
    core_state = json.loads(core_path.read_text(encoding="utf-8"))

    assert result["state"]["fail_safe_active"] is True
    assert quant_state["fail_safe_active"] is True
    assert quant_state["fail_safe_reason"] == "telegram_pause"
    assert core_state["fail_safe_active"] is True
    assert core_state["fail_safe_reason"] == "telegram_pause"
    assert core_state["source_module"] == "atlas_operator_interface"
    assert core_state["contract_version"] == 1
