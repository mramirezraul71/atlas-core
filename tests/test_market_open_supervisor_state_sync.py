from __future__ import annotations

import json
from pathlib import Path

from atlas_code_quant.scripts import market_open_supervisor as supervisor


def test_run_checks_publishes_premarket_contract(monkeypatch, tmp_path: Path) -> None:
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
                "autonomy_mode": "safe",
                "max_risk_per_trade_pct": 0.02,
            },
            indent=2,
        ),
        encoding="utf-8",
    )

    monkeypatch.setattr(supervisor, "is_market_open_window", lambda *args, **kwargs: True)
    monkeypatch.setattr(supervisor, "check_push", lambda *args, **kwargs: supervisor.ServiceCheck("push", True, "push"))
    monkeypatch.setattr(supervisor, "check_quant", lambda *args, **kwargs: supervisor.ServiceCheck("quant", True, "quant"))
    monkeypatch.setattr(supervisor, "check_robot", lambda *args, **kwargs: supervisor.ServiceCheck("robot", True, "robot"))
    monkeypatch.setattr(supervisor, "read_robot_safe_mode", lambda *args, **kwargs: False)
    monkeypatch.setattr(supervisor, "_start_autonomy_orchestrator", lambda: {"ok": True})
    monkeypatch.setattr(supervisor, "quant_operation_state_path", lambda root=None: quant_path)

    captured: dict[str, object] = {}

    def _update(state: dict, *, quant_state_path: Path | None = None, core_state_path: Path | None = None, source_module: str = "", ensure_ascii: bool = False):
        from atlas_code_quant.operations.operation_state_contract import update_quant_state

        captured["source_module"] = source_module
        return update_quant_state(
            state,
            quant_state_path=quant_state_path,
            core_state_path=core_path,
            source_module=source_module,
            ensure_ascii=ensure_ascii,
        )

    monkeypatch.setattr(supervisor, "update_quant_state", _update)

    payload = supervisor.run_checks()
    quant_state = json.loads(quant_path.read_text(encoding="utf-8"))
    core_state = json.loads(core_path.read_text(encoding="utf-8"))

    assert payload["autonomy_orchestrator_start"]["ok"] is True
    assert quant_state["paper_bypass_exit_now_guard"] is True
    assert quant_state["paper_bypass_entry_validation_guard"] is True
    assert quant_state["paper_market_hours_strict"] is True
    assert core_state["auton_mode"] == "paper_supervised"
    assert core_state["source_module"] == "atlas_market_open_supervisor"
    assert captured["source_module"] == "atlas_market_open_supervisor"
