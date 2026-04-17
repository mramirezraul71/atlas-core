from __future__ import annotations


def test_quant_adapter_updates_operation_state(tmp_path) -> None:
    import json
    from atlas_core.autonomy.adapters.quant_adapter import QuantAutonomyAdapter
    from atlas_core.autonomy.models import Command

    summary_path = tmp_path / "iron_butterfly_summary.json"
    operation_state_path = tmp_path / "operation_center_state.json"

    summary_path.write_text(
        json.dumps(
            {
                "max_drawdown_pct": -0.2,
                "win_rate_pct": 0.55,
                "roi_pct": 0.12,
                "total_trades": 10,
            },
            indent=2,
        ),
        encoding="utf-8",
    )
    operation_state_path.write_text("{}", encoding="utf-8")

    adapter = QuantAutonomyAdapter(
        summary_path=summary_path,
        operation_state_path=operation_state_path,
    )

    adapter.apply_command(Command(target="quant", action="set_mode", params={"mode": "safe"}))
    updated = json.loads(operation_state_path.read_text(encoding="utf-8"))
    assert updated.get("autonomy_mode") == "safe"

    # reduce_risk default uses current=0.02 when key missing
    adapter.apply_command(Command(target="quant", action="reduce_risk", params={"factor": 0.5}))
    updated2 = json.loads(operation_state_path.read_text(encoding="utf-8"))
    assert abs(float(updated2["max_risk_per_trade_pct"]) - 0.01) < 1e-9

