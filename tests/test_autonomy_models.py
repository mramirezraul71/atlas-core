from __future__ import annotations


def test_autonomy_models_basic_types() -> None:
    from atlas_core.autonomy.models import (
        AutonomyMode,
        Command,
        ModuleRisks,
        ModuleState,
        Snapshot,
    )

    state = ModuleState(
        name="quant",
        mode="semi",  # type: ignore[arg-type]
        health="ok",
        details={"roi_pct": 0.12},
    )
    risks = ModuleRisks(name="quant", risks={"drawdown_pct": -0.2})
    cmd = Command(target="quant", action="reduce_risk", params={"factor": 0.5})
    snap = Snapshot(modules={"quant": {"state": state, "risks": risks, "capabilities": {}}})

    assert isinstance(state.name, str)
    assert state.health in {"ok", "degraded", "critical"}
    assert isinstance(risks.risks, dict)
    assert isinstance(cmd.params, dict)
    assert isinstance(snap.modules, dict)

    # Type is Literal-based; runtime just needs acceptability.
    assert snap.global_meta == {}

