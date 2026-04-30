from __future__ import annotations


def test_registry_and_state_bus_collects_snapshot() -> None:
    from atlas_core.autonomy.module_registry import ModuleRegistry
    from atlas_core.autonomy.state_bus import StateBus
    from atlas_core.autonomy.models import Command, ModuleRisks, ModuleState

    class FakeModule:
        name = "fake"

        def get_capabilities(self) -> dict:
            return {"type": "fake"}

        def get_state(self) -> ModuleState:
            return ModuleState(name=self.name, mode="semi", health="ok", details={})

        def get_risks(self) -> ModuleRisks:
            return ModuleRisks(name=self.name, risks={"drawdown_pct": -0.01})

        def apply_command(self, command: Command) -> dict:
            return {"ok": True, "action": command.action}

    registry = ModuleRegistry()
    registry.register(FakeModule())

    bus = StateBus(registry)
    snapshot = bus.collect_snapshot()

    assert snapshot.modules["fake"]["state"].name == "fake"
    assert snapshot.modules["fake"]["risks"].risks["drawdown_pct"] == -0.01
    assert snapshot.modules["fake"]["capabilities"]["type"] == "fake"

