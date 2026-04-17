from __future__ import annotations


def test_policy_engine_reduce_risk_on_drawdown() -> None:
    from atlas_core.autonomy.models import Command, ModuleRisks, ModuleState
    from atlas_core.autonomy.module_registry import ModuleRegistry
    from atlas_core.autonomy.policy_engine import PolicyEngine
    from atlas_core.autonomy.state_bus import StateBus

    class FakeQuant:
        name = "quant"

        def get_capabilities(self) -> dict:
            return {"type": "trading"}

        def get_state(self) -> ModuleState:
            return ModuleState(name=self.name, mode="semi", health="ok", details={})

        def get_risks(self) -> ModuleRisks:
            return ModuleRisks(name=self.name, risks={"drawdown_pct": -0.5})

        def apply_command(self, command: Command) -> dict:
            return {"ok": True}

    class FakeRobot:
        name = "robot"

        def get_capabilities(self) -> dict:
            return {"type": "robot"}

        def get_state(self) -> ModuleState:
            return ModuleState(name=self.name, mode="semi", health="ok", details={})

        def get_risks(self) -> ModuleRisks:
            return ModuleRisks(name=self.name, risks={"safe_mode_active": False})

        def apply_command(self, command: Command) -> dict:
            return {"ok": True}

    registry = ModuleRegistry()
    registry.register(FakeQuant())
    registry.register(FakeRobot())

    bus = StateBus(registry)
    engine = PolicyEngine(registry=registry, state_bus=bus, config={"global_mode": "semi", "max_drawdown_pct": -0.08})
    cmds = engine.evaluate()

    assert any(c.target == "quant" and c.action == "reduce_risk" for c in cmds)
    assert not any(c.target == "quant" and c.action == "set_mode" for c in cmds)


def test_policy_engine_set_quant_safe_mode_on_robot_safe() -> None:
    from atlas_core.autonomy.models import Command, ModuleRisks, ModuleState
    from atlas_core.autonomy.module_registry import ModuleRegistry
    from atlas_core.autonomy.policy_engine import PolicyEngine
    from atlas_core.autonomy.state_bus import StateBus

    class FakeQuant:
        name = "quant"

        def get_capabilities(self) -> dict:
            return {"type": "trading"}

        def get_state(self) -> ModuleState:
            return ModuleState(name=self.name, mode="semi", health="ok", details={})

        def get_risks(self) -> ModuleRisks:
            return ModuleRisks(name=self.name, risks={"drawdown_pct": -0.01})

        def apply_command(self, command: Command) -> dict:
            return {"ok": True}

    class FakeRobot:
        name = "robot"

        def get_capabilities(self) -> dict:
            return {"type": "robot"}

        def get_state(self) -> ModuleState:
            return ModuleState(name=self.name, mode="semi", health="ok", details={})

        def get_risks(self) -> ModuleRisks:
            return ModuleRisks(name=self.name, risks={"safe_mode_active": True})

        def apply_command(self, command: Command) -> dict:
            return {"ok": True}

    registry = ModuleRegistry()
    registry.register(FakeQuant())
    registry.register(FakeRobot())

    bus = StateBus(registry)
    engine = PolicyEngine(registry=registry, state_bus=bus, config={"global_mode": "semi", "max_drawdown_pct": -0.08})
    cmds = engine.evaluate()

    assert any(c.target == "quant" and c.action == "set_mode" and c.params.get("mode") == "safe" for c in cmds)

