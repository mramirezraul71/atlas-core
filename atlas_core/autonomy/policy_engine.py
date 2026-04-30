from __future__ import annotations

from typing import List

from .models import Command, Snapshot, AutonomyMode
from .module_registry import ModuleRegistry
from .state_bus import StateBus


class PolicyEngine:
    """
    Reglas globales de autonomía (simple, extensible).
    Más adelante se puede conectar aquí RL o un LLM local.
    """

    def __init__(
        self,
        registry: ModuleRegistry,
        state_bus: StateBus,
        config: dict | None = None,
    ) -> None:
        self._registry = registry
        self._state_bus = state_bus
        self._config = config or {
            "max_drawdown_pct": -0.08,
            "global_mode": "semi",  # manual | semi | auto | experimental
        }

    def evaluate(self) -> List[Command]:
        _snapshot: Snapshot = self._state_bus.collect_snapshot()
        cmds: List[Command] = []

        m = _snapshot.modules
        global_mode: AutonomyMode = self._config.get("global_mode", "semi")  # type: ignore[assignment]

        # En modo manual sólo observamos
        if global_mode == "manual":
            return []

        max_dd = float(self._config.get("max_drawdown_pct", -0.08))

        # Regla 1: drawdown demasiado grande → reducir riesgo Quant
        quant = m.get("quant")
        if quant:
            quant_risks = quant.get("risks")
            dd = None
            if quant_risks and hasattr(quant_risks, "risks"):
                dd = quant_risks.risks.get("drawdown_pct")
            if dd is not None:
                try:
                    dd_val = float(dd)
                except Exception:
                    dd_val = None
                if dd_val is not None and dd_val < max_dd:
                    cmds.append(
                        Command(
                            target="quant",
                            action="reduce_risk",
                            params={"factor": 0.5},
                        )
                    )

        # Regla 2: Robot en safe_mode crítico → poner Quant en modo seguro
        robot = m.get("robot")
        if robot:
            robot_risks = robot.get("risks")
            safe_mode_active = False
            if robot_risks and hasattr(robot_risks, "risks"):
                safe_mode_active = bool(robot_risks.risks.get("safe_mode_active"))
            if safe_mode_active:
                cmds.append(
                    Command(
                        target="quant",
                        action="set_mode",
                        params={"mode": "safe"},
                    )
                )

        return cmds

