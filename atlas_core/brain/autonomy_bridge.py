"""Compatibility bridge between legacy autonomy package and canonical Brain Core."""
from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import Any

from .models import ModuleState, RiskState

logger = logging.getLogger("atlas.brain.autonomy_bridge")


@dataclass
class BridgeStatus:
    canonical_controller: str = "brain"
    legacy_control_enabled: bool = False
    note: str = "autonomy bridge is compatibility/read-only by default"


class AutonomyBridge:
    """Read legacy autonomy telemetry without enabling dual control planes."""

    def __init__(self, state_bus: Any | None = None, registry: Any | None = None, allow_legacy_control: bool = False) -> None:
        self._state_bus = state_bus
        self._registry = registry
        self._allow_legacy_control = allow_legacy_control

    def status(self) -> BridgeStatus:
        return BridgeStatus(legacy_control_enabled=self._allow_legacy_control)

    def collect_legacy_snapshot(self) -> Any | None:
        if self._state_bus is None:
            return None
        try:
            # legacy state bus exposes collect_snapshot()
            return self._state_bus.collect_snapshot()
        except Exception as exc:
            logger.warning("autonomy_bridge snapshot error: %s", exc)
            return None

    def as_brain_states(self) -> tuple[dict[str, ModuleState], dict[str, RiskState]]:
        snap = self.collect_legacy_snapshot()
        out_states: dict[str, ModuleState] = {}
        out_risks: dict[str, RiskState] = {}
        if snap is None:
            return out_states, out_risks

        modules = getattr(snap, "modules", {}) if snap is not None else {}
        if not isinstance(modules, dict):
            return out_states, out_risks

        for name, payload in modules.items():
            if not isinstance(payload, dict):
                continue
            state = payload.get("state")
            risks = payload.get("risks")
            if state is not None:
                h = str(getattr(state, "health", "degraded")).lower()
                health = h if h in {"ok", "degraded", "critical"} else "degraded"
                out_states[name] = ModuleState(
                    name=str(getattr(state, "name", name)),
                    health=health,  # type: ignore[arg-type]
                    mode=str(getattr(state, "mode", "legacy")),
                    details=dict(getattr(state, "details", {}) or {}),
                )
            if risks is not None:
                raw = dict(getattr(risks, "risks", {}) or {})
                level = "low"
                # lightweight heuristic
                if any(str(v).lower() in {"critical", "true"} for v in raw.values()):
                    level = "critical"
                elif any(str(v).lower() in {"high", "warning"} for v in raw.values()):
                    level = "high"
                out_risks[name] = RiskState(name=name, level=level, details=raw)
        return out_states, out_risks

    def apply_legacy_command(self, command: Any) -> dict[str, Any]:
        if not self._allow_legacy_control:
            return {"ok": False, "error": "legacy_control_disabled_brain_is_canonical"}
        if self._registry is None:
            return {"ok": False, "error": "legacy_registry_unavailable"}
        try:
            target = getattr(command, "target", None) or command.get("target")
            mod = self._registry.get(target)
            if mod is None:
                return {"ok": False, "error": "legacy_unknown_target"}
            return mod.apply_command(command)
        except Exception as exc:
            logger.exception("legacy command bridge error")
            return {"ok": False, "error": str(exc)}

