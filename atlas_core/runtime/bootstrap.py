"""Ensamblado del Brain Core y adapters."""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

from ..adapters.body_controller import BodyControllerAdapter
from ..adapters.healing_adapter import HealingBrainAdapter
from ..adapters.operator_interface_adapter import OperatorInterfaceAdapter
from ..adapters.quant_adapter import QuantBrainAdapter
from ..adapters.system_health_adapter import SystemHealthAdapter
from ..adapters.vision_adapter import VisionBrainAdapter
from ..brain.arbitration import ArbitrationEngine
from ..brain.audit_log import AuditLog
from ..brain.autonomy_bridge import AutonomyBridge
from ..brain.brain_core import BrainCore
from ..brain.command_router import CommandRouter
from ..brain.mission_manager import MissionManager
from ..brain.policy_store import PolicyStore
from ..brain.safety_kernel import SafetyKernel
from ..brain.state_bus import StateBus
from ..brain.workspace_bridge import WorkspaceBridge
from .mode_manager import ModeManager


@dataclass
class BrainRuntime:
    state_bus: StateBus
    mission_manager: MissionManager
    mode_manager: ModeManager
    safety_kernel: SafetyKernel
    arbitration: ArbitrationEngine
    command_router: CommandRouter
    brain: BrainCore
    adapters: dict[str, Any]
    policy_store: PolicyStore
    audit_log: AuditLog
    autonomy_bridge: AutonomyBridge
    workspace_bridge: WorkspaceBridge


def build_brain_system(
    operation_center_path: Path | None = None,
    policy_override: dict | None = None,
    legacy_autonomy_state_bus: Any | None = None,
    legacy_autonomy_registry: Any | None = None,
) -> BrainRuntime:
    state_bus = StateBus()
    audit_log_path = operation_center_path.with_name("brain_audit_log.jsonl") if operation_center_path else None
    audit_log = AuditLog(path=audit_log_path)
    mission_manager = MissionManager()
    policy_store = PolicyStore(policy_override)
    safety = SafetyKernel(policy_store.as_dict(), state_bus=state_bus, audit_log=audit_log)
    arbitration = ArbitrationEngine()
    router = CommandRouter()

    body = BodyControllerAdapter()
    vision = VisionBrainAdapter()
    quant = QuantBrainAdapter(operation_state_path=operation_center_path, audit_log=audit_log)
    healing = HealingBrainAdapter()
    system_health = SystemHealthAdapter()
    operator = OperatorInterfaceAdapter()

    adapters: dict[str, Any] = {
        body.name: body,
        vision.name: vision,
        quant.name: quant,
        healing.name: healing,
        system_health.name: system_health,
        operator.name: operator,
    }

    autonomy_bridge = AutonomyBridge(
        state_bus=legacy_autonomy_state_bus,
        registry=legacy_autonomy_registry,
        allow_legacy_control=False,
    )
    workspace_bridge = WorkspaceBridge(audit_log=audit_log)

    for name, ad in adapters.items():
        router.register_adapter(name, ad)

    mode_manager = ModeManager(state_bus)
    brain = BrainCore(
        state_bus=state_bus,
        mission_manager=mission_manager,
        safety_kernel=safety,
        arbitration=arbitration,
        command_router=router,
        mode_manager=mode_manager,
        audit_log=audit_log,
        workspace_bridge=workspace_bridge,
    )

    mode_manager.set_mode(str(policy_store.get("default_mode", "semi")))

    return BrainRuntime(
        state_bus=state_bus,
        mission_manager=mission_manager,
        mode_manager=mode_manager,
        safety_kernel=safety,
        arbitration=arbitration,
        command_router=router,
        brain=brain,
        adapters=adapters,
        policy_store=policy_store,
        audit_log=audit_log,
        autonomy_bridge=autonomy_bridge,
        workspace_bridge=workspace_bridge,
    )
