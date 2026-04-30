"""Orquestación central: eventos → arbitraje → seguridad → ejecución."""
from __future__ import annotations

import logging
from collections import deque
from typing import TYPE_CHECKING, Any

from .arbitration import ArbitrationEngine
from .audit_log import AuditLog
from .command_router import CommandRouter
from .mission_manager import MissionManager
from .models import Command, Event, SystemSnapshot
from .safety_kernel import SafetyKernel
from .state_bus import StateBus
from .workspace_bridge import WorkspaceBridge

if TYPE_CHECKING:
    from ..runtime.mode_manager import ModeManager

logger = logging.getLogger("atlas.brain.core")


class BrainCore:
    def __init__(
        self,
        state_bus: StateBus,
        mission_manager: MissionManager,
        safety_kernel: SafetyKernel,
        arbitration: ArbitrationEngine,
        command_router: CommandRouter,
        mode_manager: ModeManager | None = None,
        audit_log: AuditLog | None = None,
        workspace_bridge: WorkspaceBridge | None = None,
    ) -> None:
        self._bus = state_bus
        self._missions = mission_manager
        self._safety = safety_kernel
        self._arbitration = arbitration
        self._router = command_router
        self._mode_manager = mode_manager
        self._audit = audit_log
        self._workspace = workspace_bridge
        self._events: deque[Event] = deque()

    def ingest_event(self, event: Event) -> None:
        self._events.append(event)
        self._bus.increment_meta_counter("events_ingested", 1)
        logger.debug("event ingested source=%s kind=%s", event.source, event.kind)
        if self._audit:
            self._audit.write("event_ingested", source=event.source, kind=event.kind, payload=event.payload)

    def set_mode(self, mode: str) -> None:
        if self._mode_manager is not None:
            self._mode_manager.set_mode(mode)
        else:
            self._bus.set_global_mode(mode)
        if self._audit:
            self._audit.write("brain_mode_changed", mode=mode)

    def get_snapshot(self) -> SystemSnapshot:
        return self._bus.get_snapshot()

    def pending_event_count(self) -> int:
        return len(self._events)

    def note_runtime_tick(self, queue_before: int, queue_after: int) -> None:
        self._bus.set_meta("event_queue_before", int(queue_before))
        self._bus.set_meta("event_queue_after", int(queue_after))
        self._bus.increment_meta_counter("event_loop_cycles", 1)

    def dispatch(self, command: Command) -> dict[str, Any]:
        snap = self._bus.get_snapshot()
        ok, why = self._safety.authorize(command, snap)
        if not ok:
            rec = {"ok": False, "error": why}
            self._bus.record_execution_result(command.target, rec)
            if self._audit:
                self._audit.write("command_blocked", command={"target": command.target, "action": command.action}, reason=why)
            return rec
        out = self._router.route(command)
        self._bus.record_execution_result(command.target, out)
        if self._audit:
            self._audit.write("command_dispatched", command={"target": command.target, "action": command.action}, result=out)
        return out

    def _commands_for_event(self, ev: Event) -> list[Command]:
        if ev.source == "operator" and ev.kind == "speak":
            return [Command(target="body", action="speak", params=dict(ev.payload))]
        if ev.source == "quant" and ev.kind == "risk_alert":
            return [Command(target="quant", action="set_mode", params={"mode": ev.payload.get("mode", "safe")})]
        if ev.source == "operator" and ev.kind == "set_mission":
            name = str(ev.payload.get("mission_name") or ev.payload.get("name") or "")
            pri = int(ev.payload.get("priority", 0))
            if name:
                self._missions.set_mission(name, priority=pri)
                self._bus.set_active_mission(self._missions.get_active_mission())
            return []
        return []

    def evaluate(self) -> list[dict[str, Any]]:
        started = self._bus.mark_loop_start("brain_eval")
        pending: list[Command] = []
        while self._events:
            ev = self._events.popleft()
            if ev.source == "system" and ev.kind == "critical_health":
                self._safety.trigger_fail_safe(str(ev.payload.get("reason", "critical_health")), self._bus)
                if self._audit:
                    self._audit.write("event_trigger_fail_safe", source=ev.source, kind=ev.kind, payload=ev.payload)
                continue
            pending.extend(self._commands_for_event(ev))

        if not pending:
            return []

        snap = self._bus.get_snapshot()
        resolved = self._arbitration.resolve(pending, snap)
        self._bus.set_meta("last_arbitration_count", len(resolved))
        results: list[dict[str, Any]] = []
        for cmd in resolved:
            pre_emit = None
            if self._workspace:
                pre_emit = self._workspace.emit_decision(cmd, authorized=False, stage="pre_authorization")
            snap = self._bus.get_snapshot()
            ok, why = self._safety.authorize(cmd, snap)
            if self._workspace:
                self._workspace.emit_decision(cmd, authorized=ok, stage="post_authorization", reason=None if ok else why)
            if not ok:
                rec = {"ok": False, "error": why}
                self._bus.record_execution_result(cmd.target, rec)
                results.append(rec)
                if self._audit:
                    self._audit.write("command_blocked", command={"target": cmd.target, "action": cmd.action}, reason=why)
                continue
            if snap.global_mode.lower().strip() in {"manual", "supervised"} and self._workspace and pre_emit:
                fb = self._workspace.wait_for_operator_feedback(pre_emit["decision_id"], timeout_sec=30.0)
                if not fb.get("approved", False):
                    rec = {"ok": False, "error": "operator_rejected"}
                    self._bus.record_execution_result(cmd.target, rec)
                    results.append(rec)
                    if self._audit:
                        self._audit.write(
                            "command_blocked",
                            command={"target": cmd.target, "action": cmd.action},
                            reason="operator_rejected",
                        )
                    continue
            out = self._router.route(cmd)
            self._bus.record_execution_result(cmd.target, out)
            results.append(out)
            if self._workspace:
                ev = self._workspace.analyze_execution_evidence(cmd, out)
                if ev.get("anomaly"):
                    self._workspace.trigger_healing_cascade(
                        self._router,
                        reason=f"anomaly_after_{cmd.target}.{cmd.action}",
                        evidence=ev,
                    )
            if self._audit:
                self._audit.write("command_result", command={"target": cmd.target, "action": cmd.action}, result=out)
        elapsed_ms = int((self._bus.mark_loop_end("brain_eval", started)) * 1000)
        self._bus.set_meta("last_decision_latency_ms", elapsed_ms)
        return results

    def evaluate_and_decide(self) -> list[dict[str, Any]]:
        """Alias semántico del ciclo principal para integración externa."""
        return self.evaluate()
