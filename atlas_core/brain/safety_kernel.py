"""Safety Kernel: governance guardrails for Brain Core decisions."""
from __future__ import annotations

import logging
from typing import TYPE_CHECKING

from .models import Command, SystemSnapshot
from .audit_log import AuditLog

if TYPE_CHECKING:
    from .state_bus import StateBus

logger = logging.getLogger("atlas.brain.safety_kernel")


class SafetyKernel:
    def __init__(self, policy: dict | None = None, state_bus: StateBus | None = None, audit_log: AuditLog | None = None) -> None:
        from .policy_store import PolicyStore

        self._policy = PolicyStore(policy or {})
        self._state_bus = state_bus
        self._audit = audit_log

        self._allowed_by_mode: dict[str, set[str]] = {
            "manual": {"observe", "status", "speak", "notify", "confirm", "reject"},
            "safe": {"observe", "status", "speak", "notify", "confirm", "reject", "set_mode", "stop", "diagnose", "recover"},
            "recovery": {"observe", "status", "notify", "confirm", "reject", "set_mode", "diagnose", "recover", "stop"},
        }
        self._quant_aggressive_actions = {
            "open_trade",
            "submit",
            "increase_exposure",
            "aggressive_buy",
            "aggressive_sell",
            "reduce",  # explicit from request
        }
        self._body_physical_actions = {"move", "execute_motion", "actuate", "walk", "run", "grip"}

    def compute_global_risk(self, snapshot: SystemSnapshot) -> str:
        levels = {"low": 0, "medium": 1, "high": 2, "critical": 3}
        worst = "low"
        for r in snapshot.risks.values():
            if levels.get(r.level, 0) > levels.get(worst, 0):
                worst = r.level
        return worst

    def _reject(
        self,
        reason: str,
        command: Command,
        snapshot: SystemSnapshot,
        trigger_fail_safe: bool = False,
        apply_side_effects: bool = True,
    ) -> tuple[bool, str]:
        if trigger_fail_safe and apply_side_effects:
            self.trigger_fail_safe(reason, self._state_bus)
        if self._audit and apply_side_effects:
            self._audit.write(
                "safety_rejection",
                reason=reason,
                command={"target": command.target, "action": command.action, "params": command.params},
                global_mode=snapshot.global_mode,
            )
        return False, reason

    def _authorize_internal(self, command: Command, snapshot: SystemSnapshot, apply_side_effects: bool) -> tuple[bool, str]:
        mode = snapshot.global_mode.lower().strip()
        action = command.action.lower().strip()
        target = command.target.lower().strip()
        global_risk = self.compute_global_risk(snapshot)

        # 1) Mode-based baseline rules
        if mode in self._allowed_by_mode:
            allowed = set(self._allowed_by_mode[mode])
            if action not in allowed:
                return self._reject(f"{mode} mode blocks action '{action}'", command, snapshot, apply_side_effects=apply_side_effects)

        # 2) System health dominates everything
        sh = snapshot.risks.get("system_health")
        if sh and sh.level == "critical":
            if not (
                (target == "healing" and action in {"diagnose", "recover", "status"})
                or (target == "system_health" and action in {"status", "set_mode"})
                or (action == "set_mode" and str(command.params.get("mode", "")).lower() in {"safe", "recovery"})
                or target == "operator"
            ):
                return self._reject(
                    "system_health critical: only safe/recovery/status/healing/operator allowed",
                    command,
                    snapshot,
                    trigger_fail_safe=True,
                    apply_side_effects=apply_side_effects,
                )

        # 3) Degraded health blocks non-essential expensive actions
        if (
            self._policy.get("system_health_block_nonessential_on_degraded", True)
            and sh
            and sh.level in {"high", "medium"}
            and target == "quant"
            and action in {"open_trade", "submit", "increase_exposure"}
        ):
            return self._reject("system_health degraded: blocks expensive quant action", command, snapshot, apply_side_effects=apply_side_effects)

        # 4) Body local safety constraints
        body = snapshot.modules.get("body")
        if body and body.health == "critical" and self._policy.get("body_block_on_critical", True):
            if target == "body" and action not in {"stop", "status"}:
                return self._reject(
                    "body critical: only stop/status allowed",
                    command,
                    snapshot,
                    trigger_fail_safe=True,
                    apply_side_effects=apply_side_effects,
                )
        if target == "body" and action in self._body_physical_actions and mode in {"safe", "recovery"}:
            return self._reject(f"{mode} mode blocks physical body action '{action}'", command, snapshot, apply_side_effects=apply_side_effects)

        # 5) Quant local risk constraints
        qr = snapshot.risks.get("quant")
        if qr and qr.level == "critical" and self._policy.get("quant_block_on_high_risk", True):
            if target == "quant" and action in self._quant_aggressive_actions:
                return self._reject(
                    "quant critical risk blocks aggressive action",
                    command,
                    snapshot,
                    trigger_fail_safe=True,
                    apply_side_effects=apply_side_effects,
                )
        if mode == "recovery" and target == "quant" and action in {"open_trade", "submit", "increase_exposure"}:
            return self._reject("recovery mode blocks aggressive quant actions", command, snapshot, apply_side_effects=apply_side_effects)
        if mode == "safe" and target == "quant" and action in {"open_trade", "submit", "increase_exposure"}:
            return self._reject("safe mode blocks trading actions", command, snapshot, apply_side_effects=apply_side_effects)

        # 6) Global risk constraints
        if global_risk == "critical" and target in {"body", "quant"} and action not in {"status", "stop", "set_mode", "reduce_risk", "pause_trading"}:
            return self._reject(
                "global risk critical: restrictive command set",
                command,
                snapshot,
                trigger_fail_safe=True,
                apply_side_effects=apply_side_effects,
            )

        if self._audit and apply_side_effects:
            self._audit.write(
                "safety_authorized",
                command={"target": command.target, "action": command.action, "params": command.params},
                global_mode=snapshot.global_mode,
                global_risk=global_risk,
            )
        return True, "ok"

    def explain_rejection(self, command: Command, snapshot: SystemSnapshot) -> str:
        allowed, reason = self._authorize_internal(command, snapshot, apply_side_effects=False)
        if allowed:
            return "allowed"
        return reason

    def authorize(self, command: Command, snapshot: SystemSnapshot) -> tuple[bool, str]:
        return self._authorize_internal(command, snapshot, apply_side_effects=True)

    def trigger_fail_safe(self, reason: str, state_bus: StateBus | None) -> None:
        logger.warning("FAIL_SAFE: %s", reason)
        if self._audit:
            self._audit.write("safety_fail_safe_trigger", reason=reason)
        if state_bus is None:
            return
        state_bus.set_global_mode("safe")
        state_bus.append_fail_safe(reason)
