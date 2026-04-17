"""Quant adapter using hardened shared state access (paper-safe)."""
from __future__ import annotations

import logging
from pathlib import Path
from typing import Any

from ..brain.models import Command, ModuleState, RiskState
from ..brain.audit_log import AuditLog
from ..brain.shared_state import SharedStateStore

logger = logging.getLogger("atlas.brain.adapter.quant")


def _default_operation_state_path() -> Path:
    repo_root = Path(__file__).resolve().parents[2]
    return repo_root / "data" / "operation" / "operation_center_state.json"


class QuantBrainAdapter:
    name = "quant"

    def __init__(self, operation_state_path: Path | None = None, audit_log: AuditLog | None = None) -> None:
        self._path = operation_state_path or _default_operation_state_path()
        self._audit = audit_log
        self._shared = SharedStateStore(self._path, audit_log=audit_log)

    @staticmethod
    def _defaults() -> dict[str, Any]:
        # setdefault-only policy
        return {
            "autonomy_mode": "semi",
            "max_risk_per_trade_pct": 0.02,
        }

    def _load_merge_defaults(self) -> dict[str, Any]:
        self._shared.ensure_defaults(self._defaults())
        return self._shared.read_state()

    @staticmethod
    def _core_contract_meta() -> dict[str, Any]:
        return {
            "contract_version": 1,
            "source_module": "atlas_core",
        }

    def get_state(self) -> ModuleState:
        raw = self._load_merge_defaults()
        mode = str(raw.get("autonomy_mode", "semi"))
        fail = bool(raw.get("fail_safe_active"))
        health: Any = "ok"
        if fail:
            health = "degraded"
        return ModuleState(
            name=self.name,
            health=health,
            mode=mode,
            details={"paper": True, "path": str(self._path)},
        )

    def get_risks(self) -> RiskState:
        raw = self._load_merge_defaults()
        lvl: Any = "low"
        if raw.get("fail_safe_active"):
            lvl = "high"
        mrp = float(raw.get("max_risk_per_trade_pct") or 0.02)
        if mrp > 0.1:
            lvl = "medium"
        return RiskState(name=self.name, level=lvl, details={"max_risk_per_trade_pct": mrp})

    def apply_command(self, command: Command) -> dict[str, Any]:
        a = command.action.lower()
        self._shared.ensure_defaults(self._defaults())
        if a == "set_mode":
            mode = str(command.params.get("mode", "semi"))
            out = self._shared.write_state(lambda d: {**d, "autonomy_mode": mode, **self._core_contract_meta()})
            if self._audit:
                self._audit.write("quant_command", action="set_mode", mode=mode)
            return {"ok": True, "autonomy_mode": out.get("autonomy_mode")}
        if a == "reduce_risk":
            factor = float(command.params.get("factor", 0.5))

            def _mut(d: dict[str, Any]) -> dict[str, Any]:
                cur = float(d.get("max_risk_per_trade_pct") or 0.02)
                d["max_risk_per_trade_pct"] = max(0.001, round(cur * factor, 6))
                d.update(self._core_contract_meta())
                return d

            out = self._shared.write_state(_mut)
            if self._audit:
                self._audit.write("quant_command", action="reduce_risk", factor=factor)
            return {"ok": True, "max_risk_per_trade_pct": out.get("max_risk_per_trade_pct")}
        if a == "pause_trading":
            out = self._shared.write_state(
                lambda d: {
                    **d,
                    "auton_mode": "off",
                    "fail_safe_active": True,
                    "fail_safe_reason": "brain_pause_trading",
                    **self._core_contract_meta(),
                }
            )
            if self._audit:
                self._audit.write("quant_command", action="pause_trading")
            return {"ok": True, "paused": True, "auton_mode": out.get("auton_mode")}
        return {"ok": False, "error": f"unsupported:{a}"}
