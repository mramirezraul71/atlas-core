"""Declarative runtime config v4 validation."""
from __future__ import annotations

from typing import Any

from pydantic import BaseModel, Field, model_validator

from atlas_code_quant.operations.runtime_mode import RuntimeMode


class RuntimeConfigV4(BaseModel):
    deployment_mode: str = "dev"
    allowed_runtime_modes: list[str] = Field(
        default_factory=lambda: [
            RuntimeMode.PAPER_BASELINE.value,
            RuntimeMode.PAPER_AGGRESSIVE.value,
            RuntimeMode.SUPERVISED_LIVE.value,
            RuntimeMode.GUARDED_LIVE.value,
            RuntimeMode.FULL_LIVE.value,
        ]
    )
    live_unlock_policy: str = "human_approval_required"
    readiness_thresholds: dict[str, float] = Field(default_factory=dict)
    strategy_mode_overrides: dict[str, str] = Field(default_factory=dict)
    symbol_mode_overrides: dict[str, str] = Field(default_factory=dict)
    broker_adapter_enabled: bool = False
    require_human_unlock: bool = True
    require_dual_confirmation: bool = False
    full_live_globally_locked: bool = True

    @model_validator(mode="after")
    def _validate_modes(self) -> "RuntimeConfigV4":
        valid_modes = {mode.value for mode in RuntimeMode}
        cleaned = [str(m).strip().lower() for m in self.allowed_runtime_modes]
        for mode in cleaned:
            if mode not in valid_modes:
                raise ValueError(f"Invalid runtime mode in allowed_runtime_modes: {mode}")
        if self.deployment_mode in {"dev", "test"} and RuntimeMode.FULL_LIVE.value in cleaned:
            raise ValueError("FULL_LIVE cannot be allowed in dev/test deployment_mode")
        if self.require_dual_confirmation and not self.require_human_unlock:
            raise ValueError("require_dual_confirmation requires require_human_unlock=true")
        return self


def validate_runtime_config_v4(payload: dict[str, Any] | None) -> RuntimeConfigV4:
    return RuntimeConfigV4.model_validate(payload or {})
