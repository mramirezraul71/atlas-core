"""Runtime mode resolver V4 for live-capable but live-locked architecture."""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Any


class RuntimeMode(str, Enum):
    PAPER_BASELINE = "paper_baseline"
    PAPER_AGGRESSIVE = "paper_aggressive"
    SUPERVISED_LIVE = "supervised_live"
    GUARDED_LIVE = "guarded_live"
    FULL_LIVE = "full_live"


_ALLOWED_TRANSITIONS: dict[RuntimeMode, set[RuntimeMode]] = {
    RuntimeMode.PAPER_BASELINE: {
        RuntimeMode.PAPER_BASELINE,
        RuntimeMode.PAPER_AGGRESSIVE,
        RuntimeMode.SUPERVISED_LIVE,
    },
    RuntimeMode.PAPER_AGGRESSIVE: {
        RuntimeMode.PAPER_BASELINE,
        RuntimeMode.PAPER_AGGRESSIVE,
        RuntimeMode.SUPERVISED_LIVE,
        RuntimeMode.GUARDED_LIVE,
    },
    RuntimeMode.SUPERVISED_LIVE: {
        RuntimeMode.PAPER_BASELINE,
        RuntimeMode.PAPER_AGGRESSIVE,
        RuntimeMode.SUPERVISED_LIVE,
        RuntimeMode.GUARDED_LIVE,
    },
    RuntimeMode.GUARDED_LIVE: {
        RuntimeMode.PAPER_AGGRESSIVE,
        RuntimeMode.SUPERVISED_LIVE,
        RuntimeMode.GUARDED_LIVE,
        RuntimeMode.FULL_LIVE,
    },
    RuntimeMode.FULL_LIVE: {
        RuntimeMode.GUARDED_LIVE,
        RuntimeMode.FULL_LIVE,
    },
}


@dataclass(frozen=True)
class RuntimeResolution:
    effective_mode: RuntimeMode
    requested_mode: RuntimeMode
    policy_variant: str
    symbol: str
    account_scope: str
    deployment_scope: str
    reasons: list[str]


def _to_mode(value: str | RuntimeMode | None, *, fallback: RuntimeMode) -> RuntimeMode:
    if isinstance(value, RuntimeMode):
        return value
    token = str(value or "").strip().lower()
    for mode in RuntimeMode:
        if mode.value == token:
            return mode
    return fallback


def validate_runtime_mode_transition(current_mode: RuntimeMode, target_mode: RuntimeMode) -> bool:
    return target_mode in _ALLOWED_TRANSITIONS.get(current_mode, {current_mode})


def resolve_runtime_mode(
    *,
    requested_mode: str | RuntimeMode | None,
    policy_variant: str | None,
    symbol: str | None,
    account_scope: str | None,
    deployment_scope: str | None,
    allowed_runtime_modes: list[str] | None = None,
    strategy_mode_overrides: dict[str, str] | None = None,
    symbol_mode_overrides: dict[str, str] | None = None,
) -> RuntimeResolution:
    policy = str(policy_variant or "baseline_v1").strip().lower()
    sym = str(symbol or "").strip().upper()
    scope = str(account_scope or "paper").strip().lower()
    deploy = str(deployment_scope or "dev").strip().lower()

    requested = _to_mode(requested_mode, fallback=RuntimeMode.PAPER_BASELINE)
    reasons: list[str] = []
    effective = requested

    allowed = (
        {_to_mode(v, fallback=RuntimeMode.PAPER_BASELINE) for v in (allowed_runtime_modes or [])}
        if allowed_runtime_modes
        else set(RuntimeMode)
    )
    if effective not in allowed:
        reasons.append("requested_mode_not_allowed_by_config")
        effective = RuntimeMode.PAPER_BASELINE

    sym_over = dict(symbol_mode_overrides or {})
    if sym and sym in sym_over:
        effective = _to_mode(sym_over.get(sym), fallback=effective)
        reasons.append("symbol_override_applied")

    strat_over = dict(strategy_mode_overrides or {})
    if policy and policy in strat_over:
        effective = _to_mode(strat_over.get(policy), fallback=effective)
        reasons.append("strategy_override_applied")

    if scope != "live" and effective in {
        RuntimeMode.SUPERVISED_LIVE,
        RuntimeMode.GUARDED_LIVE,
        RuntimeMode.FULL_LIVE,
    }:
        reasons.append("account_scope_not_live_downgraded_to_supervised_shadow")
        effective = RuntimeMode.SUPERVISED_LIVE

    if deploy in {"dev", "test"} and effective in {RuntimeMode.GUARDED_LIVE, RuntimeMode.FULL_LIVE}:
        reasons.append("deployment_scope_blocks_guarded_or_full_live")
        effective = RuntimeMode.SUPERVISED_LIVE

    return RuntimeResolution(
        effective_mode=effective,
        requested_mode=requested,
        policy_variant=policy,
        symbol=sym,
        account_scope=scope,
        deployment_scope=deploy,
        reasons=reasons,
    )


def runtime_resolution_to_event(resolution: RuntimeResolution) -> dict[str, Any]:
    return {
        "requested_mode": resolution.requested_mode.value,
        "effective_mode": resolution.effective_mode.value,
        "policy_variant": resolution.policy_variant,
        "symbol": resolution.symbol,
        "account_scope": resolution.account_scope,
        "deployment_scope": resolution.deployment_scope,
        "reasons": list(resolution.reasons),
    }
