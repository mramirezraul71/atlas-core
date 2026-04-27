"""Atlas Code Quant — Live readiness checklist (F20).

Construye un informe (``LiveChecklistResult``) con el estado de las
condiciones necesarias para poder *armar* live trading en el futuro.

**No activa live**. El orquestador F17 sigue rechazando transiciones a
``LIVE_ARMED`` / ``LIVE_EXECUTING`` (``assert_transition(allow_live=False)``).

Items evaluados (todos deben ser ``ok=True`` para que ``overall_ok``
sea True):

1. ``env_live_requested`` — ``ATLAS_ENV=live`` y
   ``ATLAS_LIVETRADINGENABLED=true``.
2. ``tradier_not_dry_run`` — ``ATLAS_TRADIERDRYRUN=false``.
3. ``risk_limits_configured`` — todos los límites > 0.
4. ``killswitch_path_configured`` — fichero de killswitch resoluble.
5. ``killswitch_clear`` — fichero NO presente / vacío.
6. ``vision_available_or_optional`` — cámara ok o vision no requerida.
7. ``broker_paper_constructible`` — ``TradierAdapter`` dry-run construye.
8. ``fsm_paper_only_invariant`` — ``LIVE_FORBIDDEN_STATES`` siguen prohibidos.

``overall_ok`` siendo True es **necesario pero no suficiente**: además
hace falta un arming explícito (no implementado en F20).
"""

from __future__ import annotations

import logging
from dataclasses import dataclass, field
from typing import Any, Mapping, Sequence

from atlas_code_quant.autonomy.states import (
    LIVE_FORBIDDEN_STATES,
    AutonomyState,
)
from atlas_code_quant.config.live_readiness import (
    LiveReadinessEnv,
    is_live_mode_requested,
    is_live_mode_safe_to_arm,
    load_live_readiness_env,
)
from atlas_code_quant.risk.kill_switch import FileKillSwitch

logger = logging.getLogger("atlas.code_quant.autonomy.live_checklist")


__all__ = [
    "LiveChecklistItem",
    "LiveChecklistResult",
    "build_live_checklist",
]


@dataclass(frozen=True)
class LiveChecklistItem:
    name: str
    ok: bool
    reason: str
    evidence: Mapping[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class LiveChecklistResult:
    overall_ok: bool
    items: Sequence[LiveChecklistItem]
    env: LiveReadinessEnv

    def to_dict(self) -> dict:
        return {
            "overall_ok": self.overall_ok,
            "items": [
                {
                    "name": it.name,
                    "ok": it.ok,
                    "reason": it.reason,
                    "evidence": dict(it.evidence),
                }
                for it in self.items
            ],
        }


def _check_env_live_requested(env: LiveReadinessEnv) -> LiveChecklistItem:
    ok = is_live_mode_requested(env)
    return LiveChecklistItem(
        name="env_live_requested",
        ok=ok,
        reason="atlas_env_live_and_flag_true" if ok else "atlas_env_not_live",
        evidence={
            "atlas_env": env.atlas_env,
            "live_trading_enabled": env.live_trading_enabled,
        },
    )


def _check_tradier_not_dry_run(env: LiveReadinessEnv) -> LiveChecklistItem:
    ok = not env.tradier_dry_run
    return LiveChecklistItem(
        name="tradier_not_dry_run",
        ok=ok,
        reason="tradier_dry_run_disabled" if ok else "tradier_dry_run_active",
        evidence={"tradier_dry_run": env.tradier_dry_run},
    )


def _check_risk_limits_configured(env: LiveReadinessEnv) -> LiveChecklistItem:
    ok = (
        env.max_daily_loss_usd > 0
        and env.max_position_notional_usd > 0
        and env.max_orders_per_minute > 0
    )
    return LiveChecklistItem(
        name="risk_limits_configured",
        ok=ok,
        reason="risk_limits_positive" if ok else "risk_limits_invalid",
        evidence={
            "max_daily_loss_usd": env.max_daily_loss_usd,
            "max_position_notional_usd": env.max_position_notional_usd,
            "max_orders_per_minute": env.max_orders_per_minute,
        },
    )


def _check_killswitch_path_configured(
    env: LiveReadinessEnv,
) -> LiveChecklistItem:
    ok = bool(env.killswitch_file)
    return LiveChecklistItem(
        name="killswitch_path_configured",
        ok=ok,
        reason="killswitch_path_set" if ok else "killswitch_path_missing",
        evidence={"path": env.killswitch_file},
    )


def _check_killswitch_clear(
    env: LiveReadinessEnv, *, switch: FileKillSwitch | None = None
) -> LiveChecklistItem:
    ks = switch or FileKillSwitch(path=env.killswitch_file)
    try:
        status = ks.status()
    except Exception as exc:  # noqa: BLE001
        return LiveChecklistItem(
            name="killswitch_clear",
            ok=False,
            reason=f"killswitch_status_raised:{exc}",
            evidence={"path": env.killswitch_file},
        )
    return LiveChecklistItem(
        name="killswitch_clear",
        ok=not status.activated,
        reason="killswitch_clear" if not status.activated else status.reason,
        evidence={"path": status.path, "marker": status.raw_marker or ""},
    )


def _check_vision_available_or_optional(
    env: LiveReadinessEnv,
    *,
    camera_available: bool | None = None,
) -> LiveChecklistItem:
    if not env.vision_required_for_live:
        return LiveChecklistItem(
            name="vision_available_or_optional",
            ok=True,
            reason="vision_not_required",
        )
    if camera_available is None:
        # Defensivo: si no se inyecta probe explícito, asumimos no
        # disponible para no marcar safe espuriamente.
        return LiveChecklistItem(
            name="vision_available_or_optional",
            ok=False,
            reason="vision_required_but_camera_status_unknown",
        )
    return LiveChecklistItem(
        name="vision_available_or_optional",
        ok=bool(camera_available),
        reason=(
            "camera_available"
            if camera_available
            else "camera_unavailable_but_required"
        ),
        evidence={"camera_available": bool(camera_available)},
    )


def _check_broker_paper_constructible() -> LiveChecklistItem:
    try:
        from atlas_code_quant.execution.tradier_adapter import (
            TradierAdapter,
            TradierAdapterConfig,
        )

        adapter = TradierAdapter(TradierAdapterConfig(dry_run=True))
        ok = adapter.config.dry_run is True
    except Exception as exc:  # noqa: BLE001
        return LiveChecklistItem(
            name="broker_paper_constructible",
            ok=False,
            reason=f"broker_unavailable:{exc}",
        )
    return LiveChecklistItem(
        name="broker_paper_constructible",
        ok=ok,
        reason="broker_paper_ok" if ok else "broker_not_dry_run",
    )


def _check_fsm_paper_only_invariant() -> LiveChecklistItem:
    forbidden = set(LIVE_FORBIDDEN_STATES)
    expected = {AutonomyState.LIVE_ARMED, AutonomyState.LIVE_EXECUTING}
    ok = expected.issubset(forbidden)
    return LiveChecklistItem(
        name="fsm_paper_only_invariant",
        ok=ok,
        reason="live_states_forbidden" if ok else "live_states_unguarded",
        evidence={"forbidden": sorted(s.value for s in forbidden)},
    )


def build_live_checklist(
    *,
    env: LiveReadinessEnv | None = None,
    camera_available: bool | None = None,
    switch: FileKillSwitch | None = None,
) -> LiveChecklistResult:
    """Construye la checklist completa.

    No muta env. No activa live. Sólo evalúa.
    """
    e = env or load_live_readiness_env()
    items: list[LiveChecklistItem] = [
        _check_env_live_requested(e),
        _check_tradier_not_dry_run(e),
        _check_risk_limits_configured(e),
        _check_killswitch_path_configured(e),
        _check_killswitch_clear(e, switch=switch),
        _check_vision_available_or_optional(
            e, camera_available=camera_available
        ),
        _check_broker_paper_constructible(),
        _check_fsm_paper_only_invariant(),
    ]
    overall = all(it.ok for it in items)
    # Sanity: si overall_ok=True pero is_live_mode_safe_to_arm devuelve
    # False, forzamos overall_ok=False (defensivo).
    if overall and not is_live_mode_safe_to_arm(e):
        overall = False
    return LiveChecklistResult(overall_ok=overall, items=tuple(items), env=e)
