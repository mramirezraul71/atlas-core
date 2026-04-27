"""Atlas Code Quant — Autonomy gates (F17).

Define la interfaz canónica ``Gate`` y los stubs F17 de los gates
del orquestador. Cada gate devuelve un ``GateResult`` (booleano,
razón legible, flag de degradación). En F17 todos los gates son
defensivos y nunca lanzan.

Gates incluidos en F17:

* ``HealthGate`` — comprueba salud básica (snapshot/canonical disponible).
* ``RadarGate`` — comprueba que hay oportunidades Radar aprobadas.
* ``StrategyGate`` — comprueba que F12/F14 son importables.
* ``RiskGate`` — stub: siempre OK (F19 lo conectará a `risk/limits`).
* ``VisionGate`` — stub: siempre OK (F18 lo conectará a VisionTimingGate).
* ``BrokerGate`` — comprueba que TradierAdapter dry-run construye OK.
* ``LiveGate`` — siempre ``ok=False`` en F17 (live bloqueado).
* ``KillSwitchGate`` — stub: siempre OK (F19 lo conectará a fichero).

Reglas duras (F17):

* No tocar ``execution/`` ni ``operations.live_*``.
* No leer ningún fichero real en disco (``KillSwitchGate`` se queda en
  stub hasta F19).
* No usar estos gates como autorización de live.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass, field
from typing import Any, Mapping

logger = logging.getLogger("atlas.code_quant.autonomy.gates")


@dataclass(frozen=True)
class GateResult:
    """Resultado canónico de cualquier gate del orquestador.

    Attributes:
        ok: True si el gate permite continuar.
        reason: explicación legible (siempre presente).
        degraded: True si el gate permite continuar pero en modo
            degradado (p.ej. cámara unavailable + intent no requiere
            confirmación visual).
        evidence: payload opcional (métricas, refs).
    """

    ok: bool
    reason: str
    degraded: bool = False
    evidence: Mapping[str, Any] = field(default_factory=dict)


class Gate:
    """Interfaz base para los gates del orquestador F17."""

    name: str = "gate"

    def evaluate(self, ctx: Mapping[str, Any] | None = None) -> GateResult:  # pragma: no cover - interfaz
        raise NotImplementedError

    # alias semántico
    def __call__(self, ctx: Mapping[str, Any] | None = None) -> GateResult:
        try:
            return self.evaluate(ctx or {})
        except Exception as exc:  # noqa: BLE001 — defensiva total
            logger.warning("%s.evaluate raised: %s", self.name, exc)
            return GateResult(
                ok=False, reason=f"{self.name}_raised:{exc}", degraded=True
            )


# ---------------------------------------------------------------------------
# Stubs F17
# ---------------------------------------------------------------------------


class HealthGate(Gate):
    name = "health"

    def evaluate(self, ctx: Mapping[str, Any] | None = None) -> GateResult:
        ctx = ctx or {}
        # Para F17: si el contexto explícitamente marca health=False,
        # consideramos sistema degradado. Por defecto OK.
        if ctx.get("health") is False:
            return GateResult(
                ok=False, reason="health_unavailable", degraded=True
            )
        return GateResult(ok=True, reason="health_ok")


class RadarGate(Gate):
    name = "radar"

    def evaluate(self, ctx: Mapping[str, Any] | None = None) -> GateResult:
        ctx = ctx or {}
        if ctx.get("radar_degraded") is True:
            return GateResult(
                ok=False, reason="radar_degraded", degraded=True
            )
        opps = ctx.get("opportunities", None)
        if opps is None:
            # contexto sin payload de Radar todavía → permitido pero
            # sin oportunidades concretas (el orquestador hará loop).
            return GateResult(ok=True, reason="radar_idle")
        try:
            count = len(opps)  # type: ignore[arg-type]
        except TypeError:
            count = 0
        return GateResult(
            ok=True,
            reason="radar_ok",
            evidence={"opportunity_count": count},
        )


class StrategyGate(Gate):
    name = "strategy"

    def evaluate(self, ctx: Mapping[str, Any] | None = None) -> GateResult:
        try:  # imports tardíos para no acoplar el módulo
            from atlas_code_quant.strategies.evaluation import (  # noqa: F401
                evaluate_strategies_for_opportunity,
            )
            from atlas_code_quant.strategies.factory.dispatch import (  # noqa: F401
                build_strategies_for_opportunity,
            )
        except Exception as exc:  # noqa: BLE001
            return GateResult(
                ok=False, reason=f"strategy_unavailable:{exc}", degraded=True
            )
        return GateResult(ok=True, reason="strategy_ok")


class RiskGate(Gate):
    """Stub F17 — F19 conectará con ``risk/limits`` y circuit breaker."""

    name = "risk"

    def evaluate(self, ctx: Mapping[str, Any] | None = None) -> GateResult:
        ctx = ctx or {}
        if ctx.get("risk_violation") is True:
            return GateResult(
                ok=False, reason="risk_violation_simulated", degraded=False
            )
        return GateResult(ok=True, reason="risk_ok_stub_f17")


class VisionGate(Gate):
    """Stub F17 — F18 conectará con ``VisionTimingGate``."""

    name = "vision"

    def evaluate(self, ctx: Mapping[str, Any] | None = None) -> GateResult:
        ctx = ctx or {}
        if ctx.get("vision_block") is True:
            return GateResult(ok=False, reason="vision_block_simulated")
        if ctx.get("vision_force_exit") is True:
            return GateResult(
                ok=False, reason="vision_force_exit_simulated"
            )
        return GateResult(ok=True, reason="vision_ok_stub_f17")


class BrokerGate(Gate):
    """Comprueba que ``TradierAdapter`` paper construye sin errores."""

    name = "broker"

    def evaluate(self, ctx: Mapping[str, Any] | None = None) -> GateResult:
        try:
            from atlas_code_quant.execution.tradier_adapter import (
                TradierAdapter,
                TradierAdapterConfig,
            )

            adapter = TradierAdapter(TradierAdapterConfig(dry_run=True))
            if not adapter.config.dry_run:
                return GateResult(
                    ok=False, reason="broker_not_dry_run", degraded=True
                )
        except Exception as exc:  # noqa: BLE001
            return GateResult(
                ok=False, reason=f"broker_unavailable:{exc}", degraded=True
            )
        return GateResult(ok=True, reason="broker_ok_paper")


class LiveGate(Gate):
    """En F17–F20 el live siempre está bloqueado.

    Este gate es lo último que protege ante un intento de
    transicionar a ``LIVE_ARMED`` / ``LIVE_EXECUTING``: siempre
    devuelve ``ok=False``.
    """

    name = "live"

    def evaluate(self, ctx: Mapping[str, Any] | None = None) -> GateResult:
        return GateResult(
            ok=False,
            reason="live_blocked_in_f17_f20_block",
            degraded=False,
            evidence={"policy": "paper_only"},
        )


class KillSwitchGate(Gate):
    """Stub F17 — F19 lo conectará a ``ATLAS_KILLSWITCH_FILE``."""

    name = "killswitch"

    def evaluate(self, ctx: Mapping[str, Any] | None = None) -> GateResult:
        ctx = ctx or {}
        if ctx.get("killswitch") is True:
            return GateResult(ok=False, reason="killswitch_triggered")
        return GateResult(ok=True, reason="killswitch_clear_stub_f17")


__all__ = [
    "Gate",
    "GateResult",
    "HealthGate",
    "RadarGate",
    "StrategyGate",
    "RiskGate",
    "VisionGate",
    "BrokerGate",
    "LiveGate",
    "KillSwitchGate",
]
