"""Atlas Code Quant — Autonomy orchestrator (F17).

FSM de 14 estados (estudio maestro v9 §11) operando **sólo en papel**
sobre el pipeline F16 (Radar → Strategy → fitness → Tradier dry-run).

Política F17–F20 (verbatim del usuario):

    Scanner propone. Radar decide. Estrategias y ejecución sólo
    operan sobre oportunidades aprobadas por Radar, en papel hasta
    que se defina un bloque posterior de autonomía y live readiness.

Reglas duras F17:

* La FSM nunca transiciona a ``LIVE_ARMED`` ni ``LIVE_EXECUTING``.
* ``step()`` es defensivo: nunca lanza hacia fuera; cualquier
  excepción interna se convierte en transición a ``ERROR_RECOVERY``.
* No toca ``execution/`` directamente (sólo a través del pipeline F16).
* No lee ficheros (``KillSwitchGate`` queda stub hasta F19).
* No autoriza live: aunque llegue ``LiveArmRequested``, el
  orquestador lo registra y lo rechaza.
"""

from __future__ import annotations

import logging
import os
from datetime import datetime, timezone
from dataclasses import dataclass, field
from typing import Any, Iterable, Mapping

from atlas_code_quant.autonomy.events import (
    AutonomyEvent,
    DegradationDetected,
    KillSwitchTriggered,
    LiveArmRequested,
    OpportunityArrived,
    PaperExecuted,
    RecoveryDetected,
    StrategyBuilt,
    Tick,
)
from atlas_code_quant.autonomy.gates import (
    BrokerGate,
    Gate,
    GateResult,
    HealthGate,
    KillSwitchGate,
    LiveGate,
    RadarGate,
    RiskGate,
    StrategyGate,
    VisionGate,
)
from atlas_code_quant.autonomy.states import (
    ALLOWED_TRANSITIONS,
    LIVE_FORBIDDEN_STATES,
    AutonomyState,
    IllegalTransition,
    assert_transition,
)

logger = logging.getLogger("atlas.code_quant.autonomy.orchestrator")


__all__ = [
    "AutonomyOrchestrator",
    "OrchestratorTickResult",
    "AutonomyConfig",
]


def _log_fsm_transition(
    logger_obj: logging.Logger,
    tick_result: "OrchestratorTickResult",
    event: str,
    *,
    trace_id: str | None = None,
    symbol: str | None = None,
    strategy_id: str | None = None,
) -> None:
    gates_summary: dict[str, dict[str, bool]] = {}
    for idx, gate_result in enumerate(tick_result.gates, start=1):
        gates_summary[f"gate_{idx}"] = {
            "ok": bool(gate_result.ok),
            "degraded": bool(gate_result.degraded),
        }
    payload = {
        "type": "fsm_transition",
        "from_state": tick_result.src.value,
        "to_state": tick_result.dst.value,
        "event": event,
        "reason": tick_result.reason,
        "degraded": bool(tick_result.degraded),
        "gates_summary": gates_summary,
        "trace_id": trace_id,
        "symbol": symbol,
        "strategy_id": strategy_id,
        "timestamp": datetime.now(timezone.utc).isoformat(),
    }
    logger_obj.info("fsm_transition %s", payload)


@dataclass(frozen=True)
class AutonomyConfig:
    """Configuración del orquestador (F17).

    Atributos:
        env: lectura de ``ATLAS_ENV`` por defecto. Sólo el modo
            ``"paper"`` permite avanzar más allá de SCANNING en F17.
        allow_live: SIEMPRE False en F17–F20.
    """

    env: str = "paper"
    allow_live: bool = False


def _env_default_config() -> AutonomyConfig:
    env = os.environ.get("ATLAS_ENV", "paper").strip().lower() or "paper"
    return AutonomyConfig(env=env, allow_live=False)


@dataclass(frozen=True)
class OrchestratorTickResult:
    """Resultado de un ``step()``: estado entrante, saliente y razón."""

    src: AutonomyState
    dst: AutonomyState
    reason: str
    gates: tuple[GateResult, ...] = ()
    degraded: bool = False

    @property
    def transitioned(self) -> bool:
        return self.src != self.dst


class AutonomyOrchestrator:
    """Orquestador FSM autónomo en modo paper (F17)."""

    def __init__(
        self,
        *,
        config: AutonomyConfig | None = None,
        gates: Mapping[str, Gate] | None = None,
        initial_state: AutonomyState = AutonomyState.BOOTING,
    ) -> None:
        self._config: AutonomyConfig = config or _env_default_config()
        self._state: AutonomyState = initial_state
        self._gates: dict[str, Gate] = dict(gates) if gates else self._default_gates()
        self._history: list[AutonomyState] = [initial_state]
        self._current_event_name: str = "unknown"
        self._current_trace_id: str | None = None
        self._current_symbol: str | None = None
        self._current_strategy_id: str | None = None

    # ------------------------------------------------------------------ public

    @property
    def state(self) -> AutonomyState:
        return self._state

    @property
    def history(self) -> tuple[AutonomyState, ...]:
        return tuple(self._history)

    @property
    def config(self) -> AutonomyConfig:
        return self._config

    def gate(self, name: str) -> Gate:
        return self._gates[name]

    def step(
        self,
        event: AutonomyEvent | None = None,
        *,
        ctx: Mapping[str, Any] | None = None,
    ) -> OrchestratorTickResult:
        """Avanza la FSM un paso. Defensivo: nunca lanza."""
        evt = event if event is not None else Tick()
        ctx_map = ctx or {}
        self._current_event_name = evt.__class__.__name__
        self._current_trace_id = self._extract_ctx_str(ctx_map, "trace_id")
        self._current_symbol = self._extract_ctx_str(ctx_map, "symbol")
        self._current_strategy_id = self._extract_ctx_str(ctx_map, "strategy_id")
        try:
            return self._step_inner(evt, ctx_map)
        except Exception as exc:  # noqa: BLE001
            logger.warning(
                "orchestrator: unexpected error %s; transition to ERROR_RECOVERY",
                exc,
            )
            return self._safe_transition(
                AutonomyState.ERROR_RECOVERY,
                reason=f"unexpected_error:{exc}",
                gates=(),
            )
        finally:
            self._current_event_name = "unknown"
            self._current_trace_id = None
            self._current_symbol = None
            self._current_strategy_id = None

    def feed(
        self,
        events: Iterable[AutonomyEvent],
        *,
        ctx: Mapping[str, Any] | None = None,
    ) -> list[OrchestratorTickResult]:
        return [self.step(e, ctx=ctx) for e in events]

    # ----------------------------------------------------------------- helpers

    def _default_gates(self) -> dict[str, Gate]:
        return {
            "health": HealthGate(),
            "radar": RadarGate(),
            "strategy": StrategyGate(),
            "risk": RiskGate(),
            "vision": VisionGate(),
            "broker": BrokerGate(),
            "live": LiveGate(),
            "killswitch": KillSwitchGate(),
        }

    @staticmethod
    def _extract_ctx_str(ctx: Mapping[str, Any], key: str) -> str | None:
        value = ctx.get(key)
        if value is None:
            return None
        as_str = str(value).strip()
        return as_str or None

    # ----------------------------------------------------------------- inner

    def _step_inner(
        self, event: AutonomyEvent, ctx: Mapping[str, Any]
    ) -> OrchestratorTickResult:
        # 1) Kill switch dominante: cualquier estado → KILL_SWITCH.
        ks = self._gates["killswitch"](ctx)
        if not ks.ok or isinstance(event, KillSwitchTriggered):
            return self._safe_transition(
                AutonomyState.KILL_SWITCH, reason=ks.reason, gates=(ks,)
            )

        # 2) Live arm requests son rechazadas en F17–F20. La FSM no
        #    transiciona; se queda donde está.
        if isinstance(event, LiveArmRequested):
            logger.warning(
                "orchestrator: LiveArmRequested rechazada (F17–F20: live blocked)"
            )
            return OrchestratorTickResult(
                src=self._state,
                dst=self._state,
                reason="live_arm_rejected_in_paper_block",
                gates=(self._gates["live"](ctx),),
            )

        # 3) Degradación / recuperación
        if isinstance(event, DegradationDetected):
            return self._safe_transition(
                AutonomyState.DEGRADED,
                reason="degradation_detected",
                gates=(),
            )
        if isinstance(event, RecoveryDetected) and self._state in (
            AutonomyState.DEGRADED,
            AutonomyState.ERROR_RECOVERY,
        ):
            target = (
                AutonomyState.SCANNING
                if self._state == AutonomyState.DEGRADED
                else AutonomyState.BOOTING
            )
            return self._safe_transition(
                target, reason="recovery", gates=()
            )

        # 4) Routing por estado
        s = self._state
        if s == AutonomyState.BOOTING:
            return self._handle_booting(ctx)
        if s == AutonomyState.DEGRADED:
            return self._handle_degraded(ctx)
        if s == AutonomyState.SCANNING:
            return self._handle_scanning(event, ctx)
        if s == AutonomyState.OPPORTUNITY_DETECTED:
            return self._handle_opportunity_detected(event, ctx)
        if s == AutonomyState.STRATEGY_BUILDING:
            return self._handle_strategy_building(event, ctx)
        if s == AutonomyState.BACKTESTING:
            return self._handle_backtesting(event, ctx)
        if s == AutonomyState.PAPER_READY:
            return self._handle_paper_ready(event, ctx)
        if s == AutonomyState.PAPER_EXECUTING:
            return self._handle_paper_executing(event, ctx)
        if s == AutonomyState.MONITORING:
            return self._handle_monitoring(event, ctx)
        if s == AutonomyState.EXITING:
            return self._handle_exiting(ctx)
        if s == AutonomyState.KILL_SWITCH:
            return self._safe_transition(
                AutonomyState.ERROR_RECOVERY,
                reason="killswitch_to_recovery",
                gates=(),
            )
        if s == AutonomyState.ERROR_RECOVERY:
            return self._safe_transition(
                AutonomyState.BOOTING,
                reason="recovery_to_booting",
                gates=(),
            )

        # Estados live: por contrato no se debería llegar aquí, pero
        # si se llegara (config corrupta), forzamos ERROR_RECOVERY.
        return self._safe_transition(
            AutonomyState.ERROR_RECOVERY,
            reason=f"unexpected_state:{s.value}",
            gates=(),
        )

    # ----------------------------------------------------------- per-state

    def _handle_booting(
        self, ctx: Mapping[str, Any]
    ) -> OrchestratorTickResult:
        gates = (
            self._gates["health"](ctx),
            self._gates["radar"](ctx),
            self._gates["strategy"](ctx),
            self._gates["broker"](ctx),
        )
        if any(not g.ok for g in gates):
            return self._safe_transition(
                AutonomyState.DEGRADED,
                reason="boot_gates_degraded",
                gates=gates,
            )
        return self._safe_transition(
            AutonomyState.SCANNING, reason="boot_ok", gates=gates
        )

    def _handle_degraded(
        self, ctx: Mapping[str, Any]
    ) -> OrchestratorTickResult:
        gates = (
            self._gates["health"](ctx),
            self._gates["radar"](ctx),
            self._gates["broker"](ctx),
        )
        if all(g.ok for g in gates):
            return self._safe_transition(
                AutonomyState.SCANNING,
                reason="recovered_from_degraded",
                gates=gates,
            )
        return OrchestratorTickResult(
            src=self._state,
            dst=self._state,
            reason="still_degraded",
            gates=gates,
            degraded=True,
        )

    def _handle_scanning(
        self, event: AutonomyEvent, ctx: Mapping[str, Any]
    ) -> OrchestratorTickResult:
        if isinstance(event, OpportunityArrived):
            return self._safe_transition(
                AutonomyState.OPPORTUNITY_DETECTED,
                reason="opportunity_arrived",
                gates=(),
            )
        radar = self._gates["radar"](ctx)
        if not radar.ok:
            return self._safe_transition(
                AutonomyState.DEGRADED,
                reason=radar.reason,
                gates=(radar,),
            )
        return OrchestratorTickResult(
            src=self._state,
            dst=self._state,
            reason="scanning_idle",
            gates=(radar,),
        )

    def _handle_opportunity_detected(
        self, event: AutonomyEvent, ctx: Mapping[str, Any]
    ) -> OrchestratorTickResult:
        strat = self._gates["strategy"](ctx)
        if not strat.ok:
            return self._safe_transition(
                AutonomyState.DEGRADED,
                reason=strat.reason,
                gates=(strat,),
            )
        return self._safe_transition(
            AutonomyState.STRATEGY_BUILDING,
            reason="strategy_gate_ok",
            gates=(strat,),
        )

    def _handle_strategy_building(
        self, event: AutonomyEvent, ctx: Mapping[str, Any]
    ) -> OrchestratorTickResult:
        if isinstance(event, StrategyBuilt) or ctx.get("strategy_built"):
            return self._safe_transition(
                AutonomyState.BACKTESTING,
                reason="strategy_built",
                gates=(),
            )
        # Esperar; si Radar pierde gate, degradar.
        radar = self._gates["radar"](ctx)
        if not radar.ok:
            return self._safe_transition(
                AutonomyState.SCANNING,
                reason="radar_gate_lost_during_build",
                gates=(radar,),
            )
        return OrchestratorTickResult(
            src=self._state,
            dst=self._state,
            reason="awaiting_strategy_built",
            gates=(),
        )

    def _handle_backtesting(
        self, event: AutonomyEvent, ctx: Mapping[str, Any]
    ) -> OrchestratorTickResult:
        if ctx.get("fitness_ready") is True or event.name == "fitness_ready":
            return self._safe_transition(
                AutonomyState.PAPER_READY,
                reason="fitness_ready",
                gates=(),
            )
        return OrchestratorTickResult(
            src=self._state,
            dst=self._state,
            reason="awaiting_fitness",
            gates=(),
        )

    def _handle_paper_ready(
        self, event: AutonomyEvent, ctx: Mapping[str, Any]
    ) -> OrchestratorTickResult:
        # En F17 los gates risk/vision son stubs OK. Aun así
        # cualquier ``ok=False`` debe bloquear paper_executing.
        risk = self._gates["risk"](ctx)
        vision = self._gates["vision"](ctx)
        broker = self._gates["broker"](ctx)
        gates = (risk, vision, broker)
        if not risk.ok:
            return self._safe_transition(
                AutonomyState.EXITING,
                reason=f"risk_block:{risk.reason}",
                gates=gates,
            )
        if not vision.ok:
            # F18 mapping de VisionTimingGate al orquestador:
            #   * vision_force_exit·→ EXITING
            #   * vision_block      → STRATEGY_BUILDING (revaluar estrategia)
            #   * vision_delay      → permanecer en PAPER_READY
            reason = vision.reason or ""
            if "force_exit" in reason:
                return self._safe_transition(
                    AutonomyState.EXITING,
                    reason=reason,
                    gates=gates,
                )
            if "block" in reason:
                return self._safe_transition(
                    AutonomyState.STRATEGY_BUILDING,
                    reason=reason,
                    gates=gates,
                )
            # delay (incluido vision_delay simulated) → espera
            return OrchestratorTickResult(
                src=self._state,
                dst=self._state,
                reason=f"vision_delay:{reason}",
                gates=gates,
                degraded=any(g.degraded for g in gates),
            )
        if not broker.ok:
            return self._safe_transition(
                AutonomyState.DEGRADED, reason=broker.reason, gates=gates
            )
        return self._safe_transition(
            AutonomyState.PAPER_EXECUTING,
            reason="paper_gates_ok",
            gates=gates,
        )

    def _handle_paper_executing(
        self, event: AutonomyEvent, ctx: Mapping[str, Any]
    ) -> OrchestratorTickResult:
        if isinstance(event, PaperExecuted) or ctx.get("paper_executed"):
            return self._safe_transition(
                AutonomyState.MONITORING,
                reason="paper_executed",
                gates=(),
            )
        return OrchestratorTickResult(
            src=self._state,
            dst=self._state,
            reason="awaiting_paper_executed",
            gates=(),
        )

    def _handle_monitoring(
        self, event: AutonomyEvent, ctx: Mapping[str, Any]
    ) -> OrchestratorTickResult:
        if (
            ctx.get("monitor_done") is True
            or event.name in ("monitoring_timeout", "exit_completed")
        ):
            return self._safe_transition(
                AutonomyState.EXITING,
                reason="monitoring_complete",
                gates=(),
            )
        return OrchestratorTickResult(
            src=self._state,
            dst=self._state,
            reason="monitoring",
            gates=(),
        )

    def _handle_exiting(
        self, ctx: Mapping[str, Any]
    ) -> OrchestratorTickResult:
        return self._safe_transition(
            AutonomyState.SCANNING, reason="exit_to_scanning", gates=()
        )

    # ----------------------------------------------------------- transition

    def _safe_transition(
        self,
        target: AutonomyState,
        *,
        reason: str,
        gates: tuple[GateResult, ...],
    ) -> OrchestratorTickResult:
        # Defensa anti-live (paranoid layer 1 además de assert_transition):
        if target in LIVE_FORBIDDEN_STATES and not self._config.allow_live:
            logger.warning(
                "orchestrator: bloqueada transición a estado live %s", target
            )
            result = OrchestratorTickResult(
                src=self._state,
                dst=self._state,
                reason=f"live_blocked:{target.value}",
                gates=gates,
                degraded=False,
            )
            _log_fsm_transition(
                logger,
                result,
                self._current_event_name,
                trace_id=self._current_trace_id,
                symbol=self._current_symbol,
                strategy_id=self._current_strategy_id,
            )
            return result
        try:
            assert_transition(self._state, target, allow_live=self._config.allow_live)
        except IllegalTransition as exc:
            logger.warning("orchestrator: transición ilegal: %s", exc)
            result = OrchestratorTickResult(
                src=self._state,
                dst=self._state,
                reason=f"illegal_transition:{exc}",
                gates=gates,
            )
            _log_fsm_transition(
                logger,
                result,
                self._current_event_name,
                trace_id=self._current_trace_id,
                symbol=self._current_symbol,
                strategy_id=self._current_strategy_id,
            )
            return result
        prev = self._state
        self._state = target
        self._history.append(target)
        result = OrchestratorTickResult(
            src=prev,
            dst=target,
            reason=reason,
            gates=gates,
            degraded=any(g.degraded for g in gates),
        )
        _log_fsm_transition(
            logger,
            result,
            self._current_event_name,
            trace_id=self._current_trace_id,
            symbol=self._current_symbol,
            strategy_id=self._current_strategy_id,
        )
        return result
