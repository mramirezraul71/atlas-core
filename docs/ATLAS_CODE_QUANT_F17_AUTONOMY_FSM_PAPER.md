# F17 — Autonomy Orchestrator (paper-only FSM)

## Política

F17 introduce el **esqueleto de orquestador autónomo** descrito en la
sección 11 del estudio maestro v9: una máquina de 14 estados con
gates explícitos. La FSM **opera exclusivamente sobre el pipeline
paper construido en F11–F16** (Radar → Strategy → fitness → Tradier
dry-run). Los estados ``LIVE_ARMED`` y ``LIVE_EXECUTING`` están
declarados pero NO son alcanzables en este bloque (F17–F20):

> Scanner propone. Radar decide. Estrategias y ejecución sólo operan
> sobre oportunidades aprobadas por Radar, en papel hasta que se
> defina un bloque posterior de autonomía y live readiness.

## Alcance (in / out)

In:

- 14 estados canónicos (`AutonomyState`).
- Mapa cerrado de transiciones legales (`ALLOWED_TRANSITIONS`).
- Eventos básicos (`Tick`, `OpportunityArrived`, `StrategyBuilt`,
  `FitnessReady`, `PaperExecuted`, `DegradationDetected`,
  `RecoveryDetected`, `KillSwitchTriggered`, `LiveArmRequested`,
  `MonitoringTimeout`, `ExitCompleted`).
- 8 gates con interfaz uniforme (`Gate`, `GateResult`):
  HealthGate, RadarGate, StrategyGate, RiskGate, VisionGate,
  BrokerGate, LiveGate, KillSwitchGate. Risk/Vision/KillSwitch
  son stubs F17 y se conectan en F18/F19.
- `AutonomyOrchestrator.step(event, ctx)` defensivo, nunca lanza.
- `OrchestratorTickResult` con src/dst/reason/gates/degraded.

Out (NO entra en F17):

- Live trading, broker live, live_authorization, live_loop,
  operation_center, signal_executor.
- VisionTimingGate real (F18).
- Risk limits y circuit breaker reales (F19).
- KillSwitch desde fichero (F19).
- Live readiness checklist (F20).

## Aislamiento

`atlas_code_quant/autonomy/orchestrator.py` sólo importa:

- `atlas_code_quant.autonomy.events`
- `atlas_code_quant.autonomy.gates`
- `atlas_code_quant.autonomy.states`

No importa execution canónico (`tradier_execution`, `broker_router`,
`tradier_controls`, `tradier_pdt_ledger`), ni `operations.auton_executor`,
`operations.live_authorization`, `operations.live_loop`,
`operations.live_switch`, `operations.operation_center`,
`operations.signal_executor`, `operations.start_paper_trading`,
`production.live_activation`, ni `atlas_adapter`. Verificado por
AST guard.

Tampoco referencia `paper_only` ni `full_live_globally_locked` como
`Name`/`Attribute` (los locks globales no se tocan).

## Diagrama de transiciones (paper-only)

```
                         ┌───────────────────────────────────────┐
                         ▼                                       │
   BOOTING ─► SCANNING ─► OPPORTUNITY_DETECTED ─► STRATEGY_BUILDING
       │         │              │                       │
       │         │              ▼                       ▼
       ▼         ▼          DEGRADED              BACKTESTING
   DEGRADED  DEGRADED            │                       │
       │         │               ▼                       ▼
       │         │          (recovery)              PAPER_READY
       │         │               │                  │ │   │
       │         │               ▼                  │ │   ▼
       │         └──────────► SCANNING ◄────────────┘ │   STRATEGY_BUILDING
       │                                              │   (vision_block)
       │                                              ▼
       │                                       PAPER_EXECUTING
       │                                              │
       │                                              ▼
       │                                          MONITORING
       │                                              │
       │                                              ▼
       │                                           EXITING
       │                                              │
       │                                              ▼
       │                                          SCANNING
       │
       └─► (cualquier estado)
                  │
                  ▼ KillSwitchTriggered / killswitch ctx
              KILL_SWITCH ─► ERROR_RECOVERY ─► BOOTING
```

LIVE_ARMED / LIVE_EXECUTING existen como enum pero NO son alcanzables
en F17–F20 (`assert_transition` los rechaza con `IllegalTransition`
cuando `allow_live=False`, que es el default).

## Gates

| Gate | F17 stub | Conexión real |
|---|---|---|
| HealthGate | ctx.health=False → ok=False | F19 (snapshot/canonical) |
| RadarGate | ctx.radar_degraded=True → ok=False | ya conectado a contexto Radar |
| StrategyGate | importa F12+F14, falla si no se pueden importar | ya real |
| RiskGate | ctx.risk_violation=True → ok=False | F19 (limits + circuit breaker) |
| VisionGate | ctx.vision_block / vision_force_exit | F18 (VisionTimingGate) |
| BrokerGate | construye TradierAdapter dry-run | ya real |
| LiveGate | siempre ok=False | permanece igual hasta el bloque live |
| KillSwitchGate | ctx.killswitch=True → ok=False | F19 (fichero + límites) |

## API pública

```python
from atlas_code_quant.autonomy.orchestrator import (
    AutonomyOrchestrator, AutonomyConfig, OrchestratorTickResult,
)
from atlas_code_quant.autonomy.states import AutonomyState
from atlas_code_quant.autonomy.events import Tick, OpportunityArrived

orch = AutonomyOrchestrator()  # default paper, allow_live=False
result = orch.step(Tick())          # BOOTING → SCANNING
orch.step(OpportunityArrived())     # → OPPORTUNITY_DETECTED
```

`step(event, ctx)` no lanza. Cualquier excepción interna se traduce
a transición a `ERROR_RECOVERY` con `reason="unexpected_error:..."`.

## Tests

`tests/atlas_code_quant/test_autonomy_fsm_f17.py` — **18 tests**:

1. `TestStatesAndTransitions` (4): 14 estados; live forbidden;
   transiciones paper legales; rechazo de saltos ilegales.
2. `TestHappyPath` (1): ciclo completo BOOTING→…→SCANNING.
3. `TestDegradedPath` (3): boot degradado, recuperación por contexto,
   recuperación por evento.
4. `TestKillSwitch` (3): kill switch desde múltiples estados; vía ctx;
   KILL_SWITCH→ERROR_RECOVERY→BOOTING.
5. `TestLiveBlocked` (3): live no alcanzable; LiveArmRequested
   rechazada; LiveGate siempre ok=False.
6. `TestDefensive` (2): step nunca propaga ante gate que explota;
   tras 50 ticks la FSM sigue en estado válido.
7. AST guards (2): no imports prohibidos; sin locks como Name/Attribute.

Resultado: **18 passed**.

## Invariantes

- `step()` nunca lanza.
- Todas las transiciones están en `ALLOWED_TRANSITIONS` (verificado
  por test).
- `LIVE_ARMED` / `LIVE_EXECUTING` no se visitan jamás con
  `allow_live=False`.
- La FSM no escribe a disco ni abre HTTP por sí misma.
- KillSwitch dominante: cualquier evento `KillSwitchTriggered` o
  `ctx.killswitch=True` fuerza `KILL_SWITCH` antes de cualquier otra
  transición.

## Rollback

```bash
cd /home/user/workspace/atlas-core
git revert <commit_F17>
# o, manualmente
git rm atlas_code_quant/autonomy/states.py \
       atlas_code_quant/autonomy/events.py \
       atlas_code_quant/autonomy/orchestrator.py \
       atlas_code_quant/autonomy/gates/orchestrator_gates.py \
       tests/atlas_code_quant/test_autonomy_fsm_f17.py \
       docs/ATLAS_CODE_QUANT_F17_AUTONOMY_FSM_PAPER.md
git checkout atlas_code_quant/autonomy/gates/__init__.py
```

F17 no modifica F11–F16 ni el scaffold previo (`autonomy/gates/contracts.py`,
`autonomy/fsm/`, `autonomy/policy/`).
