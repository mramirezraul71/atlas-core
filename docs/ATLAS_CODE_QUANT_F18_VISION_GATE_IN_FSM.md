# F18 — VisionTimingGate integrado en la FSM (paper-only)

## Política

F18 conecta el gate visual canónico del estudio maestro v9 al
orquestador F17 sin sacar el sistema del modo paper:

> Si cámara unavailable y `requires_visual_confirmation=False`
> → decision=ALLOW, degraded=True.
>
> Si cámara unavailable y `requires_visual_confirmation=True`
> → decision=BLOCK, degraded=True.

`VisionTimingGate` es **independiente** del orquestador y no acopla
tipos de Radar/Strategy. Recibe un `GateInput` mínimo y devuelve un
`GateOutput` con veredicto canónico (`allow | delay | block | force_exit`).

## Alcance (in / out)

In:

- `atlas_code_quant/vision/timing_gate/gate.py`:
  - `VisionDecision` (enum: ALLOW/DELAY/BLOCK/FORCE_EXIT).
  - `GateInput` (opportunity, strategy_id, requires_visual_confirmation,
    market_open, extras).
  - `GateOutput` (decision, reason, degraded, camera_status, evidence).
  - `VisionTimingGate(capture_probe=..., pattern_evaluator=...)`.
- Reexports en `atlas_code_quant/vision/timing_gate/__init__.py`.
- `VisionGate` actualizado en `autonomy/gates/orchestrator_gates.py`:
  delega en `VisionTimingGate` y traduce `decision → GateResult`.
- Mapping en `AutonomyOrchestrator._handle_paper_ready`:
  - ALLOW → PAPER_EXECUTING.
  - DELAY → permanece en PAPER_READY (degraded propagado).
  - BLOCK → STRATEGY_BUILDING (re-evaluar estrategia).
  - FORCE_EXIT → EXITING.
- Overrides explícitos para tests del orquestador
  (`vision_block` / `vision_delay` / `vision_force_exit` en ctx).

Out (NO entra en F18):

- Endpoint HTTP `/vision_confirm` público (sólo se diseña, no se expone).
- Modelos visuales reales (OCR, charts, patterns) — el evaluador es
  inyectable; el default sólo aplica la política de cámara.
- Cualquier autorización live; F18 sigue siendo paper-only.

## Aislamiento

`vision/timing_gate/gate.py` NO importa execution canónico,
`operations.live_*`, `production.live_activation`, ni `atlas_adapter`
(verificado por AST guard). No referencia `paper_only` /
`full_live_globally_locked` como código.

`autonomy/gates/orchestrator_gates.py::VisionGate` importa
`vision.timing_gate` de forma perezosa para mantener el módulo
defensivo si el paquete `vision` está roto en runtime.

## Diagrama integrado (paper)

```
PAPER_READY
   │
   ├── risk.ok=False    → EXITING
   ├── vision.decision=ALLOW       → PAPER_EXECUTING
   ├── vision.decision=DELAY       → PAPER_READY (espera, degraded prop)
   ├── vision.decision=BLOCK       → STRATEGY_BUILDING (revaluar)
   ├── vision.decision=FORCE_EXIT  → EXITING
   ├── broker.ok=False → DEGRADED
   └── (todos OK)      → PAPER_EXECUTING
```

LIVE_ARMED / LIVE_EXECUTING continúan **inalcanzables** (`assert_transition`
con `allow_live=False` los rechaza). VisionGate ALLOW no autoriza live
bajo ninguna circunstancia en este bloque.

## API pública

```python
from atlas_code_quant.vision.timing_gate import (
    VisionTimingGate, GateInput, GateOutput, VisionDecision,
)

gate = VisionTimingGate(
    capture_probe=lambda: {"ok": False, "source": "none", "error": "no_cam"},
    pattern_evaluator=None,  # default: política de cámara
)
out = gate.evaluate(GateInput(
    strategy_id="vs1", requires_visual_confirmation=False,
))
# out.decision == VisionDecision.ALLOW, out.degraded == True
```

Y en el orquestador:

```python
from atlas_code_quant.autonomy.orchestrator import AutonomyOrchestrator
from atlas_code_quant.autonomy.gates import VisionGate

# inyectar un timing_gate específico (tests)
orch = AutonomyOrchestrator(gates={
    ...,
    "vision": VisionGate(timing_gate=VisionTimingGate(...)),
    ...,
})
```

## Tests

`tests/atlas_code_quant/test_vision_timing_gate_f18.py` — 12 tests:

- `TestPolicyDefault` (4): camera ok/unavailable optional/required;
  probe que lanza.
- `TestInjectedEvaluator` (4): DELAY, FORCE_EXIT, evaluador que lanza,
  retorno inválido.
- `TestDefensiveInputs` (2): GateInput None, default sin probe.

`tests/atlas_code_quant/test_autonomy_vision_gate_f18.py` — 8 tests:

- `TestVisionInFSM` (4): ALLOW/DELAY/BLOCK/FORCE_EXIT en PAPER_READY.
- `TestCameraUnavailable` (2): optional ALLOW degraded;
  required BLOCK.
- `TestLiveStillBlocked` (1): live no alcanzable aun con ALLOW.
- AST guards (2): no imports prohibidos; sin locks como código.

Total F18: **20 tests passed**. Regresión F17: 18 passed.

## Invariantes

- `VisionTimingGate.evaluate(None)` → BLOCK degraded.
- Si `capture_probe` o `pattern_evaluator` lanzan, `evaluate` los
  envuelve y devuelve BLOCK degraded.
- VisionGate del orquestador: BLOCK siempre devuelve `GateResult.ok=False`,
  pero el orquestador interpreta el `reason` (`vision_block` vs
  `vision_force_exit` vs `vision_delay`) para mapear a STRATEGY_BUILDING /
  EXITING / espera.
- El orquestador NUNCA transiciona a `LIVE_ARMED`/`LIVE_EXECUTING` aunque
  VisionGate diga ALLOW.
- Endpoint HTTP `/vision_confirm` NO se expone en F18.

## Rollback

```bash
cd /home/user/workspace/atlas-core
git revert <commit_F18>
# o, manualmente
git rm atlas_code_quant/vision/timing_gate/gate.py \
       tests/atlas_code_quant/test_vision_timing_gate_f18.py \
       tests/atlas_code_quant/test_autonomy_vision_gate_f18.py \
       docs/ATLAS_CODE_QUANT_F18_VISION_GATE_IN_FSM.md
git checkout atlas_code_quant/vision/timing_gate/__init__.py \
              atlas_code_quant/autonomy/gates/orchestrator_gates.py \
              atlas_code_quant/autonomy/orchestrator.py
```

F11–F17 quedan intactos: el revert sólo borra/restaura módulos vision
y la integración del VisionGate en el handler de PAPER_READY.
