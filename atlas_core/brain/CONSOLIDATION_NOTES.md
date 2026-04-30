# ATLAS Core Consolidation Notes

## Scope
- Compared `atlas_core/autonomy/` (legacy orchestration package) against `atlas_core/brain/` (current canonical Brain Core path).
- Goal: avoid dual-governance and keep migration additive/paper-safe.

## Responsibility Map

| Area | `autonomy` | `brain` | Assessment |
|---|---|---|---|
| Event/command models | `models.py` (`Command`, `ModuleState`, `Snapshot`) | `brain/models.py` (`Event`, `Command`, `SystemSnapshot`) | **Overlap** (different shapes) |
| State aggregation | `autonomy/state_bus.py` | `brain/state_bus.py` | **Overlap** |
| Policy decisions | `autonomy/policy_engine.py` | `brain/safety_kernel.py` + brain event logic | **Overlap** |
| Runtime orchestrator loop | `autonomy/orchestrator.py` | `runtime/event_loop.py` + `brain/brain_core.py` | **Overlap** |
| Module adapters | `autonomy/adapters/*` | `adapters/*` | **Partial overlap** |
| Mission handling | none explicit | `brain/mission_manager.py` | **Brain-only** |
| Mode lifecycle | implicit in config | `runtime/mode_manager.py` | **Brain-only** |
| Operator-facing decisions | not explicit | `brain_core` + `operator_interface_adapter` | **Brain-only** |

## Conflicts Detected
1. Two possible control centers (`AutonomyOrchestrator` vs `BrainCore`) can coexist if both are started.
2. Duplicated Quant adapter logic writes/reads operational state in different ways.
3. Similar policy semantics implemented in separate modules, increasing drift risk.
4. Different snapshot schemas make direct reuse non-trivial.

## Canonical Direction (from now on)
- **Canonical brain path**: `atlas_core/brain/` + `atlas_core/runtime/`.
- **Legacy reusable support**: `atlas_core/autonomy/`.
- `autonomy` should not run as a second sovereign control loop in parallel with `BrainCore`.

## What to Keep / Wrap / Mark

### Keep and coexist
- `autonomy` package files remain available for backward compatibility.
- Existing `start_autonomy_orchestrator.py` remains untouched (no destructive migration).

### Adapt / wrap
- Introduced `brain/autonomy_bridge.py` to read legacy autonomy state in a controlled way.
- Bridge defaults to **read-only compatibility mode** to prevent dual control.

### Mark as legacy support
- `autonomy/orchestrator.py` is considered legacy runtime orchestrator for older flows.
- `autonomy/policy_engine.py` is considered legacy policy path, not canonical for new governance logic.

## Minimum Safe Consolidation Strategy
1. Keep `BrainCore` as single canonical coordinator.
2. Allow legacy telemetry from `autonomy` only via bridge adapters.
3. Disable legacy command dispatch by default through bridge guard.
4. Move new hardening (audit, state integrity, safety rules) only into `brain`.
5. Plan phased migration of useful `autonomy` pieces into `brain` wrappers when stable.

## Risks Still Open
- External scripts could still manually start legacy autonomy orchestrator.
- Snapshot schema differences still require translation layer.
- Shared file-state contention remains possible if non-brain components write same JSON concurrently (mitigated with `shared_state.py` in Brain path).

## Migration Plan by Phases
1. **Phase A (done):** Add bridge and explicit canonical declaration (`brain`).
2. **Phase B:** Route legacy reads through bridge, keep command writes disabled by default.
3. **Phase C:** Gradually deprecate legacy orchestrator entrypoints in docs/scripts.
4. **Phase D:** Optional full convergence once all consumers use brain snapshot/actions.

