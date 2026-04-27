# Atlas Code Quant — F19: Safety y Kill Switch centralizados (paper-only)

Fase F19 del bloque F17–F20 de autonomía. Introduce los tres mecanismos
de seguridad que el orquestador F17/F18 venía consumiendo como stubs:

1. **Risk limits** explícitos sobre PnL diario, notional por posición y
   tasa de órdenes.
2. **Circuit breaker** global con estados `closed` / `open` / `half_open`
   y cooldown temporal, alimentado por las violaciones del `RiskGate`.
3. **Kill switch por fichero** activable manualmente vía
   `ATLAS_KILLSWITCH_FILE`, leído por el `KillSwitchGate`.

Toda la fase es **paper-only**: ninguna pieza autoriza, arma ni ejecuta
órdenes reales. Los gates `LiveGate` siguen devolviendo `ok=False` y los
estados `LIVE_ARMED` / `LIVE_EXECUTING` continúan inalcanzables (F17).

---

## Política consolidada (qué cambia en F19)

- F19 **no modifica** la FSM ni `assert_transition`: las transiciones
  hacia live siguen rechazadas en `allow_live=False`.
- F19 **no toca** `atlas_adapter`, ni el scanner, ni Radar.
- F19 **no toca** `execution/tradier_adapter` ni el pipeline F16.
- F19 sólo proporciona infraestructura de safety que el `RiskGate` y el
  `KillSwitchGate` ya conocían como stubs.
- El override `ctx["risk_violation"]` (F17) y `ctx["killswitch"]` (F17)
  se conservan: tienen **prioridad** sobre la lectura de límites/fichero
  para no romper los tests del orquestador F17/F18.

## Alcance

Módulos nuevos y modificados (todo bajo `atlas_code_quant/risk/` y
`atlas_code_quant/autonomy/gates/`):

| Path | Tipo | Función |
| --- | --- | --- |
| `atlas_code_quant/risk/limits/checks.py` | nuevo | `RiskLimitsConfig`, `check_daily_loss_limit`, `check_position_notional_limit`, `check_orders_per_minute`, `check_all_limits`, `load_risk_limits_from_env`. |
| `atlas_code_quant/risk/limits/__init__.py` | actualizado | scaffold → reexports reales. |
| `atlas_code_quant/risk/circuit_breaker.py` | nuevo | `BreakerState`, `CircuitBreakerConfig`, `CircuitBreaker` con reloj inyectable. |
| `atlas_code_quant/risk/kill_switch/file_switch.py` | nuevo | `KillSwitchStatus`, `FileKillSwitch`, `load_kill_switch_path_from_env`. |
| `atlas_code_quant/risk/kill_switch/__init__.py` | actualizado | scaffold → reexports reales. |
| `atlas_code_quant/autonomy/gates/orchestrator_gates.py` | actualizado | `RiskGate` y `KillSwitchGate` ahora consumen los módulos reales (sin perder overrides ctx). |

Tests:

- `tests/atlas_code_quant/test_risk_limits_and_killswitch_f19.py` — 31
  tests verdes cubriendo límites, breaker, file switch, integración con
  los gates del orquestador y guards AST.

## API

### `RiskLimitsConfig`

```python
from atlas_code_quant.risk.limits import RiskLimitsConfig, load_risk_limits_from_env

cfg = RiskLimitsConfig(
    max_daily_loss_usd=500.0,        # absoluto, positivo
    max_position_notional_usd=2_500.0,
    max_orders_per_minute=30,
)
# o desde env: ATLAS_MAX_DAILY_LOSS_USD, ATLAS_MAX_POSITION_NOTIONAL_USD,
# ATLAS_MAX_ORDERS_PER_MINUTE
cfg = load_risk_limits_from_env()
```

Defaults conservadores. F20 cargará estos valores desde
`config/live_readiness.py`.

### Checks

Todos los checks devuelven el mismo `GateResult` canónico que usan los
gates del orquestador:

```python
from atlas_code_quant.risk.limits import check_all_limits

result = check_all_limits(
    realized_pnl_usd=-120.0,   # negativo = pérdida
    notional_usd=1_500.0,
    orders_in_last_minute=12,
    config=cfg,
)
assert result.ok
```

Reglas:

- `check_daily_loss_limit`: falla si `realized_pnl_usd < -max_daily_loss_usd`.
- `check_position_notional_limit`: falla si
  `abs(notional_usd) > max_position_notional_usd`.
- `check_orders_per_minute`: falla si
  `orders_in_last_minute >= max_orders_per_minute`.
- `check_all_limits`: secuencial; falla rápido al primer
  `ok=False`. Inputs corruptos → 0/0/0 (defensivo).

### `CircuitBreaker`

```python
from atlas_code_quant.risk.circuit_breaker import (
    CircuitBreaker, CircuitBreakerConfig, BreakerState,
)

cb = CircuitBreaker(config=CircuitBreakerConfig(failure_threshold=3, cooldown_seconds=60))
cb.record_failure()  # 1
cb.record_failure()  # 2
cb.record_failure()  # 3 → OPEN
cb.allow_request()   # False mientras dure el cooldown
# tras cooldown: state lazy → HALF_OPEN; allow_request() == True
cb.record_success()  # → CLOSED
```

Detalles:

- `time_fn` inyectable para tests deterministas.
- En `HALF_OPEN`, **cualquier** fallo re-abre el breaker.
- `trip(reason)` fuerza apertura manual; `reset()` vuelve a `CLOSED`.

### `FileKillSwitch`

```python
from atlas_code_quant.risk.kill_switch import (
    FileKillSwitch, load_kill_switch_path_from_env,
)

# Lee ATLAS_KILLSWITCH_FILE (default /tmp/atlas_killswitch).
ks = FileKillSwitch()
ks.is_activated()  # True sólo si fichero existe y contenido != ""
status = ks.status()
# KillSwitchStatus(activated, path, reason, raw_marker, error)
```

Reglas:

- Fichero ausente → `activated=False, reason="killswitch_file_absent"`.
- Fichero presente y vacío → `activated=False, reason="killswitch_file_empty"`.
- Fichero presente con contenido → `activated=True`,
  `raw_marker=content[:64]`.
- Errores de E/S → `activated=False` con `error` poblado (defensivo).

## Integración con el orquestador

### `RiskGate` (autonomy/gates/orchestrator_gates.py)

```python
RiskGate(breaker: CircuitBreaker | None = None,
         config: RiskLimitsConfig | None = None)
```

Comportamiento:

1. Si `ctx["risk_violation"] is True` (override F17) → registra fallo
   en breaker (si existe) y devuelve `ok=False`.
2. Si breaker en `OPEN` → `ok=False, reason="risk_circuit_breaker_open"`.
3. Llama `check_all_limits(...)` con valores leídos del ctx
   (`realized_pnl_usd`, `notional_usd`, `orders_in_last_minute`, `risk_config`).
4. Resultado `ok=True` ⇒ `breaker.record_success()`. Resultado
   `ok=False` ⇒ `breaker.record_failure()`.

### `KillSwitchGate`

```python
KillSwitchGate(switch: FileKillSwitch | None = None)
```

Comportamiento:

1. Si `"killswitch" in ctx` → override explícito gana.
2. Si no, llama `switch.status()`. Activado ⇒ `ok=False`. No activado ⇒
   `ok=True` con la razón del status.
3. Errores se transforman en `ok=True, degraded=True` (no bloquear
   paper por errores de E/S locales).

### Mapping a la FSM (F17 inalterado)

| Gate | Fallo en estado | Acción FSM |
| --- | --- | --- |
| `RiskGate` (PAPER_READY) | `ok=False` | → `EXITING` (F17). |
| `KillSwitchGate` (cualquier estado) | `ok=False` | → `KILL_SWITCH` → `ERROR_RECOVERY` → `BOOTING` (F17). |

Ningún cambio en `assert_transition`. `LIVE_FORBIDDEN_STATES` sigue
igual.

## Invariantes

- **Live nunca alcanzable**: `LiveGate` sigue devolviendo `ok=False`,
  ningún módulo F19 lo modifica.
- **No imports prohibidos**: `risk/limits/checks.py`,
  `risk/circuit_breaker.py`, `risk/kill_switch/file_switch.py` no
  importan `tradier_execution`, `broker_router`, `tradier_controls`,
  `tradier_pdt_ledger`, `auton_executor`, `live_authorization`,
  `live_loop`, `live_switch`, `operation_center`, `signal_executor`,
  `start_paper_trading`, `production.live_activation`, `atlas_adapter`.
- **Defensivo total**: ningún método de los nuevos módulos lanza ante
  inputs corruptos. Errores se traducen a `ok=False` o `activated=False`
  con `reason`/`error` explícitos.
- **Overrides ctx prioritarios**: `ctx["risk_violation"]` y
  `ctx["killswitch"]` siguen funcionando para los tests F17/F18 sin
  requerir disco ni breaker.

## Tests

`tests/atlas_code_quant/test_risk_limits_and_killswitch_f19.py` (31
tests, 0.45 s):

- **RiskLimits** (10): cada límite individual, combinación,
  carga de env, inputs corruptos.
- **CircuitBreaker** (6): apertura tras N fallos, cooldown,
  HALF_OPEN reabre con fallo, success cierra, trip/reset, allow_request.
- **FileKillSwitch** (6): ausente, vacío, con contenido, env override,
  rutas inválidas, raw_marker truncado.
- **RiskGate** (3): override ctx, breaker abierto bloquea, breaker
  registra success/failure según resultado.
- **KillSwitchGate en FSM** (4): override ctx (compatibilidad F17),
  switch real activado, switch limpio, error de E/S no bloquea.
- **Guards AST** (2): los 3 módulos no contienen imports prohibidos ni
  referencias a símbolos `paper_only`/`full_live_globally_locked`.

## Validación de regresión

```bash
python -m pytest \
  tests/atlas_code_quant/test_autonomy_fsm_f17.py \
  tests/atlas_code_quant/test_vision_timing_gate_f18.py \
  tests/atlas_code_quant/test_autonomy_vision_gate_f18.py \
  tests/atlas_code_quant/test_risk_limits_and_killswitch_f19.py -q
# 68 passed
```

Baseline collect:

```bash
python -m pytest atlas_code_quant/tests --collect-only -q
# 1054 tests collected, 40 errors  (preexistentes: sqlalchemy/backtesting)
```

## Rollback

F19 es un commit aislado. Para revertir:

```bash
git revert <hash_f19>
```

Tras revertir:

- Los módulos `risk/limits/checks.py`, `risk/circuit_breaker.py` y
  `risk/kill_switch/file_switch.py` desaparecen.
- `RiskGate` y `KillSwitchGate` vuelven a ser stubs F17 (siempre `ok=True`
  salvo override ctx).
- `tests/atlas_code_quant/test_risk_limits_and_killswitch_f19.py`
  desaparece.
- F17 y F18 siguen verdes; ninguna otra fase depende de F19.
