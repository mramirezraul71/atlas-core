# Atlas Code Quant — F20: Live Readiness Checklist (sin activar live)

Fase final del bloque F17–F20. Introduce la **checklist de preparación
para live trading** y los helpers de configuración asociados. **No
activa live**: la FSM F17 sigue rechazando cualquier transición a
`LIVE_ARMED` / `LIVE_EXECUTING` (`assert_transition(allow_live=False)`).

F20 es informativo: permite a un humano (o a un futuro F21+) saber si
las condiciones mínimas están dadas para *armar* live. Mientras tanto,
Atlas Code Quant sigue siendo paper-only end-to-end.

---

## Política consolidada (recordatorio del bloque)

- F17: FSM 14 estados, `LIVE_FORBIDDEN_STATES` declarados, transición
  hacia live rechazada con `allow_live=False`.
- F18: `VisionTimingGate` integrado en la FSM como gate adicional.
- F19: risk limits, circuit breaker y file kill switch reales,
  consumidos por `RiskGate` y `KillSwitchGate`.
- **F20**: checklist y helpers de env. Defaults siguen siendo paper.
  Aunque `overall_ok=True`, F17 NO permite la transición a live.

## Alcance

Archivos nuevos:

| Path | Función |
| --- | --- |
| `atlas_code_quant/config/live_readiness.py` | Helpers env: `LiveReadinessEnv`, `load_live_readiness_env`, `is_live_mode_requested`, `is_live_mode_safe_to_arm`. |
| `atlas_code_quant/autonomy/live_checklist.py` | `LiveChecklistItem`, `LiveChecklistResult`, `build_live_checklist`. |
| `tests/atlas_code_quant/test_live_readiness_f20.py` | 20 tests verdes. |
| `docs/ATLAS_CODE_QUANT_F20_LIVE_READINESS_CHECKLIST.md` | Esta doc. |

Archivos modificados: ninguno. F20 es 100% aditivo.

## Variables de entorno

Defaults conservadores. Ningún cambio en el comportamiento por
defecto del proceso.

| Variable | Tipo | Default | Uso |
| --- | --- | --- | --- |
| `ATLAS_ENV` | str | `paper` | `paper` / `live` (intención declarativa). |
| `ATLAS_LIVETRADINGENABLED` | bool | `false` | Habilita intención live. |
| `ATLAS_TRADIERDRYRUN` | bool | `true` | Dry-run del adaptador Tradier. |
| `ATLAS_VISION_REQUIRED_FOR_LIVE` | bool | `true` | Vision obligatoria para live. |
| `ATLAS_KILLSWITCH_FILE` | path | `/tmp/atlas_killswitch` | Reutiliza F19. |
| `ATLAS_MAX_DAILY_LOSS_USD` | float | `500` | Reutiliza F19. |
| `ATLAS_MAX_POSITION_NOTIONAL_USD` | float | `2500` | Reutiliza F19. |
| `ATLAS_MAX_ORDERS_PER_MINUTE` | int | `30` | Reutiliza F19. |

Booleanos aceptan `1/0`, `true/false`, `yes/no`, `on/off`, `y/n`,
`t/f` (case-insensitive). Inputs corruptos → default seguro.

## API

### `LiveReadinessEnv` (frozen dataclass)

```python
from atlas_code_quant.config.live_readiness import (
    LiveReadinessEnv, load_live_readiness_env,
    is_live_mode_requested, is_live_mode_safe_to_arm,
)

env = load_live_readiness_env()
# LiveReadinessEnv(atlas_env='paper', live_trading_enabled=False, ...)

is_live_mode_requested(env)    # True sólo si atlas_env=live AND flag=true
is_live_mode_safe_to_arm(env)  # además requiere tradier_dry_run=False
```

`is_live_mode_safe_to_arm` es **declarativo**: sólo confirma que la
config es consistente con un intento de armar live. **No** autoriza la
transición de la FSM. F17 sigue siendo la fuente de verdad.

### `build_live_checklist`

```python
from atlas_code_quant.autonomy.live_checklist import build_live_checklist

result = build_live_checklist(camera_available=True)
print(result.overall_ok)
print(result.to_dict())
```

Argumentos:

* `env: LiveReadinessEnv | None` — defaults a `load_live_readiness_env()`.
* `camera_available: bool | None` — None = desconocido (defensivo).
* `switch: FileKillSwitch | None` — defaults a `FileKillSwitch(path=env.killswitch_file)`.

Resultado:

```python
@dataclass(frozen=True)
class LiveChecklistItem:
    name: str
    ok: bool
    reason: str
    evidence: Mapping[str, Any]

@dataclass(frozen=True)
class LiveChecklistResult:
    overall_ok: bool
    items: Sequence[LiveChecklistItem]
    env: LiveReadinessEnv

    def to_dict(self) -> dict: ...
```

## Items evaluados

| # | Nombre | Condición OK |
| --- | --- | --- |
| 1 | `env_live_requested` | `atlas_env=live` y `ATLAS_LIVETRADINGENABLED=true`. |
| 2 | `tradier_not_dry_run` | `ATLAS_TRADIERDRYRUN=false`. |
| 3 | `risk_limits_configured` | Todos los límites > 0. |
| 4 | `killswitch_path_configured` | `ATLAS_KILLSWITCH_FILE` no vacío. |
| 5 | `killswitch_clear` | Fichero ausente o vacío. |
| 6 | `vision_available_or_optional` | Cámara disponible **o** vision no requerida. |
| 7 | `broker_paper_constructible` | `TradierAdapter(dry_run=True)` construye. |
| 8 | `fsm_paper_only_invariant` | `LIVE_FORBIDDEN_STATES` contiene `LIVE_ARMED` y `LIVE_EXECUTING`. |

`overall_ok = all(items.ok)` **y además** `is_live_mode_safe_to_arm(env)`
debe ser True (sanity defensivo: si no, fuerza `overall_ok=False`).

## Mapping a la FSM

F20 NO modifica la FSM. La intención es:

* Antes de cualquier futuro intento de armar live, llamar
  `build_live_checklist(...)`.
* Si `overall_ok=False`: ningún paso adicional.
* Si `overall_ok=True`: una fase **futura** (F21+) podrá decidir, con
  arming explícito y autorización humana, llamar
  `assert_transition(..., allow_live=True)`. **F20 NO hace esto**.

## Invariantes (verificadas por tests)

- **Live nunca alcanzable en F17–F20**: aunque la checklist sea verde,
  `assert_transition(PAPER_READY, LIVE_ARMED, allow_live=False)` lanza
  `IllegalTransition` (test
  `test_full_green_checklist_does_not_unlock_fsm`).
- **`LIVE_ARMED` y `LIVE_EXECUTING` siguen en `LIVE_FORBIDDEN_STATES`**.
- **No imports prohibidos**: `live_readiness.py` y `live_checklist.py`
  no importan `tradier_execution`, `broker_router`, `tradier_controls`,
  `tradier_pdt_ledger`, `auton_executor`, `live_authorization`,
  `live_loop`, `live_switch`, `operation_center`, `signal_executor`,
  `start_paper_trading`, `production.live_activation`, `atlas_adapter`.
- **Cero `allow_live=True` hardcoded**: ni en código F20 ni en cualquier
  ruta que la checklist invoque.
- **Defaults paper**: `ATLAS_ENV` ausente ⇒ `paper`;
  `ATLAS_LIVETRADINGENABLED` ausente ⇒ `false`; `ATLAS_TRADIERDRYRUN`
  ausente ⇒ `true`. Este es el invariante explícito del bloque
  F17–F20.

## Tests

`tests/atlas_code_quant/test_live_readiness_f20.py` (20 tests, 0.22 s):

- `load_live_readiness_env`: defaults, parsing truthy, inputs corruptos.
- `is_live_mode_requested` / `is_live_mode_safe_to_arm`: combinaciones
  de flags.
- `build_live_checklist`: estado paper por defecto, killswitch presente
  bloquea, killswitch ausente pasa, full-green requiere env explícito,
  vision required + camera unknown → fail, vision opcional pasa,
  límites inválidos fallan, `to_dict` serializable.
- Invariantes FSM: `LIVE_ARMED`/`LIVE_EXECUTING` siguen prohibidos;
  full-green NO desbloquea la FSM.
- Guards AST: imports prohibidos ausentes; `allow_live=True` ausente.

## Validación de regresión

Tests dirigidos del bloque F17–F20:

```bash
python -m pytest \
  tests/atlas_code_quant/test_autonomy_fsm_f17.py \
  tests/atlas_code_quant/test_vision_timing_gate_f18.py \
  tests/atlas_code_quant/test_autonomy_vision_gate_f18.py \
  tests/atlas_code_quant/test_risk_limits_and_killswitch_f19.py \
  tests/atlas_code_quant/test_live_readiness_f20.py -q
# 88 passed
```

Baseline collect:

```bash
python -m pytest atlas_code_quant/tests --collect-only -q
# 1054 tests collected, 40 errors  (preexistentes: sqlalchemy/backtesting)
```

## Rollback

F20 es un commit aislado y aditivo:

```bash
git revert <hash_f20>
```

Tras revertir:

- Desaparecen `config/live_readiness.py` y `autonomy/live_checklist.py`.
- Desaparecen los 20 tests F20.
- F17–F19 quedan inalterados y verdes. Ningún módulo previo importa
  F20.
