# PLAN_STEP_D — Scaffold de `DecisionEngine` dentro de `atlas_push/`

> Versión aprobada por el owner el 2026-04-21. Documenta el Paso D de
> la reestructuración de Atlas Push. Referencias cruzadas:
> `ARCHITECTURE.md`, `CURRENT_STATE.md`, `DEFECTS.md`,
> `PLAN_STEP_B.md`, `PLAN_STEP_C.md`.

## 1. Objetivo

Introducir dentro de `atlas_push/` la cara hermana del `IntentRouter`:
un `DecisionEngine` con su contrato de entrada (`MarketState`) y su
contrato de salida (`DecisionOutput`), siguiendo literalmente
`ARCHITECTURE.md` §3.

El motor se introduce como **scaffold pass-through**:
`DecisionEngine(...).decide(state)` devuelve `DecisionOutput.empty()`
para cualquier entrada. No hay lógica de trading, no hay estrategias
reales, no hay cableado con el `IntentRouter`. D fija únicamente
tipos, layout de paquete y puntos de extensión (Protocols).

Fuera de ese perímetro, D no toca nada del código vivo del repo.

## 2. Principios (vinculantes)

1. **Hermanos, no anidados.** `IntentRouter` y `DecisionEngine` son
   dos componentes al mismo nivel dentro de `atlas_push/`.
   `IntentRouter` **no** importa `DecisionEngine`, y viceversa. Esta
   separación está blindada con tests source-level en D2.
2. **Sin cableado en D.** El flujo HTTP `/intent` y los 12 `Kind`
   aprobados en B no se tocan. `DecisionEngine` no recibe texto de
   usuario; solo consumirá `MarketState` cuando pasos posteriores lo
   conecten con intents de cerebro.
3. **Contratos completos desde ya.** `MarketState` y `DecisionOutput`
   se declaran con todos los campos de `ARCHITECTURE.md` §3.1/§3.2
   aunque D no los consuma todavía. Esto evita roturas de firma en
   pasos posteriores y permite construir estados realistas en tests.
4. **Inmutabilidad estructural.** Todas las value types son
   `@dataclass(frozen=True)`. Las colecciones por orden
   (`positions`, `orders`, `target_weights`, `vetoes`, `notes`,
   `reasons`) son `tuple`, no `list`.
5. **Regla de oro.** Ningún módulo de `atlas_push/engine`,
   `atlas_push/state` o `atlas_push/outputs` importa símbolos del
   brain core mayor de ATLAS (`brain_core`, `mission_manager`,
   `safety_kernel`, `state_bus`, `arbitration`, `policy_store`,
   `modules.command_router`). Los contratos se expresan vía
   `typing.Protocol`.
6. **Sin sofisticación prematura.** D usa defaults simples
   (`field(default_factory=dict)`), sin `MappingProxyType` ni
   `__slots__`, sin `StrategyProposal` como Protocol. El scaffold fija
   layout y firma, no endurece inmutabilidad profunda ni introduce
   optimizaciones.
7. **Pureza.** `DecisionEngine` no guarda estado de negocio. Mismo
   `MarketState` → misma `DecisionOutput`. En D, además, cualquier
   estado produce la misma salida (pass-through).
8. **No se cierra ningún defecto en D.** `DEFECTS.md` no se modifica.

## 3. Estado previo (diagnóstico sobre `main@f930c0bf`)

Sobre `atlas_push/` tras el merge de C (PR #10):

- `atlas_push/__init__.py`: docstring listando `engine`, `state`,
  `outputs` como "próximos pasos (no implementados aún)".
- `atlas_push/intents/`: `IntentRouter`, `IntentResult`, `Kind` con
  68 tests en verde (PR #9, B).
- No existen en el repo `atlas_push/engine/`, `atlas_push/state/` ni
  `atlas_push/outputs/`.
- No existen en el repo los tipos `MarketState`, `AccountSnapshot`,
  `Position`, `Quote`, `RiskContext`, `DecisionOutput`,
  `LogicalOrder`, `TargetWeight`, `RiskVeto`.
- `atlas_adapter/atlas_http_api.py` quedó limpio en C (11 tests de
  contrato HTTP, D-002/D-003 cerrados).
- Suite unit total en `main`: 79 tests passing.

D añade tres paquetes nuevos y sus tests sin tocar ese suelo.

## 4. Alcance de D

### Se toca

- `atlas_push/__init__.py` (edita re-exports y docstring, en D1).
- `atlas_push/state/__init__.py` (nuevo, en D1).
- `atlas_push/state/market_state.py` (nuevo, en D1).
- `atlas_push/outputs/__init__.py` (nuevo, en D1).
- `atlas_push/outputs/decision_output.py` (nuevo, en D1).
- `atlas_push/engine/__init__.py` (nuevo, en D1).
- `atlas_push/engine/decision_engine.py` (nuevo, en D1).
- `tests/unit/test_market_state.py` (nuevo, en D2).
- `tests/unit/test_decision_output.py` (nuevo, en D2).
- `tests/unit/test_decision_engine.py` (nuevo, en D2).
- `docs/atlas_push/PLAN_STEP_D.md` (este archivo, en D3).
- `docs/atlas_push/CURRENT_STATE.md` (nota post-D al inicio, en D3).

### No se toca en D

- `atlas_adapter/atlas_http_api.py` (paridad verificada con
  `git diff main -- atlas_adapter/atlas_http_api.py` vacío).
- `atlas_push/intents/` entero. Los 68 tests de B siguen en verde.
- `modules/command_router.py` y cualquier otro módulo de `modules/`.
- `docs/atlas_push/ARCHITECTURE.md`, `DEFECTS.md`.
- `tests/unit/test_intent_router.py` (68 tests) y
  `tests/unit/test_http_api_contract.py` (11 tests).
- `tests/smoke/*` (`a2_parity_check.sh`, `b_parity_check.sh`).
- `03_run_atlas_api.ps1` y cualquier otro `.ps1`.
- `legacy/`, canales (Telegram, GUI, bridge).

## 5. D1 — Código del scaffold

### 5.1 Layout nuevo

```
atlas_push/
├── __init__.py                (edita re-exports + docstring)
├── intents/                   (intocado)
├── engine/
│   ├── __init__.py
│   └── decision_engine.py
├── state/
│   ├── __init__.py
│   └── market_state.py
└── outputs/
    ├── __init__.py
    └── decision_output.py
```

`atlas_push/engine/pipeline.py` queda **diferido**. Se introducirá
cuando aparezca la primera estrategia real y justifique un archivo
de orquestación. Introducir un `pipeline.py` vacío en D sería
sofisticación prematura.

Tampoco se crean en D `atlas_push/strategies/`, `atlas_push/risk/`,
`atlas_push/ports/` ni `atlas_push/config/`. Son pasos posteriores.

### 5.2 `atlas_push/state/market_state.py` (contrato §3.1)

Dataclasses inmutables con los campos fijados en `ARCHITECTURE.md`
§3.1:

- `AccountSnapshot`: `equity`, `cash`, `buying_power`,
  `realized_pnl_today`.
- `Position`: `symbol`, `qty`, `avg_price`, `market_value`,
  `unrealized_pnl`.
- `Quote`: `symbol`, `bid`, `ask`, `last`, `as_of`.
- `RiskContext`: `halted: bool = False`, `reasons: tuple[str, ...]`,
  `limits: Mapping[str, float] = dict`.
- `MarketState`: `as_of`, `account`, `positions: tuple[Position, ...]`,
  `quotes: Mapping[str, Quote]`, `indicators: Mapping[str, Mapping]`,
  `risk_context`, `meta`. Los mappings usan `dict` como factory; el
  tipo expuesto es `Mapping` para indicar intención de solo-lectura.

### 5.3 `atlas_push/outputs/decision_output.py` (contrato §3.2)

Dataclasses inmutables con los campos fijados en `ARCHITECTURE.md`
§3.2:

- `OrderSide = Literal["buy", "sell"]`.
- `OrderKind = Literal["market", "limit"]`.
- `LogicalOrder`: `symbol`, `side: OrderSide`, `qty`, `strategy_id`,
  `reason`, `kind: OrderKind = "market"`, `limit_price: float | None =
  None`, `meta`. Nota: el campo se llama `kind`, **no** `order_type`.
- `TargetWeight`: `symbol`, `weight`, `strategy_id`, `reason`, `meta`.
- `RiskVeto`: `code`, `message`, `symbol: str | None = None`,
  `strategy_id: str | None = None`, `meta`.
- `DecisionOutput`: `orders`, `target_weights`, `vetoes`,
  `notes: tuple[str, ...]`, `meta`. Método `classmethod empty()` que
  devuelve una instancia canónica vacía (misma igualdad estructural
  entre llamadas).

### 5.4 `atlas_push/engine/decision_engine.py` (contrato §3.3)

- `StrategyProposal`: `@dataclass(frozen=True)` vacía. Placeholder
  tipado para fijar la firma de `Strategy.propose`; su contenido
  real se definirá cuando aparezcan estrategias.
- `Strategy`: `@runtime_checkable Protocol` con
  `propose(self, state: MarketState) -> StrategyProposal`.
- `RiskManager`: `@runtime_checkable Protocol` con
  `apply(self, state: MarketState, draft: DecisionOutput) ->
  DecisionOutput`.
- `DecisionEngine`: `@dataclass(frozen=True)` con
  `strategies: tuple[Strategy, ...] = field(default_factory=tuple)` y
  `risk: RiskManager | None = None`. Método `decide(state) ->
  DecisionOutput` que en D devuelve `DecisionOutput.empty()` sin
  invocar ni estrategias ni risk.

### 5.5 `atlas_push/__init__.py`

Se re-exportan los símbolos nuevos junto con los de `intents`:

- `IntentRouter`, `IntentResult`, `Kind`.
- `DecisionEngine`, `Strategy`, `RiskManager`.
- `MarketState`, `AccountSnapshot`, `Position`, `Quote`,
  `RiskContext`.
- `DecisionOutput`, `LogicalOrder`, `TargetWeight`, `RiskVeto`.

El docstring pasa de "próximos pasos (no implementados aún)" a
documentar el estado real tras D, incluyendo los dos invariantes
vinculantes: regla de oro y "hermanos, no anidados".

## 6. D2 — Tests que fijan el contrato

Objetivo: que la forma de los tipos y el comportamiento del scaffold
no puedan moverse sin que un test lo grite. Tres archivos nuevos
bajo `tests/unit/`. Sin `fastapi.testclient`, sin uvicorn, sin disco
real; tests hermenéuticos en Linux/macOS/Windows.

### 6.1 `test_market_state.py` (14 tests)

- Los 5 tipos son `@dataclass(frozen=True)`: `AccountSnapshot`,
  `Position`, `Quote`, `RiskContext`, `MarketState` (parametrizado).
- `MarketState` se construye con solo `as_of` y `account`; el resto
  tiene defaults sanos (`()`, `{}`, `RiskContext()`).
- Igualdad estructural de `MarketState` y `RiskContext`.
- `MarketState.positions` y `RiskContext.reasons` son `tuple`.
- Defaults mutables (`quotes`, `indicators`, `meta`) son
  independientes entre instancias.
- Campos declarados exactos en `AccountSnapshot`, `Position` y
  `MarketState` (guardrail contra cambios accidentales).

### 6.2 `test_decision_output.py` (16 tests)

- Los 4 tipos son `@dataclass(frozen=True)`: `LogicalOrder`,
  `TargetWeight`, `RiskVeto`, `DecisionOutput` (parametrizado).
- `LogicalOrder` y `DecisionOutput` rechazan mutación con
  `FrozenInstanceError`.
- `LogicalOrder` usa `kind`, **no** `order_type`.
- `LogicalOrder` construido mínimo tiene `kind == "market"` y
  `limit_price is None`.
- `LogicalOrder(kind="limit", limit_price=150.0)` se construye OK.
- `typing.get_args(OrderSide) == {"buy", "sell"}`; idem
  `OrderKind == {"market", "limit"}`.
- `RiskVeto(code, message)` mínimo deja `symbol/strategy_id=None` y
  `meta == {}`.
- `DecisionOutput.empty()` tiene todas las colecciones vacías.
- `DecisionOutput.empty() == DecisionOutput.empty()` por estructura.
- Colecciones de `DecisionOutput` son `tuple`.
- Campos declarados exactos en `DecisionOutput`.
- Defaults `meta` independientes entre instancias.

### 6.3 `test_decision_engine.py` (14 tests)

- `DecisionEngine` es `@dataclass(frozen=True)`; `setattr` lanza
  `FrozenInstanceError`.
- `DecisionEngine()` sin args → `strategies == ()` (tupla), `risk is
  None`.
- `DecisionEngine(strategies=(s,), risk=r)` almacena ambos.
- `StrategyProposal` es dataclass frozen vacía.
- `decide(state)` devuelve `DecisionOutput.empty()` para cualquier
  estado.
- Pureza: `decide(st) == decide(st)`.
- En D, estados distintos (`hour=13` vs `hour=14`) producen la misma
  salida (pass-through).
- El scaffold no ejecuta las estrategias configuradas
  (`RecordingStrategy` registra `[]`).
- `Strategy` y `RiskManager` son `runtime_checkable`; implementaciones
  dummy pasan `isinstance`.
- **Regla de oro a nivel de fuente** (parametrizado sobre `engine`,
  `state`, `outputs`): ningún `.py` del paquete contiene
  `from|import` de `brain_core`, `mission_manager`, `safety_kernel`,
  `state_bus`, `arbitration`, `policy_store` o
  `modules.command_router`. Implementado con regex multiline; solo
  matchea imports reales, no menciones en docstrings.
- **Invariante de hermanos**: ningún `.py` de `atlas_push/engine/`
  importa `atlas_push.intents`, y ningún `.py` de
  `atlas_push/intents/` importa `atlas_push.engine`. Dos tests
  simétricos.

Totales D2: **44 tests nuevos**. Suite completa tras D: **123 tests
passing** (79 previos + 44 nuevos).

### 6.4 Smokes externos (bash)

No se añade un `d_parity_check.sh`. D no cambia la superficie HTTP.
Los smokes `a2_parity_check.sh` (9/9) y `b_parity_check.sh` (14/14)
deben seguir en verde sin modificación contra la rama D. La
validación HTTP en máquina del owner es opcional pero bienvenida.

## 7. D3 — Docs

### 7.1 `docs/atlas_push/PLAN_STEP_D.md`

Este mismo archivo. Formato alineado con `PLAN_STEP_B.md` y
`PLAN_STEP_C.md` (10 secciones, idéntica voz).

### 7.2 `docs/atlas_push/CURRENT_STATE.md`

Se añade una **nota post-D** al inicio (paralela a la "Nota post-A2"
ya presente) que:

- Señala que tras el merge de D existen tres paquetes nuevos
  dentro de `atlas_push/`: `engine/`, `state/`, `outputs/`.
- Remite a `ARCHITECTURE.md` §3 y a este documento para el contrato
  de tipos.
- Aclara que el snapshot histórico de la sección 4 **no** se
  reescribe; el perímetro vivo vive en `ARCHITECTURE.md` y en los
  planes B/C/D.

No se modifica el resto de `CURRENT_STATE.md`, igual que se hizo tras
A2. Preservamos el documento como snapshot histórico; la evolución
se registra solo con notas iniciales.

### 7.3 `docs/atlas_push/DEFECTS.md`

**No se modifica en D.** D no cierra defectos.

### 7.4 `docs/atlas_push/ARCHITECTURE.md`

**No se modifica en D.** La arquitectura ya describe lo que D
implementa; cambiar el documento objetivo mientras cumplimos su §3
sería contradictorio.

## 8. D3 — PR y commits

### 8.1 Un solo PR

- Rama: `feat/atlas-push-d-decision-engine-scaffold`.
- Título: `feat(atlas_push): introduce DecisionEngine scaffold (D)`.
- Base: `main`. Head: la rama anterior.
- PR body: resumen de D1/D2/D3, tabla de cambios, referencia a este
  documento, qué no cambia y checklist de validación. Mismo formato
  que PR #9 (B) y PR #10 (C).

### 8.2 Tres commits atómicos

**D1 — `feat(atlas_push): add MarketState, DecisionOutput and
DecisionEngine scaffold (D1)`**

- Toca solo `atlas_push/__init__.py` y los 6 archivos nuevos bajo
  `atlas_push/{state,outputs,engine}/`.
- Introduce los tipos, los Protocols y el `DecisionEngine`
  pass-through. Sin tests, sin docs.

**D2 — `test(atlas_push): pin DecisionEngine and value-type
contracts (D2)`**

- Añade los tres archivos de tests (`test_market_state.py`,
  `test_decision_output.py`, `test_decision_engine.py`) con los 44
  tests que fijan los invariantes de §6.
- No modifica código de producción.

**D3 — `docs(atlas_push): add PLAN_STEP_D.md and update
CURRENT_STATE.md (D3)`**

- Añade `docs/atlas_push/PLAN_STEP_D.md` (este documento).
- Añade la nota post-D al inicio de `docs/atlas_push/CURRENT_STATE.md`.
- No toca `DEFECTS.md` ni `ARCHITECTURE.md`.

### 8.3 Validación previa al push

Antes de pedir aprobación para el push:

- `git diff main..HEAD` completo (los 3 commits).
- `python -m pytest tests/unit -q` en verde (123 tests: 79 previos +
  44 nuevos).
- `git diff main -- atlas_adapter/atlas_http_api.py` devuelve vacío
  (el adapter no se toca en D).
- `git diff main -- atlas_push/intents/` devuelve vacío (intents no
  se toca en D).
- Revisión estática:
  `grep -rnE "brain_core|mission_manager|safety_kernel|state_bus|
  arbitration|policy_store|modules\.command_router"
  atlas_push/engine atlas_push/state atlas_push/outputs` devuelve
  solo docstrings (no imports).
- Opcional, en máquina del owner: `a2_parity_check.sh` y
  `b_parity_check.sh` siguen en verde contra la rama D con la API
  viva en `127.0.0.1:8791`.

### 8.4 Protocolo de push

Como A, B y C: nada de push hasta que el owner apruebe el diff
local. Secuencia:

1. Crear la rama y hacer los 3 commits en el sandbox.
2. Enseñar diff + salida de tests por cada commit.
3. El owner aprueba cada sub-paso (D1, D2, D3).
4. El owner da OK final al paquete; push a origin.
5. Abrir el PR contra `main` con el body redactado.
6. El owner hace el merge desde `C:\TEMP_ATLAS_CORE\atlas-core` como
   en A1, A2, B y C.

## 9. Riesgos y mitigaciones

| Riesgo | Mitigación |
|---|---|
| Que declarar tipos "por delante de su uso" induzca a cablear antes de tiempo | El plan es explícito: D es scaffold. `DecisionEngine` no se registra en el `IntentRouter` ni en `atlas_adapter/atlas_http_api.py`. Tests de hermanos lo blindan a nivel source. |
| Que `Mapping` expuesto con `dict` como factory dé falsa sensación de inmutabilidad | `frozen=True` evita reemplazar el campo; el contenido del mapping sí es mutable, pero esa decisión se tomó explícitamente para no inflar D con `MappingProxyType`. Cuando sea necesario endurecer, se hace en un paso posterior con su propio test. |
| Que `StrategyProposal` vacío condicione el diseño de estrategias futuras | El placeholder es deliberadamente mínimo. Cuando aparezca la primera estrategia real, `StrategyProposal` ganará campos; la firma `propose(state) -> StrategyProposal` no cambia. |
| Que la regla de oro a nivel source dé falsos positivos por menciones en docstrings | La regex exige prefijo `from|import` al inicio de línea. Verificado en D2 con `test_source_does_not_import_major_brain_core_symbols` parametrizado sobre los 3 paquetes. |
| Que un cambio futuro introduzca un import indirecto que rompa hermanos sin que los tests lo vean | Los tests de hermanos son source-level, no de import-time. Un nuevo `.py` en `engine/` que importe `atlas_push.intents` caería inmediatamente en `test_engine_does_not_import_intents`. |
| Regresión de B/C por tocar `__init__.py` | `python -m pytest tests/unit -q` corre los 79 previos más los 44 nuevos. Cualquier regresión se ve en verde/rojo antes del push. |
| Que el scaffold se perciba "inútil" por no hacer nada | Es deliberado. D fija layout y tipos; la utilidad aparece cuando estrategias reales y el cableado de intents de cerebro se introduzcan en pasos posteriores. Cuanto más sobrio sea D, más barato será iterar después. |

## 10. Out of scope (no llega en D)

- **Cableado `IntentRouter → DecisionEngine`.** No es parte de D.
  Llegará cuando existan intents de cerebro (`/pnl`, `/positions`,
  `/pause-trading`) y un `MarketStateProvider` que los alimente.
- **Estrategias reales.** `atlas_push/strategies/` no se crea en D.
- **Capa de riesgo real.** `atlas_push/risk/` no se crea en D.
- **Puertos externos.** `atlas_push/ports/` (Protocols hacia data,
  execution, journal) es parte de E, no de D.
- **Config.** `atlas_push/config/settings.py` es parte de E.
- **`pipeline.py`.** Diferido por decisión del owner; se creará
  cuando la primera orquestación real lo justifique.
- **Adapter HTTP.** `atlas_adapter/atlas_http_api.py` no se toca en D.
  Cualquier cambio en la superficie HTTP es otro PR.
- **Mover, renombrar o reorganizar `atlas_push/intents/`.** No se
  toca.
- **Cierre de defectos.** D no cierra nada de `DEFECTS.md`.

## 11. Criterio de aceptación del PR D

- [x] `atlas_push/{engine,state,outputs}/` existen como paquetes
  nuevos con sus `__init__.py` y módulos dedicados.
- [x] `MarketState` y `DecisionOutput` declarados con todos los
  campos de `ARCHITECTURE.md` §3.1/§3.2 y `@dataclass(frozen=True)`.
- [x] `DecisionEngine.decide(state)` devuelve
  `DecisionOutput.empty()` para cualquier estado (pass-through).
- [x] `LogicalOrder` usa `side: OrderSide`, `kind: OrderKind`
  (default `"market"`) y **no** expone `order_type`.
- [x] `StrategyProposal` es `@dataclass(frozen=True)` vacía;
  `Strategy` y `RiskManager` son `runtime_checkable Protocol`.
- [x] `python -m pytest tests/unit -q` en verde con **123 tests**
  (79 previos + 44 nuevos).
- [x] Regla de oro respetada a nivel source en
  `atlas_push/{engine,state,outputs}/`.
- [x] Invariante de hermanos respetado en ambas direcciones.
- [x] `git diff main -- atlas_adapter/atlas_http_api.py` vacío.
- [x] `git diff main -- atlas_push/intents/` vacío.
- [x] `docs/atlas_push/PLAN_STEP_D.md` añadido.
- [x] Nota post-D añadida al inicio de
  `docs/atlas_push/CURRENT_STATE.md`.
- [ ] Smokes `a2_parity_check.sh` (9/9) y `b_parity_check.sh`
  (14/14) siguen en verde contra la rama D (validación en máquina
  del owner).
- [x] Diff revisado y aprobado por el owner antes del push.
- [ ] PR abierto contra `main` con título
  `feat(atlas_push): introduce DecisionEngine scaffold (D)`.
