# Atlas Push · Arquitectura objetivo

> Documento de diseño. No describe el estado actual del repo (eso vive en
> `CURRENT_STATE.md`). Describe hacia dónde vamos.
>
> Estado: **aprobado por el owner del proyecto, sin implementación todavía.**
> Las interfaces aquí descritas no existen aún como código; son el contrato
> que guiará los pasos B–E del plan de refactor.

## 1. Misión de Atlas Push

Atlas Push es el **núcleo local de decisiones de trading** dentro del
ecosistema ATLAS. Su responsabilidad es convertir un estado normalizado
de mercado y cartera en un conjunto de decisiones (órdenes lógicas,
pesos objetivo o vetos de riesgo), aplicando una capa central de
gestión de riesgo.

Atlas Push **no** se encarga de:

- conectarse al broker o a websockets de datos,
- renderizar interfaces visuales,
- orquestar el robot físico o hardware,
- ejecutar órdenes en un motor externo (LEAN, broker API, etc.).

Atlas Push **sí** se encarga de:

- consumir un `MarketState` normalizado,
- generar decisiones (`DecisionOutput`),
- aplicar una capa central de riesgo,
- exponer una interfaz de enrutamiento de intents (texto → acción)
  compatible con el `command_router` actual.

## 2. Dos caras del núcleo

El núcleo tiene dos componentes hermanos, no uno dentro de otro:

- `IntentRouter` (texto → `IntentResult`): heredero del actual
  `modules/command_router.py`. Atiende comandos administrativos
  (`/status`, `/doctor`, `/note`, `/snapshot`) y, en el futuro,
  consultas al cerebro (`/pnl`, `/positions`, `/pause-trading`).
- `DecisionEngine` (`MarketState` → `DecisionOutput`): nuevo. Ejecuta
  un pipeline `Strategies → Portfolio → RiskManager → Emisor`.
  Nunca recibe texto de usuario.

La entrada de texto de un canal (Telegram, HTTP `/intent`, GUI, CLI)
llega siempre al `IntentRouter`. Si el intent es una consulta al
cerebro, el router delega al `DecisionEngine` o a un `JournalSink`.
Si es administrativo, lo resuelve directamente.

## 3. Contratos principales

Firmas. Detalles de tipos en `atlas_push/state/`, `atlas_push/outputs/`,
`atlas_push/strategies/`, `atlas_push/risk/`, `atlas_push/intents/`.

```
IntentRouter.handle(text: str)            -> IntentResult
DecisionEngine.decide(state: MarketState) -> DecisionOutput
Strategy.propose(state: MarketState)      -> StrategyProposal
RiskManager.apply(state, draft)           -> DecisionOutput
```

### 3.1 MarketState (entrada al cerebro)

Dataclass inmutable con:

- `as_of: datetime`
- `account: AccountSnapshot` (equity, cash, buying_power,
  realized_pnl_today)
- `positions: tuple[Position, ...]` (symbol, qty, avg_price,
  market_value, unrealized_pnl)
- `quotes: dict[str, Quote]`
- `indicators: dict[str, dict]`  (libre, por símbolo)
- `risk_context: RiskContext` (límites vigentes + kill-switch)
- `meta: dict`

Reglas:

- Inmutable (`frozen=True`).
- No expone nada de broker, websocket o API.
- Ampliar `MarketState` requiere decisión explícita; no se enchufan
  campos por la puerta de atrás.

### 3.2 DecisionOutput (salida del cerebro)

Dataclass inmutable con:

- `orders: tuple[LogicalOrder, ...]`
- `target_weights: tuple[TargetWeight, ...]`
- `vetoes: tuple[RiskVeto, ...]`
- `notes: tuple[str, ...]`  (para journal)
- `meta: dict`

Reglas:

- Las decisiones pueden expresarse como órdenes, pesos, o ambos.
  El motor de ejecución es quien materializa los pesos.
- Los vetos de riesgo viajan **dentro** del output, no como
  excepciones.
- Cada orden lleva `strategy_id` y `reason` para trazabilidad.

### 3.3 Strategy, RiskManager, DecisionEngine

- `Strategy` es un `Protocol`. Implementaciones pueden vivir en
  `atlas_push/strategies/` o en paquetes externos (ej.
  `atlas_code_quant`) sin herencia obligatoria.
- `RiskManager` es un `Protocol`. Recibe `(state, draft)` y devuelve
  `DecisionOutput` final (con recortes o vetos). No lanza
  excepciones por reglas de negocio; las expresa como `RiskVeto`.
- `DecisionEngine` es sin estado, composable:
  `DecisionEngine(strategies=..., risk=...).decide(state)`.
  Mismo estado → misma decisión.

### 3.4 Puertos externos (Protocols, sin implementación en este repo)

- `MarketStateProvider.snapshot() -> MarketState`
- `ExecutionEngineAdapter.submit(DecisionOutput) -> ExecutionReport`
- `JournalSink.record(state, output) -> None`

Atlas Push **no** importa broker, LEAN ni observabilidad.
Son contratos que otros paquetes/servicios implementan.

## 4. Estructura de paquete

```
atlas_push/
├── __init__.py
├── engine/
│   ├── decision_engine.py
│   └── pipeline.py
├── strategies/         # generación de señales (ver §6 mapeo)
│   ├── base.py
│   └── registry.py
├── risk/               # capa central de gestión de riesgo
│   ├── risk_manager.py
│   └── limits.py
├── state/
│   └── market_state.py
├── outputs/
│   └── decision_output.py
├── intents/            # heredero de command_router
│   └── intent_router.py
├── ports/              # contratos con sistemas externos
│   ├── data.py
│   ├── execution.py
│   └── journal.py      # contrato de salida del "journal"
└── config/
    └── settings.py
```

## 5. Qué queda fuera del paquete Atlas Push

Viven en el repo pero **no** dentro de `atlas_push/`:

- Canales de entrada: Telegram, GUI Tk, HTTP adapter, CLI.
- Ejecución real en broker / LEAN.
- LLM, visión, voz (servicios auxiliares inyectables si un
  componente los necesita — nunca en `MarketState`).
- Ops: snapshots, doctor, logs, memoria vault, notas.

El `atlas_adapter/atlas_http_api.py` actual seguirá existiendo como
adapter HTTP de entrada; llamará a `IntentRouter` por dentro pero
no será parte de `atlas_push/`.

## 6. Mapeo de vocabulario: propuesta ↔ brief original

El brief original del owner habló de `signals/`, `risk/`, `journal/`.
El paquete real usa:

| Carpeta en `atlas_push/` | Concepto en el brief original | Nota |
|---|---|---|
| `strategies/` | señales / generación de señales | Una "señal" es el resultado de `Strategy.propose()`. No necesita carpeta propia. |
| `risk/` | riesgo | Mismo nombre, mismo rol. |
| `ports/journal.py` | journal | Journal es una **salida** del cerebro, se modela como puerto (contrato), no como módulo interno. |
| `state/` | (no explícito en brief) | Agrupa `MarketState` y tipos asociados. |
| `outputs/` | (no explícito en brief) | Tipos de decisión: `LogicalOrder`, `TargetWeight`, `RiskVeto`. |
| `intents/` | heredero de `command_router` | Separa enrutamiento textual del motor de decisiones. |
| `engine/` | (pieza central implícita) | Orquesta el pipeline de decisión. |
| `ports/` | (no explícito en brief) | Contratos hacia sistemas externos: data, execution, journal. |

No se crean alias físicos `signals/` ni `journal/`. El mapeo vive
aquí.

## 7. Compatibility with broader ATLAS brain architecture

> Esta sección es **conservadora por diseño**. Describe cómo Atlas Push
> *podría* encajar con un brain core mayor cuya especificación formal
> aún no está cerrada. **Este mapeo es conceptual, no de import
> directo ni de dependencia de código.** Ninguna afirmación aquí
> compromete una integración exacta; todas las conexiones se
> presentan como hipótesis a refinar cuando la spec del brain core
> mayor esté disponible.

El ecosistema mayor de ATLAS define un brain core con, al menos, los
siguientes componentes (vocabulario del owner, definición detallada
en otros repos/frentes de ATLAS):

- `brain_core`
- `mission_manager`
- `safety_kernel`
- `state_bus`
- `command_router`
- `arbitration`
- `policy_store`

Este repo **no** define todo ese brain core. Atlas Push es un
**núcleo local especializado en trading** y debe encajar como un
subsistema más dentro de esa arquitectura mayor sin introducir una
taxonomía paralela incompatible.

Mapeo conceptual propuesto entre Atlas Push y el brain core mayor
(hipótesis, no afirmación de implementación):

| Concepto mayor de ATLAS | Relación hipotética con Atlas Push | Observación |
|---|---|---|
| `command_router` | `IntentRouter` (en `atlas_push/intents/`) podría ser una implementación especializada, enfocada en intents de trading y ops del cerebro, de lo que el `command_router` mayor representa. | Firma `handle(text) -> IntentResult` pensada para poder ser despachada por un `command_router` mayor hacia múltiples subsistemas. |
| `brain_core` | `DecisionEngine` (en `atlas_push/engine/`) es un componente especializado que **podría ser hospedado, coordinado o consumido** por un `brain_core` mayor. | Atlas Push no pretende ser el `brain_core`; aporta un núcleo de decisión de dominio (trading) que un brain core mayor puede invocar. |
| `mission_manager` | No implementado aquí; potencial consumidor upstream. | Un `mission_manager` externo podría decidir *cuándo* invocar `DecisionEngine.decide()` o ajustar su `RiskContext`. Atlas Push no asume cómo se orquestan misiones; solo expone un contrato puro. |
| `safety_kernel` | Relación hipotética con `RiskManager` + `RiskContext.halted`. | El `RiskManager` local aplica riesgo específico de trading. Un `safety_kernel` mayor *podría* influir en Atlas Push forzando `halted=True` vía `RiskContext`, sin que Atlas Push necesite conocer su lógica interna. Los vetos de trading se expresan como `RiskVeto`; los vetos de seguridad de nivel superior, si existieran, llegarían pre-aplicados en el estado. Cómo se cablea esto en la práctica queda pendiente de la spec del safety_kernel. |
| `state_bus` | Hipótesis de integración con `MarketStateProvider` + `JournalSink`. No es una afirmación de implementación real. | Un `state_bus` mayor *podría* ser quien alimente `MarketState` (implementando `MarketStateProvider`) y/o reciba decisiones (implementando `JournalSink` o un puerto similar). Atlas Push no asume que el bus exista; solo consume un provider y empuja a un sink. |
| `arbitration` | No implementado aquí; potencial consumidor upstream. | Si varias fuentes de decisión (Atlas Push, otros núcleos) necesitan arbitrarse, ese arbitraje viviría **por encima** de Atlas Push. `DecisionOutput` es la salida que un árbitro consumiría. |
| `policy_store` | Relación parcial con `RiskContext` + `limits.py`. | Los límites de riesgo podrían venir externamente de un `policy_store`. En una primera fase se cargan desde config local; el contrato `RiskContext` está pensado para que un `policy_store` mayor lo provea sin cambios de API. |

### Asunciones explícitas (TBD pendientes de confirmar)

Estas asunciones se presentan como **hipótesis de integración**, no
como verdad arquitectónica cerrada. Se refinarán cuando la spec del
brain core mayor esté disponible.

1. El `command_router` mayor, si existe con esa forma, *podría*
   aceptar un subsistema especializado cuyo contrato sea
   `handle(text) -> IntentResult`. Si el contrato mayor resulta
   distinto, `IntentRouter` se envolverá en un adapter fino; su
   lógica interna no cambia.
2. El `safety_kernel`, si actúa antes de la decisión, *podría*
   modificar el `MarketState` (vía `RiskContext.halted` y `reasons`)
   antes de que llegue al `DecisionEngine`. Si actuase a posteriori
   sobre el `DecisionOutput`, añadiríamos un hook post-decisión en
   `engine/pipeline.py`.
3. El `state_bus`, si existe, *podría* ser el mecanismo natural para
   implementar `MarketStateProvider` y `JournalSink`, pero no se
   asume su existencia: cualquier proveedor/sink que implemente los
   Protocols encaja.
4. `policy_store`, si existe, *podría* integrarse como fuente de
   `RiskContext` y de parámetros de estrategia. No requiere cambios
   al contrato.

Cuando la spec del brain core mayor esté disponible, esta sección se
actualiza y cualquier divergencia se resuelve con adapters, **sin**
cambiar los contratos internos de Atlas Push.

### Regla de oro

Atlas Push **nunca** importa símbolos del brain core mayor. Define
sus propios Protocols y deja que otros implementen. Cualquier
incompatibilidad futura se resuelve con un adapter fino escrito
fuera de `atlas_push/`.

## 8. Decisiones de diseño explícitas

- El router de comandos actual es un `IntentRouter`, **no** un
  cerebro de trading. Se respeta su semántica actual y se amplía.
- El nuevo cerebro se construye **al lado** del código existente, no
  reemplazándolo. No hay *big rewrite*.
- Los vetos de riesgo viajan en el `DecisionOutput`, no como
  excepciones. Esto preserva la trazabilidad.
- Los Protocols se prefieren a herencia para maximizar
  interoperabilidad con paquetes externos (ej. `atlas_code_quant`).
- El paquete se llama `atlas_push/`. No se renombra a `brain/` para
  no colisionar con el `brain_core` mayor.
