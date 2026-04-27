# F16 — Paper Execution Pipeline (Atlas Code Quant)

## Política

F16 cierra el bloque F11–F15 conectando, **únicamente en papel**, los
componentes ya construidos:

```
Radar opportunity (F5/F10 gate)
    └── StrategyFactory (F12)
            └── StrategyWithFitness (F14 = F12 + F13)
                    └── TradierAdapter.submit (F15, dry-run)
```

La regla principal del bloque se mantiene intacta:

> **Scanner propone. Radar decide. Estrategias y ejecución sólo operan
> sobre oportunidades aprobadas por Radar, en papel hasta que se
> defina un bloque posterior de autonomía y live-readiness.**

F16 NO autoriza live trading bajo ninguna circunstancia: aunque se le
inyecte un `TradierAdapter` con `dry_run=False`, el pipeline lo
re-instancia explícitamente con `dry_run=True` y emite un warning.

## Alcance (in / out)

In:

- Orquestación **end-to-end** Radar → Strategy → fitness → Tradier
  paper para una o varias oportunidades.
- Filtros declarativos (`min_score`, `min_sharpe`, `only_evaluated`,
  `max_strategies_per_opportunity`).
- Mapeo `StrategyIntent` (F11) → `OrderIntent` (F15).
- Forzado de `dry_run=True` sobre el adapter.
- Defensividad total: ningún paso lanza hacia fuera.
- Registro estructurado por intento (`PaperPipelineRecord`) y resumen
  (`PaperPipelineReport`).

Out (NO entra en F16):

- Live trading, broker live, autonomous executor, live authorization,
  live loop, operation center, signal executor.
- Construcción de tickers Tradier reales (`SPY230721C00450000`).
- Risk gates dinámicos por cuenta / PDT / margen.
- Persistencia/ledger de órdenes (otros bloques posteriores).
- Cualquier mutación sobre `atlas_adapter` o el Radar.

## Aislamiento

Módulo `atlas_code_quant/operations/paper_pipeline.py`. Sólo importa:

- `atlas_code_quant.execution.tradier_adapter` (F15)
- `atlas_code_quant.intake.opportunity` (F0/F5)
- `atlas_code_quant.strategies.evaluation` (F14)
- `atlas_code_quant.strategies.options.intent` (F11)

NO importa (verificado con AST guard en
`tests/atlas_code_quant/test_paper_pipeline_f16.py`):

- `atlas_code_quant.operations.{auton_executor, live_authorization,
  live_loop, live_switch, operation_center, signal_executor,
  start_paper_trading}`
- `atlas_code_quant.production.live_activation`
- `atlas_code_quant.execution.{broker_router, tradier_execution,
  tradier_controls, tradier_pdt_ledger}`
- `atlas_adapter`

Tampoco referencia identificadores `paper_only` ni
`full_live_globally_locked` como `Name` ni `Attribute` (los locks
globales no se tocan desde aquí).

## API pública

### Config

```python
@dataclass(frozen=True)
class PaperPipelineConfig:
    min_score: float = 70.0
    min_sharpe: float | None = 0.0
    only_evaluated: bool = True
    max_strategies_per_opportunity: int | None = 1
    default_quantity: int = 1
```

- `min_score`: pasa a F12/F14 como floor de Radar score.
- `min_sharpe`: si no es None, descarta intents con
  `fitness.sharpe < min_sharpe`.
- `only_evaluated`: descarta intents con `fitness.success=False`.
- `max_strategies_per_opportunity`: top-N por opp después de filtros
  (None = sin tope).
- `default_quantity`: cantidad enviada en `OrderIntent`.

### Modelos

```python
@dataclass(frozen=True)
class PaperPipelineRecord:
    opportunity_symbol: str
    strategy_type: str
    trace_id: str
    fitness_success: bool
    sharpe: float
    submit_ok: bool
    dry_run: bool
    order_id: str | None
    error_code: str | None
    error_message: str | None
    raw_intent: dict          # debug only (repr/compare excluidos)
    raw_order_result: dict    # debug only (repr/compare excluidos)

@dataclass(frozen=True)
class PaperPipelineReport:
    total_opportunities: int
    total_intents: int
    total_submitted: int
    total_ok: int
    records: tuple[PaperPipelineRecord, ...]
```

### Funciones

```python
def run_paper_pipeline_for_opportunity(
    opp: RadarOpportunityInternal | None,
    *,
    config: PaperPipelineConfig | None = None,
    adapter: TradierAdapter | None = None,
    backtest_fn: Callable[[StrategyIntent], StrategyFitnessResult] | None = None,
    evaluator_fn: Callable[..., list[StrategyWithFitness]] | None = None,
) -> PaperPipelineReport: ...

def run_paper_pipeline_for_opportunities(
    opps: Iterable[RadarOpportunityInternal] | None,
    *,
    config=None,
    adapter=None,
    backtest_fn=None,
    evaluator_fn=None,
) -> PaperPipelineReport: ...
```

Parámetros inyectables (todos defensivos):

- `adapter`: si es None se construye `TradierAdapter(TradierAdapterConfig(dry_run=True))`.
  Si tiene `dry_run=False`, se re-instancia preservando base_url/token/limits
  pero forzando `dry_run=True` (warning).
- `backtest_fn`: pasado a F14; default usa F13 (`run_backtest_for_strategy_intent`,
  con mock determinista cuando `ATLAS_LEAN_ENABLED=False`).
- `evaluator_fn`: por defecto `evaluate_strategies_for_opportunity` (F14).

## Mapping `StrategyIntent` → `OrderIntent`

```
StrategyIntent (F11)              OrderIntent (F15)
-----------------------          --------------------
opportunity.symbol         →     symbol
opportunity.trace_id       →     trace_id
strategy_type              →     metadata["strategy_type"]
metadata["structure"]      →     metadata["structure"]
metadata["dte"]            →     metadata["dte"]
legs (StrategyLeg…)        →     legs (tuple[dict])
                                   {side, right, strike_offset_steps,
                                    expiry_rel_dte, quantity}
num_legs > 1               →     asset_class = "multileg"
num_legs == 1              →     asset_class = "option"
                                   side = "buy_to_open"
                                   order_type = "market"
                                   duration = "day"
                                   quantity = config.default_quantity
```

La conversión **no** construye tickers Tradier reales. F16 deja los
strikes/expiries en formato relativo (offset_steps, dte) para que una
fase posterior (chain resolver) los traduzca a símbolos OCC. F15
acepta este payload como `class="multileg"` genérico.

## Dry-run forzado

`_ensure_dry_run_adapter` aplica una invariante dura:

```python
if adapter is None:
    return TradierAdapter(TradierAdapterConfig(dry_run=True))
if not adapter.config.dry_run:
    forced = replace_with(dry_run=True)
    logger.warning("paper_pipeline: forced dry_run=True (input adapter was live)")
    return TradierAdapter(forced)
return adapter
```

El test `TestDryRunEnforced::test_live_adapter_is_forced_to_dry_run`
inyecta un adapter live espía y verifica que NO recibe ninguna
llamada (F16 lo descarta y crea uno dry-run nuevo).

## Tests

`tests/atlas_code_quant/test_paper_pipeline_f16.py` — 19 tests:

1. `TestPipelineSingle` (4): happy path, opp None, opp tipo inválido,
   opp sin intents (reject).
2. `TestDryRunEnforced` (2): adapter default dry-run; live adapter
   forzado a dry-run.
3. `TestFilters` (3): `only_evaluated`, `min_sharpe`,
   `max_strategies_per_opportunity`.
4. `TestBatch` (2): multi-opp con reject incluido; batch None.
5. `TestDefensive` (2): evaluator que lanza; adapter que lanza.
6. `TestMapping` (3): multileg vs option, legs preservadas, metadata
   propagada.
7. AST guards (3): no imports prohibidos; sin locks como Name/Attribute;
   dependencias esperadas presentes.

Resultado: **19 passed**.

## Invariantes

- Ningún paso del pipeline lanza hacia fuera; todo error queda
  registrado como `PaperPipelineRecord(submit_ok=False, ...)` o
  `records=()` cuando el evaluator falla.
- `PaperPipelineRecord.dry_run` es siempre `True` por construcción.
- F16 no escribe a disco, no toca env vars de Tradier ni LEAN, no
  abre HTTP por sí mismo.
- F16 no autoriza ni desactiva locks globales (`paper_only`,
  `full_live_globally_locked`); ni siquiera referencia esos
  identificadores como código.

## Rollback

Para revertir F16 sin afectar F11–F15:

```bash
cd /home/user/workspace/atlas-core
git revert <commit_F16>
# o
git rm atlas_code_quant/operations/paper_pipeline.py \
       tests/atlas_code_quant/test_paper_pipeline_f16.py \
       docs/ATLAS_CODE_QUANT_F16_PAPER_EXECUTION_PIPELINE.md
```

Nada en F11–F15 depende de F16, por diseño.
