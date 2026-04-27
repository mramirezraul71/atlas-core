# ATLAS_CODE_QUANT_F14 — Strategy fitness pipeline

## Política F14
- F14 conecta F12 (StrategyFactory) con F13 (LEAN adapter MVP) sin
  introducir ejecución real ni conectividad nueva.
- Pipeline puro de orquestación: ningún I/O propio, sólo combina
  outputs de F12 y F13.
- F14 NO toca `execution/`, `operations/`, `autonomy/`, `risk/`,
  `vision/`, `atlas_adapter/`, `tradier_*`, `broker_router`,
  `live_loop`, `live_activation`. Tampoco toca locks.

## Alcance
Toca SÓLO:
- `atlas_code_quant/strategies/evaluation.py` (nuevo)
- `tests/atlas_code_quant/test_strategy_evaluation_f14.py` (nuevo, 12 tests)
- `docs/ATLAS_CODE_QUANT_F14_STRATEGY_FITNESS_PIPELINE.md` (este doc)

## API pública

```python
from atlas_code_quant.strategies.evaluation import (
    StrategyWithFitness,
    evaluate_strategies_for_opportunity,
    evaluate_strategies_for_opportunities,
    rank_by_default_metric,
)
```

### `StrategyWithFitness`
- Pareja `(intent: StrategyIntent, fitness: StrategyFitnessResult)`.
- Helpers: `is_evaluated`, `primary_metric` (sharpe), `to_dict()`.

### `evaluate_strategies_for_opportunity(opp, *, min_score=70.0, backtest_fn=None, rank=True)`
1. Llama a `build_strategies_for_opportunity` (F12) con `min_score`.
2. Por cada intent llama a `backtest_fn` (default:
   `run_backtest_for_strategy_intent` de F13).
3. Empareja en `StrategyWithFitness`.
4. (opcional) Ordena por `(sharpe desc, expectancy desc, win_rate desc)`,
   con fallidos al final.

Defensivo:
- `opp` None / no `RadarOpportunityInternal` → `[]`.
- `backtest_fn` que lanza → `StrategyFitnessResult(success=False,
  error_code="EVALUATION_BACKTEST_RAISED")`.
- `backtest_fn` que devuelve un valor que no es
  `StrategyFitnessResult` → `error_code="EVALUATION_BACKTEST_INVALID_RETURN"`.

### `evaluate_strategies_for_opportunities(opps, ...)`
Misma semántica sobre un iterable. Robusto a `None`. El ranking se
aplica una sola vez sobre el total (no por opp).

### `rank_by_default_metric(items)`
Orden estable: éxitos primero (descendente por
`(sharpe, expectancy, win_rate)`), luego fallidos.

## Tests F14 — 12 casos
- `evaluate_strategies_for_opportunity`: ranking respetando sharpe,
  reject → vacío, None → vacío, backtest que lanza no propaga,
  backtest con retorno inválido detectado, `rank=False` preserva
  orden.
- `rank_by_default_metric`: fallidos al final, vacío, None.
- Batch: múltiples oportunidades + reject filtrada por F12, None.
- AST guard: ningún import prohibido.
- AST: el módulo importa F12 (`factory.dispatch`) y F13
  (`lean.runner.launcher`).

## Invariantes preservadas
- F3..F13 intactos. F14 no toca ningún módulo previo.
- Locks globales intactos.
- Inyección de dependencia (`backtest_fn`) permite tests sin LEAN
  real y futuras integraciones (paridad shadow vs real, replays
  históricos) sin tocar el adapter.

## Rollback

```bash
git -C /home/user/workspace/atlas-core revert <COMMIT_F14>
```

Sin impacto sobre runtime: el pipeline no se invoca aún desde
ningún módulo productivo. Lo consumirá F16.

## Próximos pasos
- **F15**: TradierAdapter institucional (paper / dry-run, sin live).
- **F16**: paper execution pipeline end-to-end (Radar → Strategy →
  LEAN fitness → Tradier paper).
