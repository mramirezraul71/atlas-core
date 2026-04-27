# ATLAS_CODE_QUANT_F12 — Strategy Factory

## Política F12
- Scanner propone, **Radar decide** (gate enforced en F10), **Factory selecciona estructuras** (F12).
- Factory es **pura**: no I/O, no precios, no red, no excepciones.
- F12 NO toca `execution/`, `operations/`, `autonomy/`, `risk/`,
  `vision/`, `atlas_adapter/`, `broker_router`, `tradier_*`,
  `live_*`, ni los locks.

## Alcance
Toca SÓLO:
- `atlas_code_quant/strategies/factory/dispatch.py` (nuevo)
- `tests/atlas_code_quant/test_strategy_factory_f12.py` (nuevo, 23 tests)
- `docs/ATLAS_CODE_QUANT_F12_STRATEGY_FACTORY.md` (este doc)

## API pública

```python
from atlas_code_quant.strategies.factory.dispatch import (
    build_strategies_for_opportunity,
    build_strategies_for_opportunities,
)
```

### `build_strategies_for_opportunity(opp, *, min_score=70.0) -> list[StrategyIntent]`

Aplica:
1. Gate `opp is not None` y `RadarOpportunityInternal`.
2. Gate símbolo no vacío.
3. Gate `optionable=True`.
4. Gate `asset_class ∈ {equity, etf, index}`.
5. Gate `classification != "reject"` y `score >= min_score`
   (defensa-en-profundidad: F10 ya filtra, pero la factory revalida).
6. Despacha builders F11 según `direction`:
   - `long`/`short` → `vertical_spread` + `long_single`.
   - `neutral` → `iron_condor` + `iron_butterfly` (si DTE ≠ LONG)
     + `straddle` + `strangle` (si DTE ≠ SHORT).
   - cualquier otro → `[]`.
7. Filtro por `classification == "watchlist"` → conserva sólo
   `vertical_spread`, `iron_condor`, `iron_butterfly`
   (defined-risk). Vol-plays (`straddle`/`strangle`) y `long_single`
   se descartan.
8. Orden estable: `(strategy_type, num_legs)`.

### `build_strategies_for_opportunities(opps, *, min_score=70.0) -> list[StrategyIntent]`

Conveniencia para procesar un iterable. Robusto a `None`.

## Reglas de dispatch (resumen)

| direction | classification | DTE | strategies emitidas |
|---|---|---|---|
| long | high_conviction | * | vertical_spread, long_call |
| long | watchlist | * | vertical_spread |
| short | high_conviction | * | vertical_spread, long_put |
| short | watchlist | * | vertical_spread |
| neutral | high_conviction | SHORT | iron_butterfly, iron_condor, straddle |
| neutral | high_conviction | MEDIUM | iron_butterfly, iron_condor, straddle, strangle |
| neutral | high_conviction | LONG | iron_condor, straddle, strangle |
| neutral | watchlist | SHORT | iron_butterfly, iron_condor |
| neutral | watchlist | MEDIUM | iron_butterfly, iron_condor |
| neutral | watchlist | LONG | iron_condor |
| * | reject | * | ∅ |
| asset_class ∉ {equity,etf,index} | * | * | ∅ |
| optionable=False | * | * | ∅ |

## Tests F12 — 23 casos

- Gates: None, non-opp, símbolo vacío, reject, score bajo,
  threshold inclusive, no optionable, asset_class inválida (4
  paramétricos), direction unknown → normaliza a neutral y emite
  estructuras neutrales.
- Direccional: long ETF horizon corto → vertical + long_call con
  DTE_SHORT en todas las legs; short equity horizon medio → vertical
  bear-put + long_put.
- Neutral: index horizon largo → condor + straddle + strangle (no
  butterfly); horizon corto → condor + butterfly + straddle (no
  strangle).
- Filtro classification: watchlist neutral conserva sólo defined-risk;
  watchlist long deja vertical_spread pero NO long_call.
- Robustez: 4×3×6×3×3×2 = 432 oportunidades sintéticas, ningún caso
  lanza.
- Batch helper sobre lista mixta + None.
- AST guard: factory NO importa execution / autonomy / risk / vision /
  atlas_adapter / tradier / broker_router / live_*.
- Coherencia constantes: `DEFINED_RISK_TYPES ⊆ VALID_STRATEGY_TYPES`.

## Invariantes preservadas
- F3 deprecation headers / F6/F7/F8/F9/F10 / F11: intactos.
- Locks globales: intactos.
- Atlas Adapter Radar multi-símbolo: no tocado.

## Rollback

```bash
git -C /home/user/workspace/atlas-core revert <COMMIT_F12>
```

Sin impacto sobre runtime: F12 no es importado por ningún módulo
productivo todavía.

## Próximos pasos
- **F13**: LEAN adapter MVP (subprocess + parser → fitness).
- **F14**: pipeline `evaluate_strategies_for_opportunity(opp)` que
  combina F12 + F13 fitness y devuelve `StrategyWithFitness`.
- **F15**: Tradier adapter institucional (paper / dry-run).
- **F16**: paper execution pipeline end-to-end.
