# ATLAS_CODE_QUANT_F13 — LEAN adapter MVP

## Política F13
- Adapter LEAN **externo** (subprocess/CLI). No es ejecutor de
  órdenes: produce **fitness** (Sharpe, win rate, max drawdown,
  total return, expectancy, num_orders) sobre `StrategyIntent` (F11).
- F13 NO toca Tradier, NO toca `execution/`, NO toca live, NO toca
  locks (`paper_only`, `full_live_globally_locked`).
- Default OFF: `ATLAS_LEAN_ENABLED=False`.

## Alcance
Toca SÓLO:
- `atlas_code_quant/lean/config.py` (nuevo)
- `atlas_code_quant/lean/parser/results.py` (nuevo)
- `atlas_code_quant/lean/runner/launcher.py` (nuevo)
- `tests/atlas_code_quant/test_lean_adapter_f13.py` (nuevo, 25 tests)
- `docs/ATLAS_CODE_QUANT_F13_LEAN_ADAPTER_MVP.md` (este doc)

## Configuración

`LeanAdapterConfig` se construye con `load_config_from_env()` y lee:

| Env var | Default | Descripción |
|---|---|---|
| `ATLAS_LEAN_ENABLED` | `false` | activa el adapter |
| `ATLAS_LEAN_MODE` | auto | `"mock"` o `"external"` |
| `ATLAS_LEAN_BIN` | — | path a binario LEAN (subprocess) |
| `ATLAS_LEAN_RESULTS_DIR` | — | dir donde el subprocess escribe `statistics.json` / `orders.json` |
| `ATLAS_LEAN_TIMEOUT_SEC` | `60` | timeout duro del subprocess |

Reglas de auto-modo:
- Si `ATLAS_LEAN_ENABLED=true` y hay `ATLAS_LEAN_BIN` → `external`.
- Si `enabled=true` y NO hay binario → `mock` (degradación honesta).
- Si `enabled=false` → modo `disabled` independientemente de `MODE`.

## Parser

`atlas_code_quant.lean.parser.results`:
- `parse_statistics_payload(payload)` acepta dicts, dicts anidados
  bajo `"Statistics"`, strings JSON. Reconoce variantes habituales
  de LEAN (`"Sharpe Ratio"`, `"SharpeRatio"`, `"sharpe"`, etc.) y
  porcentajes con sufijo `%`. Devuelve `(metrics, warnings)`.
- `parse_orders_payload(payload)` acepta lista directa, dict con
  `"orders"` o `"Orders"`, string JSON.
- `parse_run_artifacts(*, statistics, orders)` produce
  `LeanRunArtifacts` (frozen dataclass).

## Runner

`atlas_code_quant.lean.runner.launcher.run_backtest_for_strategy_intent`:

```python
def run_backtest_for_strategy_intent(
    intent: StrategyIntent | None,
    *,
    config: LeanAdapterConfig | None = None,
) -> StrategyFitnessResult: ...
```

Comportamiento:

| Caso | resultado |
|---|---|
| `intent` no es `StrategyIntent` o sin legs | `success=False`, `error_code=LEAN_INVALID_INTENT` |
| Flag OFF | `mode="disabled"`, `error_code=LEAN_DISABLED` |
| Modo `mock` | `success=True`, métricas deterministas derivadas de hash del intent (sharpe ∈ [-1,3], win_rate ∈ [0,100], max_dd ∈ [-30,0], total_return ∈ [-20,40], expectancy ∈ [-0.5,1.5]) |
| Modo `external` happy path | subprocess + lectura de `statistics.json` y `orders.json` + parser → `StrategyFitnessResult` |
| Subprocess timeout | `error_code=LEAN_TIMEOUT` |
| Subprocess non-zero exit | `error_code=LEAN_PROCESS_FAILED` |
| Binario no encontrado | `error_code=LEAN_PROCESS_FAILED` |
| `external` sin `bin_path` | `error_code=LEAN_PROCESS_FAILED` (sin subprocess) |
| Excepción al parsear artefactos | `error_code=LEAN_PARSE_FAILED` |

## Tests F13 — 25 casos

- Config: defaults OFF; enabled+bin → external; enabled sin bin →
  mock; timeout inválido → fallback 60s.
- Parser:
  - statistics formato LEAN nested + flat + vacío + string JSON.
  - orders dict con clave `"orders"`, lista directa, payload no-list,
    JSON inválido.
  - `parse_run_artifacts` combina statistics + orders.
- Runner:
  - flag OFF → `LEAN_DISABLED`.
  - intent None / garbage → `LEAN_INVALID_INTENT`.
  - mock determinista (mismo intent ⇒ mismas métricas, rangos
    plausibles).
  - external happy path con `subprocess.run` mockeado +
    `statistics.json` / `orders.json` reales en `tmp_path`.
  - external timeout (TimeoutExpired) → `LEAN_TIMEOUT`.
  - external non-zero exit → `LEAN_PROCESS_FAILED`.
  - binary missing (FileNotFoundError) → `LEAN_PROCESS_FAILED`.
  - external sin `bin_path` → no se llama a `subprocess.run`,
    fallo honesto.
- AST guard: `lean/config.py`, `lean/parser/results.py`,
  `lean/runner/launcher.py` NO importan execution / autonomy /
  risk / vision / atlas_adapter / tradier / broker_router /
  live_loop / live_activation.

## Invariantes preservadas
- F3..F12 intactos: F13 no toca scanner, Radar gate, factory, ni
  el modelo `StrategyIntent`.
- Locks globales: intactos.
- Subprocess solo si flag está ON y modo `external`. En tests no
  se ejecuta subprocess real (se mockea `subprocess.run`).

## Rollback

```bash
git -C /home/user/workspace/atlas-core revert <COMMIT_F13>
```

Sin impacto sobre runtime: el adapter no se invoca desde ningún
módulo runtime hasta F14.

## Próximos pasos
- **F14**: orquestador `evaluate_strategies_for_opportunity` que
  combina F12 (Strategy Factory) + F13 (LEAN fitness).
- **F15**: TradierAdapter institucional (paper / dry-run).
- **F16**: paper execution pipeline end-to-end.
