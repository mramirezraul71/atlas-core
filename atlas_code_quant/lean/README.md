# Atlas Code Quant — LEAN (QuantConnect)

**Estado:** F13 adapter + parser + runner; **CLI opcional** (`lean backtest`) vía `lean/cli_backtest.py`.

- **Simulador GBM interno** (no es LEAN): `atlas_code_quant.backtest.internal_gbm_simulator` / `lean_simulator.py` (shim deprecated).
- **Fitness por intent**: `atlas_code_quant.lean.runner.launcher.run_backtest_for_strategy_intent`.
- **Orquestación con degradación**: `atlas_code_quant.backtest.unified_engine.run_unified_backtest_for_intent` (usado por F14 `strategies/evaluation.py`).

## Variables de entorno

Ver docstring en `atlas_code_quant/lean/config.py` y bloque en `config/atlas.env.example`.

## Desarrollo y tests

Antes de `pytest tests/atlas_code_quant/` completo:  
`pip install -r atlas_code_quant/requirements.txt -r atlas_code_quant/requirements-dev.txt`

## Reglas

- No ejecuta órdenes reales; no sustituye Tradier.
- Por defecto `ATLAS_LEAN_ENABLED=false`.
- `ATLAS_LEAN_CLI_PASS_INTENT` solo si un wrapper custom acepta `--intent-json`.
