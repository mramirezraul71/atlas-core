# Atlas Code Quant — LEAN integration (scaffold)

**Estado:** scaffold F1 — sin lógica.

Integración con LEAN/QuantConnect REAL externa o vendor. F1 NO contiene LEAN; F9 decidirá runner externo / vendor docker / abandono. El simulador GBM interno SIGUE en `backtest/lean_simulator.py` y NO es LEAN.

## Reglas F1
- No importar desde runtime productivo.
- No usar en `atlas_code_quant/api/main.py` ni en `operation_center`.
- Cualquier stub debe estar marcado `TODO` y nunca debe ser invocado en F1.
