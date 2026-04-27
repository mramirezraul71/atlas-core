# risk.limits (scaffold)

**Estado:** scaffold F1 — sin lógica.

Límites duros (per-symbol, per-strategy, daily/weekly/monthly DD). TODO: derivar de `production_guard.py` y `operation_center` guards.

## Reglas F1
- No importar desde runtime productivo.
- No usar en `atlas_code_quant/api/main.py` ni en `operation_center`.
- Cualquier stub debe estar marcado `TODO` y nunca debe ser invocado en F1.
