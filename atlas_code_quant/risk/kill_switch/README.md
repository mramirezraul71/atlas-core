# risk.kill_switch (scaffold)

**Estado:** scaffold F1 — sin lógica.

Engine canónico de kill switch (manual / production_guard / risk / fsm). TODO F8: ampliar `operations/kill_switch.py` (~14 LOC) sin romper `/emergency/*`.

## Reglas F1
- No importar desde runtime productivo.
- No usar en `atlas_code_quant/api/main.py` ni en `operation_center`.
- Cualquier stub debe estar marcado `TODO` y nunca debe ser invocado en F1.
