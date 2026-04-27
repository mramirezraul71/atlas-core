# autonomy.fsm (scaffold)

**Estado:** scaffold F1 — sin lógica.

State machine con estados (idle/scanning/evaluating/paper_executing/cooldown/blocked/emergency_stop). TODO F7: implementar respetando locks live.

## Reglas F1
- No importar desde runtime productivo.
- No usar en `atlas_code_quant/api/main.py` ni en `operation_center`.
- Cualquier stub debe estar marcado `TODO` y nunca debe ser invocado en F1.
