# vision.timing_gate (scaffold)

**Estado:** scaffold F1 — sin lógica.

Vision Timing Gate institucional con enum allow|delay|block|force_exit y trace_id. TODO F6: reemplazar boolean `allow_entry` en `options_intent_router.py` con compat.

## Reglas F1
- No importar desde runtime productivo.
- No usar en `atlas_code_quant/api/main.py` ni en `operation_center`.
- Cualquier stub debe estar marcado `TODO` y nunca debe ser invocado en F1.
