# Atlas Code Quant — Autonomy (scaffold)

**Estado:** scaffold F1 — sin lógica.

FSM formal con estados nombrados, policy engine y gates. F1 sólo crea el árbol; el comportamiento autónomo sigue en `operation_center.py` y `atlas_core/autonomy/orchestrator.py`.

## Reglas F1
- No importar desde runtime productivo.
- No usar en `atlas_code_quant/api/main.py` ni en `operation_center`.
- Cualquier stub debe estar marcado `TODO` y nunca debe ser invocado en F1.
