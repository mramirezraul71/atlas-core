# Atlas Code Quant — Telemetry (scaffold)

**Estado:** scaffold F1 — sin lógica.

Telemetría dedicada (logs, monitoring, journal). F1 sólo crea el árbol; operación actual se mantiene en `operations/` y endpoints health.

## Reglas F1
- No importar desde runtime productivo.
- No usar en `atlas_code_quant/api/main.py` ni en `operation_center`.
- Cualquier stub debe estar marcado `TODO` y nunca debe ser invocado en F1.
