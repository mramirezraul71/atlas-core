# execution.brokers.tradier (scaffold)

**Estado:** scaffold F1 — sin lógica.

Sub-paquete Tradier institucional. TODO: re-host de `tradier_execution.py`, `tradier_controls.py`, `tradier_pdt_ledger.py` preservando `paper_only=True` y `full_live_globally_locked=True`.

## Reglas F1
- No importar desde runtime productivo.
- No usar en `atlas_code_quant/api/main.py` ni en `operation_center`.
- Cualquier stub debe estar marcado `TODO` y nunca debe ser invocado en F1.
