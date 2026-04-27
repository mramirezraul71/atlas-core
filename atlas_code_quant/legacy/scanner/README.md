# legacy.scanner (RESERVED)

**Estado:** scaffold F1 — sin lógica.

Destino futuro del scanner heredado (`atlas_code_quant/scanner/`). F1 NO mueve código. Cutover real en F10. Ver `atlas_code_quant/legacy/README_SCANNER_FREEZE.md`.

## Reglas F1
- No importar desde runtime productivo.
- No usar en `atlas_code_quant/api/main.py` ni en `operation_center`.
- Cualquier stub debe estar marcado `TODO` y nunca debe ser invocado en F1.
