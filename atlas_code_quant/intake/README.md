# Atlas Code Quant — Intake Layer (scaffold)

**Estado:** scaffold F1 — sin lógica.

Capa de ingesta canónica. Recibe candidatos desde Radar, universos dinámicos y feeds de mercado, y los normaliza para strategies/risk.

## Reglas F1
- No importar desde runtime productivo.
- No usar en `atlas_code_quant/api/main.py` ni en `operation_center`.
- Cualquier stub debe estar marcado `TODO` y nunca debe ser invocado en F1.
