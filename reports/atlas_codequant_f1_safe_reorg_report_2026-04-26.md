# ATLAS Code Quant — F1 Safe Reorg Report

Fecha: 2026-04-26  
Rama: `feature/atlas-codequant-f1-safe-reorg`  
Alcance: F1 (reorganización segura y preparación estructural)

## 1. Objetivo de F1

Preparar estructura futura (`intake`, `lean`, `autonomy`, `telemetry`, `strategies/options`, `vision gate`) sin romper comportamiento por defecto, sin cutover de scanner y sin integración live/Tradier/LEAN real.

## 2. Qué se creó

- Nuevos paquetes scaffold:
  - `atlas_code_quant/intake/*`
  - `atlas_code_quant/lean/*` + `lean/templates/*`
  - `atlas_code_quant/autonomy/*`
  - `atlas_code_quant/telemetry/*`
  - `atlas_code_quant/strategies/options/*`
  - `atlas_code_quant/vision/timing_gate.py`, `vision/imbalance.py`
  - `atlas_code_quant/legacy/__init__.py`
- Flags centralizados en:
  - `atlas_code_quant/config/feature_flags.py`

## 3. Qué se movió

- **No hubo movimientos físicos de carpetas legacy** en F1 (decisión conservadora).
- Se creó alias nuevo:
  - `atlas_code_quant/backtest/internal_gbm_simulator.py` (canónico F1)
- Se mantuvo implementación histórica en `backtest/lean_simulator.py` con docstring de deprecación/clarificación.

## 4. Qué quedó como legacy

- `atlas_code_quant/scanner/` permanece intacto y congelado para evitar roturas en F1.
- Documento de freeze:
  - `atlas_code_quant/legacy/README_F1_SCANNER_FREEZE.md`

## 5. Compat shims añadidos

- `backtest/internal_gbm_simulator.py` re-exporta el simulador actual para nueva ruta canónica.
- `backtest/lean_simulator.py` quedó explícitamente marcado como:
  - simulador interno
  - **NO** QuantConnect LEAN real
  - módulo en ruta deprecada por compatibilidad.

## 6. Riesgos que permanecen

1. Scanner sigue acoplado al flujo actual (diferido a F3).
2. Duplicidad Tradier (`atlas_code_quant` vs `atlas_options_brain_fase1`) no consolidada.
3. Integración LEAN aún es scaffold.

## 7. Qué queda explícitamente para F2/F3/F4

- F2: Radar multi-símbolo y contratos batch.
- F3: cutover scanner (shadow mode + retiro progresivo).
- F4: integración LEAN real (launcher operativo) y validación de estrategia.

## 8. Comandos ejecutados

```powershell
git checkout -b feature/atlas-codequant-f1-safe-reorg
python -m pytest tests/test_atlas_f1_imports.py tests/test_atlas_f1_backtest_shim.py tests/test_atlas_f1_strategy_factory.py tests/test_atlas_f1_vision_gate.py -q --tb=short
```

## 9. Resultado de tests/smoke

- Suite F1 scaffold:
  - **6 passed**, 0 failed
- Linter (IDE):
  - sin errores en paths tocados

## 10. Lista exacta de archivos tocados

Ver `git diff --name-status` en esta rama.  
Incluye scaffolds F1, tests F1 y reportes técnicos:
- `reports/atlas_f1_tradier_canonicalization_notes_2026-04-26.md`
- `reports/atlas_codequant_f1_safe_reorg_report_2026-04-26.md`
