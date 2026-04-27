# ATLAS F1 — Tradier canonicalization notes (no destructivo)

Fecha: 2026-04-26  
Rama: `feature/atlas-codequant-f1-safe-reorg`

## 1) Rutas Tradier encontradas

### Canon provisional (Code Quant)
- `atlas_code_quant/execution/tradier_execution.py`
- `atlas_code_quant/execution/tradier_controls.py`
- `atlas_code_quant/execution/tradier_pdt_ledger.py`

### Duplicado paralelo (atlas_options_brain_fase1)
- `atlas_options_brain_fase1/atlas_options_brain/broker/tradier_executor.py`
- `atlas_options_brain_fase1/atlas_options_brain/broker/tradier_live.py`
- `atlas_options_brain_fase1/atlas_options_brain/providers/tradier_provider.py`

## 2) Decisión F1 (conservadora)

- **Canon provisional para próximas fases:** `atlas_code_quant/execution/tradier_*`.
- **No se fusionó ni eliminó** código en `atlas_options_brain_fase1` en esta fase.
- La consolidación real queda para una fase posterior con pruebas de integración.

## 3) Qué no se tocó todavía

- No hubo merge destructivo de implementaciones Tradier.
- No se movieron módulos entre paquetes.
- No se habilitó live trading ni conexión a broker.

## 4) Riesgos para consolidación futura

1. Divergencia funcional entre dos implementaciones de ejecución.
2. Doble punto de verdad para manejo de errores, idempotencia y PDT.
3. Coste de mantenimiento si ambos caminos siguen activos en paralelo.

## 5) Siguiente paso sugerido (F2/F3)

- Definir contrato único `OrderIntent`/`OrderResult`.
- Elegir adaptador canónico y mapear llamadas antiguas mediante wrappers temporales.
- Añadir test matrix común antes de retirar el path duplicado.
