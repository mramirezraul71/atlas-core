# Scanner — FREEZE (F1)

**Estado:** congelado lógicamente desde F1 · cutover físico planificado para F10.

## Resumen

El componente `atlas_code_quant/scanner/` queda **congelado a partir de F1**:

- En F1 **no se mueve** ningún archivo del scanner.
- En F1 **no se borra** ningún archivo del scanner.
- En F1 **no se cambia** la semántica de los endpoints `/scanner/*` ni `/api/v2/quant/scanner/*` en `atlas_code_quant/api/main.py`.
- En F1 **no se modifica** la importación pública `from atlas_code_quant.scanner import OpportunityScannerService`.

A partir de F1, el scanner se considera **legacy** a efectos de roadmap. Cualquier nueva funcionalidad de descubrimiento de oportunidades debe construirse en la **arquitectura canónica** (Radar institucional + `atlas_code_quant/intake/` + `atlas_adapter/services/radar_batch_engine.py` cuando exista en F5).

## Razones de la congelación

Ya documentadas en `atlas_code_quant/SCANNER_CRITERIA_AUDIT.md` y en
`docs/ATLAS_CODE_QUANT_REORG_VERIFICATION.md` (F0):

- Criterios débiles ya auto-auditados.
- Duplicación de endpoints `/scanner/*` y `/api/v2/quant/scanner/*` (`api/main.py:3356-3445` y `:3677-3706`).
- Duplicación funcional con el Radar institucional (`atlas_adapter/routes/radar_public.py`, mapper, schemas, cliente).
- Tamaño y complejidad: `opportunity_scanner.py` 2218 LOC + módulos auxiliares ~3637 LOC.

## Componentes afectados (sin tocar en F1)

- `atlas_code_quant/scanner/opportunity_scanner.py` (2218 LOC)
- `atlas_code_quant/scanner/asset_classifier.py`
- `atlas_code_quant/scanner/etf_universe.py`
- `atlas_code_quant/scanner/index_universe.py`
- `atlas_code_quant/scanner/crypto_universe.py`
- `atlas_code_quant/scanner/futures_universe.py`
- `atlas_code_quant/scanner/options_flow_provider.py`
- `atlas_code_quant/scanner/universe_catalog.py`
- `scanner/__init__.py` raíz (compat shim que reexporta `atlas_code_quant.scanner`)

## Consumidores conocidos (no romper)

- `atlas_code_quant/api/main.py` (endpoints `/scanner/*` y `/api/v2/quant/scanner/*`)
- `atlas_code_quant/backtester.py:27`
- `atlas_code_quant/learning/trading_implementation_scorecard.py:568`
- Tests en `atlas_code_quant/tests/test_scanner*.py`

## Flag de configuración (informativo, sin efectos en F1)

Se introduce en F1 una constante de marcado:

```python
# atlas_code_quant/config/legacy_flags.py
SCANNER_IS_LEGACY: bool = True  # Etiqueta sólo informativa. No altera ejecución.
```

Esta constante NO se consulta en runtime productivo en F1. Su única finalidad es:

1. Servir de marcador documental para herramientas de auditoría.
2. Anclar el contrato "scanner congelado" para fases posteriores.
3. Permitir que F3/F10 lean el flag sin tener que reintroducirlo.

## Roadmap del freeze

| Fase | Acción |
|---|---|
| F1 (esta) | Documentar freeze · crear `legacy/scanner/` vacío · flag `SCANNER_IS_LEGACY=True` informativo. |
| F3 | Añadir headers `Deprecation` y `Sunset` a endpoints duplicados scanner. Sin borrar handlers. |
| F5 | Construir `/api/radar/opportunities` multi-símbolo + batch engine (alternativa al scanner). |
| F10 | Cutover real: migrar consumidores, mover `atlas_code_quant/scanner/*` a `atlas_code_quant/legacy/scanner/*` con compat shim. Endpoints scanner → 410 Gone tras Sunset. |

## Reglas duras durante la congelación

- No tocar lógica de `opportunity_scanner.py`.
- No alterar contratos JSON de `/scanner/*`.
- No remover el shim raíz `scanner/__init__.py` hasta F10.
- Cualquier defecto detectado en scanner → fix mínimo + prueba + nota en `SCANNER_CRITERIA_AUDIT.md`. NO refactor profundo.
