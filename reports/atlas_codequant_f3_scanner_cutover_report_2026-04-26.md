# F3 Report — Scanner Cutover to Radar Intake

## 1. Objetivo F3

Cortar el scanner legacy como productor activo de candidatos y mover Code Quant a consumo de oportunidades Radar multi-símbolo (F2), manteniendo paper-first, fallback seguro y sin tocar LEAN/Tradier live.

## 2. Dependencias del scanner retiradas

- En `atlas_code_quant/api/main.py` se retiró el import operativo:
  - `from scanner.opportunity_scanner import OpportunityScannerService`
  - `from scanner.universe_catalog import ScannerUniverseCatalog`
  - `from scanner.asset_classifier import classify_asset`
- Se eliminó el uso de `_UNIVERSE_CATALOG`.
- `_SCANNER` deja de ser instancia legacy y pasa a `RadarScannerAdapter`, que consume Radar intake.
- La ruta activa de oportunidades en `_auto_cycle_loop` mantiene el contrato `report()/candidates`, pero la fuente interna ahora es Radar.

## 3. Nuevo intake Radar

Implementado paquete `atlas_code_quant/intake/`:

- `opportunity.py`
  - `RadarOpportunity`
  - `RadarOpportunityBatch`
  - parseo tipado desde payload JSON de `/api/radar/opportunities`.
- `radar_client.py`
  - `RadarOpportunityClient.fetch_opportunities(...)`
  - `RadarOpportunityClient.get_opportunity(symbol, ...)`
  - manejo degradado explícito para `503`, timeouts y errores de parseo.
  - propagación de `trace_id`.
  - logging estructurado (`quant.intake.radar`).
- `__init__.py` exporta contratos y resultado del cliente.

## 4. Endpoints legacy scanner desactivados

Con `ATLAS_LEGACY_SCANNER_ENABLED=false` (default F3):

- `GET /scanner/status`
- `GET /scanner/report`
- `POST /scanner/config`
- `POST /scanner/control`
- `GET /scanner/universe/search`
- `GET /api/v2/quant/scanner/status`
- `GET /api/v2/quant/scanner/report`
- `POST /api/v2/quant/scanner/config`
- `POST /api/v2/quant/scanner/control`
- `GET /api/v2/quant/scanner/universe/search`

responden `410 Gone` con payload explícito:

- `ok=false`
- `deprecated=true`
- `message="Legacy scanner disabled; use /api/radar/opportunities"`

## 5. Qué quedó aún como legacy

- Nombres de etapas/metadatos internos (`scanner_report`) se conservan para compatibilidad de telemetría y contratos existentes.
- Esquemas `Scanner*` de `api.schemas` se mantienen para no romper importaciones ni OpenAPI histórica.
- Flags/settings `scanner_*` siguen presentes pero ya no son la fuente activa de oportunidades.

## 6. Cómo hacer rollback

1. Activar `ATLAS_LEGACY_SCANNER_ENABLED=1`.
2. Revertir commit F3 del cutover o restaurar rama previa F2.
3. Verificar endpoints `/scanner/*` y loop operativo.

No se recomienda rollback parcial (mezclar adapter Radar con scanner legacy) sin pruebas de humo.

## 7. Riesgos / limitaciones

- `RadarScannerAdapter` mantiene forma legacy, pero no replica toda semántica histórica del scanner.
- Polling REST (F3) puede introducir latencia vs stream SSE continuo.
- Dependencia de disponibilidad del endpoint Radar: cuando falla, se degrada con actividad y estado auditable.

## 8. Tests ejecutados

Comando:

```text
python -m pytest atlas_code_quant/intake/tests/test_radar_client_contract.py tests/atlas_code_quant/test_scanner_legacy_endpoints_disabled.py tests/atlas_code_quant/test_radar_intake_integration.py -q
```

Resultado:

- `5 passed`

## 9. Archivos tocados (exactos)

### Creados

- `atlas_code_quant/intake/tests/test_radar_client_contract.py`
- `atlas_code_quant/legacy/compare_scanner_vs_radar.py`
- `tests/atlas_code_quant/test_scanner_legacy_endpoints_disabled.py`
- `tests/atlas_code_quant/test_radar_intake_integration.py`
- `reports/atlas_codequant_f3_scanner_cutover_report_2026-04-26.md`

### Modificados

- `atlas_code_quant/config/feature_flags.py`
- `atlas_code_quant/intake/__init__.py`
- `atlas_code_quant/intake/opportunity.py`
- `atlas_code_quant/intake/radar_client.py`
- `atlas_code_quant/api/main.py`
