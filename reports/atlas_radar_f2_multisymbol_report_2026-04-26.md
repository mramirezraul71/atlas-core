# Informe F2 — Radar multi-símbolo (oportunidades optionables)

**Fecha:** 2026-04-26  
**Rama:** `feature/atlas-radar-f2-multisymbol`  
**Alcance:** PUSH / `atlas_adapter` + flags en `atlas_code_quant/config` — sin cutover del scanner en Code Quant.

## 1. Objetivo F2

Convertir el Radar institucional en **proveedor multi-símbolo** de oportunidades optionables, con contratos REST/SSE nuevos, **sin romper** el Radar mono-símbolo (`/api/radar/stream?symbol=`, summary, etc.) y **sin** consumo desde `atlas_code_quant/api/main.py`.

## 2. Diseño del universe provider

- **Archivo:** `atlas_adapter/services/universe_provider.py`
- **Clase:** `UniverseProvider` con `refresh()`, `get_optionable_universe(max_size)`, `search(query, limit)`.
- **Tipado:** `UniverseEntry` (symbol, asset_class, sector, optionable, notes).
- **Fuentes:** intento de import de `atlas_code_quant.scanner.etf_universe` e `index_universe`; si falla el import, **fallback curado local** (ETFs + índices + subset de equities líquidas), documentado en código.
- **TTL:** `ATLAS_RADAR_UNIVERSE_REFRESH_SEC` (default 300) para refresco en memoria.
- **Batch:** el tamaño máximo del lote se alinea con `ATLAS_RADAR_MAX_SYMBOLS_PER_BATCH` vía llamada a `get_optionable_universe`.

## 3. Diseño del batch engine

- **Archivo:** `atlas_adapter/services/radar_batch_engine.py`
- **Enfoque:** por cada símbolo del universo filtrado, llama a `radar_build_dashboard_for_sse(symbol)` (misma lógica stub/live que el Radar actual).
- **Scoring (0–100):** combinación conservadora de `fast_pressure_score` y `structural_confidence_score` (0–1) más bono pequeño por `snapshot_classification`; documentado en el módulo.
- **Salida:** diccionarios compatibles con `RadarOpportunity` (symbol, asset_class, score, classification, timestamp, horizon_min, direction, snapshot, degradations_active, source, trace_id).
- **Ranking:** orden descendente por `score`, truncado por `limit`.
- **Degradaciones:** merge global por `code` en `global_degradations`.
- **Horizon:** derivado del candidato Quant si existe; si no, heurística por timeframe o default (45–60 min).

## 4. Contrato de oportunidades

- **Modelos:** `RadarOpportunity`, `RadarOpportunitiesResponse`, `RadarOpportunityDetailResponse`, y modelos de referencia SSE (`RadarSseUniverseSnapshotData`, `RadarSseOpportunityEventData`, `RadarSseHeartbeatData`) en `atlas_adapter/routes/radar_schemas.py`.
- **SSE:** mismo patrón que el Radar existente: envelope con **6 campos** `type`, `timestamp`, `symbol`, `source`, `sequence`, `data` (`RadarSseEnvelope`).
- **Tipos F2 en stream:** `opportunity_added`, `universe_snapshot`, `heartbeat` (extensible a updated/removed en F3).

## 5. Endpoints añadidos

Montados vía `build_radar_opportunities_router()` en `atlas_http_api.py`:

| Método | Ruta | Notas |
|--------|------|--------|
| GET | `/api/radar/opportunities` | Query: `limit`, `min_score`, `asset_class`, `sector`, `source`/`mode` (informativos) |
| GET | `/api/radar/opportunities/{symbol}` | Detalle; 404 si no está en universo o bajo `min_score` |
| GET | `/api/radar/stream/opportunities` | SSE multi-símbolo |

Si `ATLAS_RADAR_MULTI_SYMBOL_ENABLED` es falso: **503** con JSON claro y cabeceras `no-store` / `Pragma: no-cache` donde aplica.

## 6. Feature flags

Centralizados en `AtlasFeatureFlags` (`atlas_code_quant/config/feature_flags.py`), leídos en **cada instancia** (`default_factory`) para pruebas y coherencia runtime:

- `ATLAS_RADAR_MULTI_SYMBOL_ENABLED` (default false)
- `ATLAS_RADAR_MAX_SYMBOLS_PER_BATCH` (default 100, acotado 1–5000)
- `ATLAS_RADAR_MIN_SCORE` (default 80, 0–100)
- `ATLAS_RADAR_UNIVERSE_REFRESH_SEC` (default 300, 15–86400)

## 7. Preservado del Radar previo

- Sin cambios al contrato del stream **mono-símbolo** `/api/radar/stream`.
- Endpoints `/scanner/*` y motor Code Quant **no** retirados.
- `atlas_code_quant/api/main.py` **no** modificado para consumir Radar.
- Summary, diagnostics, decisions, etc. **inalterados** en comportamiento base.

## 8. Mejora opcional: búsqueda de símbolos

- Si Quant no responde **y** el flag multi está activo, `GET /api/radar/symbols/search` puede devolver matches desde el universo curado (`source=universe_curated`).
- Con flag apagado, comportamiento **idéntico** al anterior (stub `unavailable`).

## 9. No hecho aún (F3+)

- Cutover del scanner / consumo Radar desde Code Quant API.
- LEAN, Tradier, deltas SSE `opportunity_updated` / `opportunity_removed` con estado persistente.
- Sector como dimensión rica si el catálogo no la tiene; hoy filtra por `UniverseEntry.sector` cuando existe.

## 10. Riesgos y limitaciones

- **Coste CPU/latencia:** un batch grande multiplica llamadas a `radar_build_dashboard_for_sse` (y posiblemente al reporte Quant).
- **Universo:** catálogo estático / curado; no sustituye datos de mercado en vivo.
- **Scoring:** heurístico; no es recomendación de inversión.
- **Scanner:** la reutilización de módulos `scanner/*` es **transitoria** (catálogos); el Radar no depende del scanner como proceso.

## 11. Resultados de tests

Ejemplo de ejecución local:

```text
pytest tests/atlas_adapter/ tests/test_radar_batch_engine.py tests/test_radar_sse_contract.py tests/test_radar_symbols_search_http.py tests/test_radar_quant_contract.py -q
59 passed
```

Incluye tests nuevos:

- `tests/atlas_adapter/test_universe_provider.py`
- `tests/atlas_adapter/test_radar_opportunities_contract.py`
- `tests/test_radar_batch_engine.py`

## 12. Archivos tocados (lista)

**Creados**

- `atlas_adapter/services/universe_provider.py`
- `atlas_adapter/services/radar_batch_engine.py`
- `atlas_adapter/routes/radar_opportunities.py`
- `tests/atlas_adapter/test_universe_provider.py`
- `tests/atlas_adapter/test_radar_opportunities_contract.py`
- `tests/test_radar_batch_engine.py`
- `reports/atlas_radar_f2_multisymbol_report_2026-04-26.md`

**Modificados**

- `atlas_code_quant/config/feature_flags.py`
- `atlas_adapter/routes/radar_schemas.py`
- `atlas_adapter/routes/radar_public.py`
- `atlas_adapter/atlas_http_api.py`
