# F5 — Atlas Radar: Oportunidades Multi-Símbolo (atlas_adapter)

Estado: F5 completada (sin push). Cambios sólo en `atlas_adapter` y
`atlas_code_quant/config/legacy_flags.py` + tests + docs.

## 1. Motivación

Hasta F4, el Radar institucional expone **un único símbolo a la vez**
(`/api/radar/dashboard/summary?symbol=…`, `/dealer/{symbol}`, `/stream`).
F5 añade la **base multi-símbolo** que permite:

- Listar las oportunidades del universo curado en un único endpoint.
- Consultar la oportunidad para un símbolo concreto.
- Suscribirse a un stream SSE multi-símbolo con los eventos canónicos
  `universe_snapshot`, `opportunity_added`, `opportunity_updated`,
  `opportunity_removed`, `heartbeat`.

F5 NO conecta el `intake` real de `atlas_code_quant` ni cambia ninguna
ruta single-symbol existente. Reutiliza la pipeline ya probada
(`radar_build_dashboard_for_sse`) por símbolo, agregando ranking,
clasificación y trazabilidad por batch.

## 2. Reglas duras respetadas (F5)

- ❌ NO ejecuta trading. ❌ NO órdenes reales. ❌ NO broker live.
- ❌ NO toca locks `paper_only`, `full_live_globally_locked`, dry-run defaults.
- ❌ NO toca `atlas_code_quant/api/main.py` ni endpoints scanner heredados.
- ❌ NO crea `atlas_code_quant/intake/radar_client.py` funcional.
- ❌ NO modifica los endpoints Radar existentes
  (`summary`, `dashboard/summary`, `dealer/{symbol}`, `diagnostics/*`,
  `sensors/camera/health`, `camera/status`, `stream`,
  `decisions/recent`, `decisions/stats`, `symbols/search`).
- ✅ Cambios aditivos en `atlas_adapter`, contratos `extra="allow"`.
- ✅ Tolerante a fallo parcial por símbolo (degradación honesta).
- ✅ Flag `ATLAS_RADAR_MULTI_SYMBOL_ENABLED = False` doc-only.

## 3. Archivos creados / modificados

### Nuevos

| Archivo | Propósito |
| --- | --- |
| `atlas_adapter/services/universe_provider.py` | Universo curado determinista (ETFs líquidos + índices + equities optionable). Filtros `asset_class`, `sector`, `optionable`, `enabled_only`, `symbols`, `limit`. |
| `atlas_adapter/services/radar_batch_engine.py` | `RadarBatchEngine` que itera el universo, llama a `radar_build_dashboard_for_sse` por símbolo (inyectable en tests), tolera excepciones por-símbolo, scoring 0..100, ranking score-desc → símbolo-asc. |
| `tests/atlas_adapter/test_universe_provider_f5.py` | 11 tests: determinismo, filtros, limit, lookup case-insensitive. |
| `tests/atlas_adapter/test_radar_batch_engine_f5.py` | 9 tests: scoring/clasificación/dirección, fallo parcial, filtros, orden, source quant/stub. |
| `tests/atlas_adapter/test_radar_opportunities_contract_f5.py` | 10 tests HTTP `TestClient`: shape envelope, items, filtros, 404, headers, no-regresión single-symbol. |
| `tests/atlas_adapter/test_radar_opportunities_stream_f5.py` | 9 tests del envelope SSE multi-símbolo (validación Pydantic + headers helper) sin abrir stream infinito. |
| `docs/ATLAS_RADAR_F5_MULTI_SYMBOL_OPPORTUNITIES.md` | Este documento. |

### Modificados

| Archivo | Cambio |
| --- | --- |
| `atlas_adapter/routes/radar_schemas.py` | + `RadarOpportunitySnapshot`, `RadarOpportunity`, `RadarOpportunitiesResponse`, `RadarOpportunityStreamEvent` (todos `extra="allow"`). Modelos previos intactos. |
| `atlas_adapter/routes/radar_public.py` | + 3 handlers nuevos al final de `build_radar_stub_api_router()`. Handlers existentes intactos. |
| `atlas_code_quant/config/legacy_flags.py` | + `ATLAS_RADAR_MULTI_SYMBOL_ENABLED: bool = False` (doc-only). |

## 4. Contrato HTTP

### `GET /api/radar/opportunities`

Parámetros (query):

| Param | Tipo | Default | Descripción |
| --- | --- | --- | --- |
| `limit` | int | 50 | Máximo número de oportunidades devueltas tras filtrado y orden. `<= 0` ⇒ vacío. |
| `min_score` | float | 0.0 | Score mínimo (0..100) para incluir el item. |
| `asset_class` | str | — | `etf`, `index`, `equity`. Case-insensitive. |
| `sector` | str | — | Match exacto case-insensitive (p.ej. `technology`). |
| `optionable` | bool | — | Filtra por flag de optionable. |

Respuesta (`RadarOpportunitiesResponse`):

```json
{
  "ok": true,
  "source": "quant" | "stub",
  "timestamp": "2026-04-27T...",
  "universe_size": 39,
  "evaluated": 39,
  "succeeded": 39,
  "failed": 0,
  "returned": 5,
  "limit": 5,
  "min_score": 0.0,
  "filters": {"asset_class": null, "sector": null, "optionable": null, "symbols": null},
  "items": [ /* RadarOpportunity */ ],
  "degradations_active": [ /* RadarDegradationEntry batch-globales */ ],
  "trace_id": "abcdef012345"
}
```

### `GET /api/radar/opportunities/{symbol}`

Devuelve un único `RadarOpportunity`. `404 symbol_not_in_universe` si el
símbolo no está en el universo curado.

### `RadarOpportunity` (campos canónicos)

```json
{
  "symbol": "SPY",
  "asset_class": "etf",
  "sector": "broad_market",
  "optionable": true,
  "score": 0.0,
  "classification": "high_conviction" | "watchlist" | "reject",
  "direction": "long" | "short" | "neutral",
  "timestamp": "2026-04-27T...",
  "horizon_min": null,
  "snapshot": { /* RadarOpportunitySnapshot */ },
  "degradations_active": [ /* per-symbol */ ],
  "source": "quant" | "stub",
  "trace_id": "<batch>:SPY"
}
```

Umbrales: `score >= 70` ⇒ `high_conviction`; `>= 40` ⇒ `watchlist`; resto
`reject`. Score = `0.6·fast_pressure + 0.4·structural_confidence` (ambos
normalizados 0..1 por `build_dashboard_summary`), escalado ×100.

`direction` se deriva de `radar.signal.bias` con la misma normalización
canónica usada por el mapper (`long/largo/alcista/buy` → `long`, etc).

## 5. Contrato SSE — `GET /api/radar/stream/opportunities`

Mismo envelope canónico de 6 campos que el resto del Radar:

```
{type, timestamp, symbol, source, sequence, data}
```

Tipos válidos: `heartbeat`, `universe_snapshot`, `opportunity_added`,
`opportunity_updated`, `opportunity_removed`. Para `heartbeat` y
`universe_snapshot` el campo `symbol` es `"*"`. Intervalos heredados de
`ATLAS_RADAR_SSE_HEARTBEAT_SEC` / `ATLAS_RADAR_SSE_SNAPSHOT_SEC`.

## 6. Universo inicial (curado, determinista)

ETFs líquidos: `SPY, QQQ, IWM, DIA, XLK, XLF, XLE, XLV, XLI, XLY, XLP,
XLU, XLB, XLRE, XLC`.

Índices (no optionable): `SPX, NDX, RUT, VIX`.

Equities optionables (mega-cap): `AAPL, MSFT, NVDA, AMZN, GOOGL, META,
TSLA, AMD, AVGO, NFLX, JPM, BAC, XOM, CVX, UNH, LLY, WMT, COST, BA, CAT`.

Total ≈ 39 símbolos. Sin red, sin DB; modificable extendiendo
`UniverseProvider(entries=…)` (inyección directa, útil en tests).

## 7. Degradación honesta

- Excepción al construir summary de un símbolo ⇒ batch sigue, símbolo se
  excluye de `items`, `failed += 1`, se añade entrada
  `OPPORTUNITY_BUILD_FAILED` en `degradations_active`.
- Si NINGÚN símbolo logra `live=True`, se añade `QUANT_UNREACHABLE_BATCH`
  (warning) y los items llevan `source = "stub"`.
- `degradations_active` por símbolo se preserva tal cual lo emite la
  pipeline existente.

## 8. Compatibilidad y no regresión

- Todas las rutas previas siguen registradas y respondiendo (verificado
  por `test_existing_endpoints_unchanged`).
- Modelos Pydantic previos intactos. Los nuevos modelos llevan
  `extra="allow"`.
- Tests previos (`tests/test_radar_sse_contract.py`,
  `tests/test_radar_symbols_search_http.py`) siguen pasando.
- La flag `ATLAS_RADAR_MULTI_SYMBOL_ENABLED` NO se consulta en runtime;
  es contrato público para fases siguientes que conecten Code Quant
  intake.

## 9. Criterio de aceptación (F5)

- ✅ `python -c "import atlas_adapter.routes.radar_public; import atlas_adapter.services.universe_provider; import atlas_adapter.services.radar_batch_engine"` sin error.
- ✅ `pytest --collect-only` sin regresión (mismos 40 errores
  preexistentes ajenos al Radar).
- ✅ 39 tests F5 nuevos verdes (universo + batch + contrato HTTP +
  envelope SSE).
- ✅ Endpoints existentes intactos.

## 10. Rollback

`git revert <commit-F5>` revierte:

- 2 archivos nuevos en `atlas_adapter/services/`.
- 4 archivos nuevos en `tests/atlas_adapter/`.
- 1 doc nuevo en `docs/`.
- Bloque añadido a `radar_schemas.py` y a `radar_public.py`
  (`build_radar_stub_api_router()` final).
- 1 flag doc-only en `legacy_flags.py`.

No hay datos persistidos ni migraciones.

## 11. Próximos pasos (NO en F5)

- Conectar `atlas_code_quant/intake/` con un cliente HTTP real al motor
  Quant que alimente el batch engine sin pasar por la pipeline single-symbol.
- Activar `ATLAS_RADAR_MULTI_SYMBOL_ENABLED` como condicional sólo cuando
  exista paridad funcional con la ruta single-symbol.
- Diff incremental real en SSE (cache externa) para no recalcular el
  universo entero en cada ciclo de snapshot.
