# Institutional Radar Runtime Guide

## Activar router radar en host API

- Variable: `ATLAS_ENABLE_RADAR_ROUTER`
- Valores:
  - `true` / `1` / `yes` -> monta rutas `/api/radar/*` en `atlas_adapter.atlas_http_api`
  - por defecto `false` -> no monta router (modo no invasivo)

## Selección de provider de flow

- Variable principal: `ATLAS_FLOW_PROVIDER`
  - `synthetic`
  - `noop`
  - `unusual_whales`
- Fallback: `ATLAS_FLOW_PROVIDER_FALLBACK` (`synthetic` por defecto)

### Variables Unusual Whales

- `ATLAS_UNUSUAL_WHALES_API_KEY` (requerida para provider real)
- `ATLAS_UNUSUAL_WHALES_BASE_URL` (default `https://api.unusualwhales.com`)
- `ATLAS_UNUSUAL_WHALES_TIMEOUT_SEC` (default `8.0`)
- `ATLAS_UNUSUAL_WHALES_MAX_EVENTS` (default `120`, max API `200`)
- `ATLAS_UNUSUAL_WHALES_MIN_PREMIUM` (default `10000`)
- `ATLAS_UNUSUAL_WHALES_CLIENT_API_ID` (opcional)

Si no hay API key o el endpoint falla, el radar degrada y usa fallback configurado.

## Handoff HTTP opcional

- `ATLAS_RADAR_HANDOFF_HTTP_ENABLED=true`
- `ATLAS_RADAR_HANDOFF_HTTP_URL=https://...`
- `ATLAS_RADAR_HANDOFF_HTTP_TIMEOUT_SEC=5.0`
- `ATLAS_RADAR_HANDOFF_HTTP_RETRIES=2`
- `ATLAS_RADAR_HANDOFF_HTTP_BACKOFF_SEC=0.5`
- `ATLAS_RADAR_HANDOFF_HTTP_BEARER_TOKEN=...` (opcional)

El publisher HTTP **solo** publica `RadarDecisionHandoff`; no ejecuta órdenes.

## Snapshot store JSONL

- Path: `ATLAS_SCANNER_RADAR_SNAPSHOT_PATH` (default `data/atlas_scanner/radar_snapshots.jsonl`)
- Rotación:
  - `ATLAS_SCANNER_RADAR_STORE_MAX_RECORDS` (default `5000`)
  - `ATLAS_SCANNER_RADAR_STORE_MAX_BYTES` (default `8000000`)

## Endpoints operativos

- `GET /api/radar/health/providers`
- `GET /api/radar/snapshot/{symbol}`
- `GET /api/radar/macro/calendar/{symbol}`
- `GET /api/radar/recent`
- `GET /api/radar/replay/{symbol}`
- `GET /api/radar/store/stats`
- `GET /api/radar/diagnostics/providers`
- `GET /api/radar/diagnostics/structural/{symbol}`
- `GET /api/radar/diagnostics/fast/{symbol}`
- `GET /api/radar/dealer/{symbol}`
- `GET /api/radar/political/{symbol}`
- `GET /api/radar/decisions/recent`
- `GET /api/radar/decisions/{symbol}`
- `GET /api/radar/decisions/replay/{symbol}`
- `GET /api/radar/decisions/stats`
- `GET /api/radar/dashboard/summary`

## Composites operativos (trazables)

En `signal.meta`:
- `structural_confidence_score`
- `structural_bullish_score`
- `structural_bearish_score`
- `fast_pressure_score`
- `fast_directional_bias`
- `fast_risk_score`
- `fast_structural_alignment`
- `fast_structural_divergence_score`
- `horizon_conflict`
- `cross_horizon_alignment`

Regla operativa:
- Structural = insider + regulatory + ownership + political (sensible a lag/freshness).
- Fast = market + flow + dealer + macro inmediato.
- No se mezcla dealer/flow intradía como trigger estructural.

## Clasificación de snapshot

Cada señal incluye `snapshot_classification`:
- `fully_operable`
- `operable_with_degradation`
- `structural_only`
- `fast_only`
- `non_operable`

Usar esta clasificación para paper supervised ops y gates del decision engine.

## Decision Gate (paper supervised)

El gate conecta el radar con la capa de decisión **sin ejecutar órdenes**.

Variables:
- `ATLAS_DECISION_GATE_ENABLED` (`true|false`)
- `ATLAS_DECISION_GATE_MODE` (usar `paper_supervised`)
- `ATLAS_DECISION_GATE_MIN_STRUCTURAL_CONFIDENCE`
- `ATLAS_DECISION_GATE_MIN_FAST_PRESSURE`
- `ATLAS_DECISION_GATE_MAX_DIVERGENCE_SCORE`
- `ATLAS_DECISION_GATE_ALLOW_DEGRADED`
- `ATLAS_DECISION_GATE_ALLOW_STRUCTURAL_ONLY`
- `ATLAS_DECISION_GATE_ALLOW_FAST_ONLY`

Reglas operativas:
- `fully_operable`: aceptable si cumple umbrales.
- `operable_with_degradation`: caution o reject según flags.
- `structural_only` / `fast_only`: controlado por flags.
- `non_operable`: reject.

En `paper_supervised`:
- se evalúa gate por snapshot/handoff,
- se registra decisión con trazabilidad completa,
- se publica a cola simulada interna (`paper_supervised_queue`),
- no se ejecutan órdenes ni se toca broker.

## Decision Store persistente

El gate persiste evaluaciones en JSONL (auditoría/replay):
- `ATLAS_DECISION_GATE_STORE_PATH` (default `data/atlas_scanner/decision_gate.jsonl`)
- `ATLAS_DECISION_GATE_STORE_MAX_RECORDS` (default `12000`)
- `ATLAS_DECISION_GATE_STORE_MAX_BYTES` (default `8000000`)

Replay/consulta:
- `/api/radar/decisions/recent`
- `/api/radar/decisions/{symbol}`
- `/api/radar/decisions/replay/{symbol}`
- `/api/radar/decisions/stats`

`/api/radar/decisions/stats` expone:
- conteo total y por decisión (`accepted/rejected/caution/bypassed`)
- breakdown de razones de rechazo
- breakdown de clasificación
- promedios de composites (`structural_confidence`, `fast_pressure`)
- métricas de archivo (`file_size_bytes`, `record_count`)

## Dashboard summary payload

`GET /api/radar/dashboard/summary` consolida en un call:
- `provider_health_summary`
- `snapshot_classification_breakdown`
- `decision_gate_stats`
- `recent_signals`
- `active_alerts_or_degradations`
- `freshness_status_by_domain`
- `circuit_breaker_status`
- `recent_decisions`
- `recent_snapshots`

## Web dashboard institucional (operaciones paper supervised)

Dashboard web independiente del resto de UIs ATLAS, orientado a lectura operativa del radar:

- Flag de activación: `ATLAS_ENABLE_RADAR_DASHBOARD=true`
- Ruta principal: `GET /radar/dashboard`
- Assets estáticos:
  - `GET /radar/dashboard/assets/dashboard.css`
  - `GET /radar/dashboard/assets/dashboard.js`

### Endpoints consumidos por la UI

- `GET /api/radar/dashboard/summary`
- `GET /api/radar/diagnostics/providers`
- `GET /api/radar/decisions/recent`
- `GET /api/radar/decisions/stats`
- `GET /api/radar/dealer/{symbol}`
- `GET /api/radar/diagnostics/fast/{symbol}`
- `GET /api/radar/diagnostics/structural/{symbol}`
- `GET /api/radar/political/{symbol}`

### Refresh / frescura / latencia

- Auto-refresh configurable desde UI (5s / 10s / 15s / 30s).
- Refresh manual explícito.
- Timestamp visible de última actualización.
- Latencia visible por ciclo de consulta.
- Indicadores explícitos:
  - estado global de snapshot (`fully_operable`, `operable_with_degradation`, etc),
  - datos frescos / envejecidos / obsoletos,
  - fallback y circuit breaker por provider.

### Alcance y garantías

- No ejecuta órdenes ni toca broker.
- Opera en modo observabilidad + decisión paper supervised.
- Mantiene independencia de módulos de ejecución/portfolio.

## Streaming live opcional (SSE)

Para reducir polling, el radar expone stream servidor->cliente con fallback limpio.

Variables:
- `ATLAS_ENABLE_RADAR_STREAM` (`true|false`, default `false`)
- `ATLAS_RADAR_STREAM_MODE` (`sse`, default `sse`)
- `ATLAS_RADAR_STREAM_HEARTBEAT_SEC` (default `10`)
- `ATLAS_RADAR_STREAM_BUFFER_SIZE` (default `1024`)

Endpoint:
- `GET /api/radar/stream`

Tipos de evento emitidos:
- `snapshot`
- `decision`
- `provider_health`
- `degradation`
- `alert`
- `heartbeat`

Payload mínimo por tipo:
- `snapshot`: `symbol`, `timeframe`, `snapshot_classification`, `fast_pressure_score`, `structural_confidence_score`, `fast_structural_alignment`, `horizon_conflict`, `timestamp`
- `decision`: `symbol`, `timeframe`, `decision`, `reason`, `classification`, `timestamp`
- `provider_health`: `provider_name`, `provider_ready`, `fallback_active`, `circuit_state`, `latency_ms`, `availability`, `timestamp`
- `degradation` / `alert`: `severity`, `domain`, `message`, `timestamp`

Comportamiento dashboard:
- intenta conectar SSE al cargar
- si conecta: muestra `Streaming activo` y reduce polling continuo
- si falla o se congela heartbeat: marca `Conexión degradada` y cae a `Polling activo`
- refresh manual se mantiene como control explícito de respaldo

Limitaciones conocidas:
- bus de eventos en memoria (no persistente entre reinicios del proceso)
- pensado para una instancia de host; multi-worker requiere backend compartido de eventos

## Integración ATLAS V4 (módulo consumible)

Feature flag:
- `ATLAS_RADAR_V4_INTEGRATION_MODE=true|false` (default `false`)

Endpoints:
- `GET /api/radar/v4/summary`
- `GET /api/radar/v4/config`

Objetivo:
- exponer contrato consolidado para dashboard maestro V4 sin acoplar radar al execution layer.
- mantener independencia operativa del dashboard radar.

Referencia de integración completa:
- `docs/atlas_scanner/ATLAS_V4_INTEGRATION.md`

## Diagnostics normalization (provider health)

Todos los providers exponen esquema normalizado:
- `provider_name`
- `provider_ready`
- `fallback_active`
- `circuit_state`
- `latency_ms`
- `p95_latency_ms` (rolling simple)
- `consecutive_errors`
- `last_error`
- `availability`
- `details`

Indicadores operativos agregados:
- `burst_error_indicator`
- `circuit_open_indicator`
- `active_fallback_indicator`

## Economic Calendar (macro)

- Selector: `ATLAS_ECONOMIC_CALENDAR_PROVIDER`
  - `tradingeconomics`
  - `forexfactory`
  - `stub` (default)
- Trading Economics:
  - `ATLAS_TRADING_ECONOMICS_API_KEY`
- Ventanas:
  - `ATLAS_ECONOMIC_CALENDAR_LOOKAHEAD_HOURS` (default `72`)
  - `ATLAS_ECONOMIC_CALENDAR_LOOKBACK_HOURS` (default `24`)

El calendario alimenta `MacroContextSnapshot` y afecta:
- `calendar_risk_score`
- `calendar_volatility_window`
- freshness dinámico de dominio macro
- ponderaciones efectivas en fusión por horizonte

## Circuit Breaker providers (reusable)

- `ATLAS_PROVIDER_CB_FAILURE_THRESHOLD` (default `3`)
- `ATLAS_PROVIDER_CB_COOLDOWN_SEC` (default `120`)

Aplicado en esta fase a:
- `calendar:tradingeconomics`
- `macro:fred`
- `flow:unusual_whales`
- `market:options_chain_openbb`
- `political:finnhub`

Estados:
- `closed`: operación normal
- `open`: rechaza requests hasta cooldown
- `half_open`: request de prueba post-cooldown

## Health providers extendido

`GET /api/radar/health/providers` ahora incluye por provider:
- `avg_latency_ms`
- `error_rate`
- `last_error`
- `fallback_active`
- `provider_ready`
- `consecutive_errors`
- `circuit_state`
- `availability`
- `details` (diagnósticos específicos)

`fallback_active=true` significa que se usó degradación segura (provider alterno/stub).
`circuit_state=open` significa que el provider está temporalmente aislado por fallos consecutivos.

## Dealer positioning profundo (gamma / walls / pinning)

El dominio dealer ahora compone señal desde:
- `gamma_flip_level` robusto con exposición ponderada por strike/OI y cercanía al spot.
- `call_wall` / `put_wall` con weighting por distancia y concentración de OI.
- `pinning_zone` con `pinning_strength`.
- `acceleration_zone` con `acceleration_direction` (`bullish`/`bearish`/`neutral`) y `acceleration_strength`.
- `dealer_pressure_score` (composite interpretable para intradía/swing corto).

### Provider de options chain

- Selector: `ATLAS_OPTIONS_CHAIN_PROVIDER`
  - `stub` (default)
  - `openbb`
- Parámetros:
  - `ATLAS_OPTIONS_CHAIN_TIMEOUT_SEC` (default `6.0`)
  - `ATLAS_OPTIONS_CHAIN_OPENBB_BACKEND` (default `yfinance`)

Comportamiento:
- Si hay chain real suficiente, dealer opera con `source=options_chain`.
- Si falla o no hay payload mínimo, degrada explícitamente a proxy (`flow + iv`) con `degradation_reasons`.
- Se registra health de `options_chain:*` y `gamma:*` en `/api/radar/health/providers`.

### Impacto por horizonte (freshness/ponderación)

- `dealer intraday`: TTL corto (segundos-minutos), dominio dominante.
- `dealer swing`: TTL corto (horas), peso medio.
- `dealer positional`: no operable (peso 0 por diseño).

## Perfil sugerido paper macro calendar real

Derivado del perfil paper actual:
- `ATLAS_MACRO_PROVIDER=fred`
- `ATLAS_FRED_API_KEY=...`
- `ATLAS_ECONOMIC_CALENDAR_PROVIDER=tradingeconomics`
- `ATLAS_TRADING_ECONOMICS_API_KEY=...`
- `ATLAS_PROVIDER_CB_FAILURE_THRESHOLD=3`
- `ATLAS_PROVIDER_CB_COOLDOWN_SEC=120`

## Insider provider real (paper)

- Selector: `ATLAS_INSIDER_PROVIDER`
  - `stub` (default)
  - `fmp` / `financialmodelingprep`
- Credenciales:
  - `ATLAS_FMP_API_KEY`
- Parámetros:
  - `ATLAS_INSIDER_TIMEOUT_SEC` (default `6.0`)
  - `ATLAS_INSIDER_LOOKBACK_DAYS` (default `45`)
  - `ATLAS_INSIDER_MAX_ROWS` (default `100`)
  - `ATLAS_INSIDER_BASE_URL` (default `https://financialmodelingprep.com/api/v4/insider-trading`)

Comportamiento:
- Si falta credencial/backend o campos críticos, insider cae a fallback `stub` con `provider_ready=false`.
- Insider no debe dominar intradía: su impacto va principalmente en `swing/positional` vía freshness y fusión existente.

## Institutional ownership / 13F real (paper)

- Selector: `ATLAS_INSTITUTIONAL_PROVIDER`
  - `stub` (default)
  - `fmp` / `financialmodelingprep`
- Credenciales:
  - `ATLAS_FMP_API_KEY`
- Parámetros:
  - `ATLAS_INSTITUTIONAL_TIMEOUT_SEC` (default `6.0`)
  - `ATLAS_INSTITUTIONAL_LOOKBACK_QUARTERS` (default `4`)
  - `ATLAS_INSTITUTIONAL_MAX_ROWS` (default `80`)
  - `ATLAS_INSTITUTIONAL_BASE_URL` (default `https://financialmodelingprep.com/stable/institutional-ownership/symbol-positions-summary`)

Comportamiento:
- Si falta credencial/backend o faltan campos críticos, ownership cae a `ownership_fallback_stub` (`provider_ready=false`).
- Este dominio es estructural: no trigger intradía; aporta sobre todo en `positional` (y secundario en `swing`) por freshness/TTL.
- Delay esperado de 13F: hasta ~45 días post-quarter, expuesto en `delay_days` / `delay_sec`.

## Regulatory provider real (paper)

- Selector: `ATLAS_REGULATORY_PROVIDER`
  - `stub` (default)
  - `fmp` / `financialmodelingprep`
- Credenciales:
  - `ATLAS_FMP_API_KEY`
- Parámetros:
  - `ATLAS_REGULATORY_TIMEOUT_SEC` (default `6.0`)
  - `ATLAS_REGULATORY_LOOKBACK_DAYS` (default `30`)
  - `ATLAS_REGULATORY_MAX_ROWS` (default `60`)
  - `ATLAS_REGULATORY_BASE_URL` (default `https://financialmodelingprep.com/stable/sec-filings-search/symbol`)

Comportamiento:
- Si provider real falla o faltan señales regulatorias mínimas, fallback seguro a `regulatory_fallback_stub`.
- `overhang_score` y `severity` se derivan de densidad/tipo de filings recientes.
- Regulatory impacta principalmente `swing/positional`, no trigger intradía.

## Political trading provider real (paper)

- Selector: `ATLAS_POLITICAL_PROVIDER`
  - `stub` (default)
  - `finnhub`
- Credenciales:
  - `ATLAS_FINNHUB_API_KEY`
- Parámetros:
  - `ATLAS_POLITICAL_TIMEOUT_SEC` (default `6.0`)
  - `ATLAS_POLITICAL_LOOKBACK_DAYS` (default `120`)
  - `ATLAS_POLITICAL_MAX_ROWS` (default `200`)
  - `ATLAS_POLITICAL_BASE_URL` (default `https://finnhub.io/api/v1/stock/congressional-trading`)

Comportamiento:
- Si falta credencial/backend o faltan campos críticos, fallback seguro a `political_fallback_stub`.
- Exposición de `transaction_count`, `buy_count`, `sell_count`, `buy_sell_balance`, `notable_entities`, `signal_strength`.
- Dominio lento por naturaleza de disclosures (`disclosure_lag_days`): contexto estructural para `swing/positional`; no trigger intradía.
