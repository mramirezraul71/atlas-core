# ATLAS V4 Integration Guide (Institutional Radar)

## Objetivo

Integrar Institutional Radar como módulo consumible por ATLAS PUSH V4 sin acoplamiento fuerte:

- Radar sigue operativo standalone (API + dashboard propio).
- V4 puede consumir vista radar por API consolidada.
- Si V4 cae, radar sigue funcionando.
- Si radar cae, V4 puede degradar esa vista sin romperse.

## Arquitectura de integración

- Productor: `atlas_scanner` (FastAPI `/api/radar/*`).
- Consumidor: dashboard maestro V4 (futuro).
- Contrato principal para embedding: `GET /api/radar/v4/summary`.
- Contrato de capacidades: `GET /api/radar/v4/config`.
- Live opcional: `GET /api/radar/stream` (SSE).

## Feature flag V4

- `ATLAS_RADAR_V4_INTEGRATION_MODE=true`
  - habilita endpoints `/api/radar/v4/*`
  - cuando está `false`, responde `404 radar_v4_integration_disabled`

## Endpoints recomendados para V4

### 1) Snapshot consolidado (primario)

`GET /api/radar/v4/summary?symbol=SPY&top_n=5&decisions_limit=20`

Campos clave:
- `radar_status`
- `top_signals`
- `gate_recent_decisions`
- `provider_health_summary`
- `degradations_active`
- `structural_context_summary`
- `fast_context_summary`
- `freshness`
- `last_update`
- `stream_available`

Uso: render principal de la pestaña radar en V4 con un solo call.

### 2) Capacidades/config (bootstrap)

`GET /api/radar/v4/config`

Campos clave:
- `domains_active`
- `providers_available`
- `modes_supported`
- `timeframes_supported`
- `streaming`
- `decision_gate_thresholds`

Uso: bootstrapping de UI, toggles y validación de capacidades.

### 3) Stream live opcional

`GET /api/radar/stream`

Tipos: `snapshot`, `decision`, `provider_health`, `degradation`, `alert`, `heartbeat`.

Uso: actualización push de vista V4.  
Fallback: polling a `/api/radar/v4/summary`.

## Polling vs Streaming

- Recomendado base: polling cada `10-15s` a `/api/radar/v4/summary`.
- Recomendado live: usar SSE y bajar polling a respaldo (`30-60s` o manual).
- Si heartbeat expira o stream cae:
  - marcar estado degradado en V4,
  - volver a polling,
  - mantener UX operativa sin bloquear.

## Interpretación operativa para V4

- `snapshot_classification`:
  - `fully_operable`: operación normal.
  - `operable_with_degradation`: operar con cautela + badge degradado.
  - `structural_only` / `fast_only`: contexto parcial.
  - `non_operable`: no confiable para decisión.
- `provider_health_summary`:
  - `circuit_open_indicator=true` y/o `active_fallback_indicator=true` -> degradación visible.
- `degradations_active`:
  - mostrar panel de alertas narrativas.
- Composites:
  - structural: tendencia de fondo.
  - fast: presión táctica.
  - conflicto (`horizon_conflict`): elevar cautela.

## Manejo de errores en V4

- `404 radar_v4_integration_disabled`: ocultar módulo radar en V4 o mostrar "no habilitado".
- `5xx`: mostrar estado temporal no disponible; conservar última data cacheada.
- stream indisponible: fallback automático a polling.

## CORS / seguridad

Estado actual:
- integración recomendada same-host (sin necesidad de CORS adicional).
- si V4 corre en otro origen, habilitar CORS explícito en adapter en fase posterior.

Autenticación:
- no forzada en esta fase (entorno interno/paper).
- recomendado futuro: token service-to-service para `/api/radar/v4/*`.

## Handoff para equipo V4

1. Consumir `GET /api/radar/v4/config` al montar módulo.
2. Pintar layout base con `GET /api/radar/v4/summary`.
3. Si `stream_available=true`, abrir SSE a `/api/radar/stream`.
4. Escuchar `snapshot/decision/provider_health/degradation/alert/heartbeat`.
5. Si stream cae, fallback a polling y badge "conexión degradada".
6. Reutilizar componentes visuales radar: badges de estado, KPIs, alertas narrativas.

## Script mock de validación

`python scripts/radar_v4_mock_consumer.py --base-url http://127.0.0.1:8795 --symbol SPY`

Valida:
- contrato estructural de `/api/radar/v4/summary`
- respuesta básica del stream SSE (si disponible)
