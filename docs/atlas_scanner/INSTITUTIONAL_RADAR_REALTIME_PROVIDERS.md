# Institutional Radar Realtime Providers (Fase 2)

## Estado por fuente

- **Market data intradía (OpenBB chain)**: `ready (opt-in)`
  - Implementado en `atlas_scanner/perception/market/openbb_realtime.py`
  - Cadena de fallback: `polygon -> yfinance -> intrinio`
  - Si OpenBB no está disponible en runtime: degradación explícita (`missing_market_data`)

- **Options flow vendor externo (Unusual Whales/análogo)**: `partially ready`
  - Implementación inicial: `unusual_whales_provider.py`
  - Auth: `Authorization: Bearer <API_KEY>`
  - Endpoint usado: `/api/option-trades/flow-alerts`
  - Campos mapeados mínimos: `created_at`, `type`, `strike`, `expiry`, `total_premium`, `total_size`, `has_sweep`, `has_floor`
  - Rate-limit headers capturados si existen: `x-uw-*`
  - Si falla/no hay key: degradación + fallback (`ATLAS_FLOW_PROVIDER_FALLBACK`)

- **Gamma/Greeks/OI full chain**: `partially ready`
  - `gamma_context.py` soporta payload completo de `options_chain` si está presente
  - Si faltan Greeks/OI: ruta degradada con proxy desde flow/spot

- **LEAN options chain integration**: `not wired yet`
  - Arquitectura preparada vía snapshots normalizados y `options_chain` mapping
  - Falta adapter de ingestión LEAN -> `GammaContextSnapshot`

## Plan migración stub -> vendor real

1. **Flow provider adapter v1**
   - Crear `FlowProvider` Protocol (`fetch_events(symbol, since, until)`).
   - Implementar adapter `UnusualWhalesFlowProvider`.
   - Reemplazar `synthetic_flow_events` por provider opt-in desde config/env.

2. **Gamma/OI enrichment v1**
   - Incorporar dataset real de chain (strike gamma + OI).
   - Persistir `gamma_by_strike` y `oi_by_strike` por ventana.
   - Eliminar proxy como ruta principal y dejarlo solo fallback.

3. **Health-driven fallback routing**
   - Usar `RealtimeProviderRegistry.resolve_provider` para elegir proveedor vivo.
   - Umbral de corte por tasa de error y latencia p95.

4. **Observabilidad/replay**
   - Persistir snapshots radar en store temporal (JSONL o sqlite).
   - Reproducir ventanas para debugging de degradación/escenarios.
