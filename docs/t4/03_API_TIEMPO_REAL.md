# API tiempo real (FastAPI + WS/SSE)

## 1. Endpoints REST (servicio `t4_robot`)

Base path: `/t4/v1`.

| Método | Path | Descripción | Auth |
|---|---|---|---|
| GET  | `/health` | Liveness | none |
| GET  | `/ready` | Readiness (data_ws, trade_ws, doppler, db) | none |
| GET  | `/status` | Estado agregado (`T4StatusPayload`) | bearer |
| GET  | `/runtime` | `RuntimeResolution` + `LiveSwitchState` | bearer |
| POST | `/connection/connect` | `T4ConnectRequest{target}` | bearer |
| POST | `/connection/disconnect` | — | bearer |
| GET  | `/account` | snapshot de cuenta activa | bearer |
| GET  | `/instruments` | metadata (tick size, contract size, margen intradía) | bearer |
| GET  | `/positions` | posiciones abiertas | bearer |
| GET  | `/orders?status=open\|all` | blotter | bearer |
| POST | `/orders` | `T4OrderRequest` | bearer + `require_live_confirmation` si scope=live |
| DELETE | `/orders/{order_id}` | cancel | bearer |
| POST | `/kill_switch` | aplana y bloquea | bearer + 2FA |
| GET  | `/audit?since=ts&trace_id=…` | audit log filtrado | bearer |
| GET  | `/metrics` | Prometheus exposition | none |

> **Live confirmation**: el decorador existente `api.decorators.require_live_confirmation` se reutiliza. En live exige header `X-Live-Confirm: <one_time_token>` emitido por el dashboard tras el unlock.

## 2. WebSocket `/t4/v1/ws`

Frames `JSON` con el envelope definido en `02_CONTRATOS_DE_DATOS.md §3`.

Topics suscribibles por el cliente:
- `t4.feed.quote`, `t4.feed.depth_l2`, `t4.feed.mbo`, `t4.feed.trade`
- `t4.account.snapshot`
- `t4.order.*`
- `t4.signal`, `t4.risk.decision`
- `t4.connection`, `t4.runtime.mode`

Protocolo de suscripción:
```jsonc
// client → server
{ "op": "subscribe", "topics": ["t4.feed.depth_l2","t4.account.snapshot"] }
// server → client (ack)
{ "type": "event", "topic": "_sub_ack", "payload": { "ok": true, "topics": [...] } }
```

Resnaps: el server emite `snapshot` cada 5 s para `t4.feed.depth_l2` y al `seq_gap`.

## 3. SSE `/t4/v1/sse/audit`

Streaming append-only del `t4.audit` (uni-direccional). Usado por el panel `AuditTimeline` para minimizar latencia frente al WS multiplexado.

## 4. Backpressure y resiliencia

- Cada cliente WS tiene buffer máximo 1 MiB. Si se desborda, el server cierra con code `1009` y el dashboard reconecta.
- `ws_client` cuenta `seq` por topic; ante gap > N publica `_resync_required` y el frontend pide snapshot vía REST.
- Para `t4.feed.mbo` se aplica throttle adaptativo: máx 4000 eventos/s por cliente; excedente se agrega a snapshot.

## 5. Seguridad

- TLS terminado en el reverse-proxy (Caddy o Cloudflare Tunnel).
- Bearer JWT firmado con clave en Doppler; expira 15 min, renovable.
- `X-Live-Confirm` válido 60 s, single-use, anti-replay (jti).
- CORS limitado al origen del dashboard.
- IP allowlist opcional vía variable Doppler `T4_API_ALLOWED_CIDRS`.

## 6. Observabilidad

`/metrics` expone (Prometheus):
- `t4_ws_messages_total{topic, account_scope}`
- `t4_order_latency_ms_bucket{op}` (intent→ack, ack→fill)
- `t4_feed_lag_ms{topic}` (server_ts − local_ts)
- `t4_breaker_state{breaker}` (gauge 0/1/2)
- `t4_account_equity{account_scope}`
- `t4_runtime_mode{mode}` (gauge labeled)
- `t4_connection_state{ws}` (gauge 0=down,1=reconnecting,2=up)
- `t4_audit_chain_verifications_total{result}`

Grafana lee de Prometheus; paneles operativos viven en la SPA.
