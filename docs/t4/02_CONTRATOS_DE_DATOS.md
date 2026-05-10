# Contratos de datos

Toda la comunicación interna usa **dataclasses tipados** (Python) y **TypeScript types** (UI) generados desde el mismo origen (JSON Schema). Los `.proto` de T4 se traducen a estos modelos internos por `codec.py`.

## 1. Eventos del bus interno (Redis pub/sub)

Canal: `atlas.t4.<topic>`. JSON serializado.

### 1.1 `t4.feed.quote`
```jsonc
{
  "v": 1,
  "ts": 1715375321.456,              // epoch float, server time
  "account_scope": "paper|live",
  "instrument": "MESM6",
  "bid": 5260.25, "bid_size": 12,
  "ask": 5260.50, "ask_size":  9,
  "last": 5260.50, "last_size": 1,
  "session_volume": 845321,
  "latency_ms": 11.2
}
```

### 1.2 `t4.feed.depth_l2`
```jsonc
{
  "v": 1, "ts": 1715375321.461,
  "account_scope": "paper",
  "instrument": "MESM6",
  "bids": [[5260.25,12],[5260.00,40],[5259.75,55], …],
  "asks": [[5260.50, 9],[5260.75,38],[5261.00,61], …],
  "levels": 10
}
```

### 1.3 `t4.feed.mbo`
```jsonc
{
  "v": 1, "ts": 1715375321.463,
  "account_scope": "paper",
  "instrument": "MESM6",
  "action": "add|modify|delete|trade",
  "order_id": "T4-9821731",
  "side": "bid|ask",
  "price": 5260.25,
  "size": 12,
  "priority": 7
}
```

### 1.4 `t4.feed.trade`
```jsonc
{
  "v": 1, "ts": 1715375321.464,
  "account_scope": "paper",
  "instrument": "MESM6",
  "price": 5260.50, "size": 3,
  "aggressor": "buy|sell|unknown",
  "trade_id": "T4-TR-871923"
}
```

### 1.5 `t4.signal`
```jsonc
{
  "v": 1, "ts": 1715375322.001,
  "account_scope": "paper",
  "trace_id": "01HXY…",
  "instrument": "MESM6",
  "direction": "long|short|flat",
  "confidence": 0.82,
  "regime": "trending_up|trending_down|ranging|volatile",
  "rationale": ["rsi_14_above_55","macd_hist_positive","delta_imbalance>0.6"],
  "model_version": "xgb_v3.2_2026-04-30"
}
```

### 1.6 `t4.risk.decision`
```jsonc
{
  "v": 1, "ts": 1715375322.045,
  "trace_id": "01HXY…",
  "account_scope": "live",
  "approved": true,
  "size_contracts": 1,
  "max_loss_usd": 25.0,
  "reasoning": {
    "equity": 4520.13, "available_margin": 4350.00,
    "instrument_initial_margin": 50.0,
    "kelly_fraction": 0.012,
    "atr_stop_ticks": 8,
    "tick_value_usd": 1.25,
    "daily_loss_budget_remaining_usd": 90.0
  },
  "blocks": []
}
```

### 1.7 `t4.order.intent` / `t4.order.ack` / `t4.order.fill` / `t4.order.reject` / `t4.order.cancel`
Mismos campos que `OrderRequest`/`PaperOrderResult` (reuso de `api/schemas.py`) **+** `trace_id`, `account_scope`, `t4_order_id`, `t4_exchange_order_id`.

### 1.8 `t4.account.snapshot`
```jsonc
{
  "v": 1, "ts": 1715375322.500,
  "account_scope": "live",
  "account_id": "T4-CUST-12345",
  "equity": 4520.13,
  "cash": 4400.00,
  "unrealized_pnl": 120.13,
  "realized_pnl_today": -35.00,
  "initial_margin_used": 50.00,
  "maintenance_margin": 35.00,
  "available_margin": 4350.00,
  "open_positions": 1,
  "daily_drawdown_pct": 0.77
}
```

### 1.9 `t4.audit`
Append-only. JSONL en disco + Postgres.
```jsonc
{
  "v": 1, "ts": 1715375322.999,
  "trace_id": "01HXY…",
  "actor": "orchestrator|risk|adapter|operator",
  "action": "signal|risk_check|order_intent|order_ack|fill|kill_switch|mode_switch",
  "account_scope": "live",
  "details": { … },
  "hash_prev": "sha256:…",
  "hash_self": "sha256:…"
}
```
> El campo `hash_self = sha256(canonical_json(event_minus_hash_self))` y `hash_prev = hash_self del evento anterior` forman cadena hash de auditoría (modelo blockchain-lite). Verificable offline.

### 1.10 `t4.connection`
```jsonc
{
  "v": 1, "ts": …,
  "account_scope": "live",
  "data_ws": "connected|reconnecting|disconnected",
  "trade_ws": "connected|reconnecting|disconnected",
  "since_ts": …,
  "last_error": null
}
```

### 1.11 `t4.runtime.mode`
Reutiliza payload de `runtime_resolution_to_event()` y `live_switch_state_to_payload()`. Sin cambios.

## 2. REST schemas (FastAPI Pydantic)

Se reusan donde existen. Adiciones:

- `T4OrderRequest(OrderRequest)` — añade `instrument`, `time_in_force ∈ {DAY,GTC,IOC,FOK}`, `bracket: {stop_ticks,target_ticks,trailing_ticks?}`.
- `T4StatusPayload` — agrega `connection`, `account_scope`, `runtime`, `breaker_state`, `last_signal_ts`.
- `T4ConnectRequest` — `target ∈ {simulator,live}`; valida `live_unlock_approved` desde Doppler.

## 3. WebSocket frames hacia el dashboard

Frame envelope:
```jsonc
{ "type": "snapshot|diff|event", "topic": "t4.feed.depth_l2", "payload": { … }, "seq": 12345 }
```

- `snapshot` se envía en `onconnect` y cada N segundos para resincronizar.
- `diff` es el delta del último snapshot (op `add`/`mod`/`del` + key).
- `event` es uni-shot (fill, risk_decision, audit).

## 4. Mapeo Protobuf T4 ⇄ modelo interno (codec)

> El paquete `.proto` exacto lo entrega el clearing; aquí registramos solo el **contrato lógico**.

| Mensaje T4 (placeholder) | Topic interno |
|---|---|
| `QuoteUpdate` | `t4.feed.quote` |
| `DepthSnapshot`, `DepthDelta` | `t4.feed.depth_l2` |
| `MBOAdd`, `MBOModify`, `MBODelete`, `MBOTrade` | `t4.feed.mbo` |
| `TradeTick` | `t4.feed.trade` |
| `OrderAck`, `OrderReject` | `t4.order.ack` / `t4.order.reject` |
| `Fill` | `t4.order.fill` |
| `AccountUpdate` | `t4.account.snapshot` |
| `MarginUpdate` | merged en `t4.account.snapshot` |
| `Heartbeat` | métrica interna; no se publica al bus |

## 5. Reglas de integridad

1. **Idempotencia**: cada `order.intent` lleva `client_order_id = ULID`; el adapter dedupe.
2. **Versionado**: todo evento lleva `v`. Incremento mayor = breaking; el dashboard cierra conexión y resincroniza.
3. **Time sync**: el adapter sella eventos con `ts_server` (T4) **y** `ts_local`. Discrepancia > 250 ms abre breaker.
4. **Compresión**: depth/MBO en WS hacia dashboard usa `permessage-deflate` y diffs por defecto.
