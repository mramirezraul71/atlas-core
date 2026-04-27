# Radar Kalshi — Auditoría de controles de riesgo

> Rama: `feat/atlas-radar-kalshi-autonomous-professional`
> Componentes: `risk_engine.py`, `gating.py`, `exit_manager.py`, `executor_v2.py`,
> `orchestrator.py`, `state/journal.py`.

Este documento mapea **cada control de riesgo** a su implementación, su prueba
automatizada y su métrica observable, de modo que el control pueda auditarse en
producción sin acceder al código.

## 1. Capas de defensa (depth in defense)

| Capa | Componente | Función |
| --- | --- | --- |
| 1. Entrada de datos | `scanner.py` | Detección de stale ticks, profundidad, cierres. |
| 2. Selección | `gating.py` | edge_net, confidence, spread, depth, cooldown. |
| 3. Sizing | `risk_engine.py` | Kelly fraccionario + caps por posición/mercado/total. |
| 4. Breakers | `risk_engine.py` | DD diario/semanal, pérdidas consecutivas, rate-limit. |
| 5. Kill-switch | `risk_engine.py` + env | Cierra entradas + fuerza salidas. |
| 6. Salida | `exit_manager.py` | TP / SL / time-stop / forced-exit / data-degraded. |
| 7. Ejecución | `executor_v2.py` | Idempotencia + reintentos + reconcile. |
| 8. Auditoría | `state/journal.py` | JSONL append-only + métricas Prometheus. |

## 2. Mapa control → código → test → métrica

| Control | Implementación | Test | Métrica/Endpoint |
| --- | --- | --- | --- |
| Edge neto mínimo | `gating.evaluate` (`edge_net = |edge_gross| - cost_prob`) | `test_radar_kalshi_gating::test_reject_low_edge` | `gate_reason="edge_net=..."` en `/api/radar/markets` |
| Confidence mínimo | `gating.evaluate` | `test_reject_low_confidence` | idem |
| Spread máximo (ticks) | `gating.evaluate` | `test_reject_wide_spread` | `spread` en `/api/radar/markets` |
| Profundidad mínima | `gating.evaluate` | `test_reject_thin_depth_*` | `depth_yes`, `depth_no` |
| Antigüedad de quote | `gating.evaluate` | `test_reject_quote_stale` | `gate_reason="quote_age=..."` |
| Latencia HTTP | `gating.evaluate` | `test_reject_high_latency` | `radar_latency_p95_ms` (Prometheus) |
| Cooldown por mercado | `_Cooldowns`, `Gating.stamp` | `test_cooldown_blocks_repeat`, `test_cooldown_does_not_block_other_market` | `gate_reason="cooldown"` |
| Kelly fraccionario | `RiskEngine.kelly`, `RiskLimits.kelly_fraction` | `test_kelly_*` | `f_full`, `f_capped` en journal `radar_orders` |
| Cap por posición | `max_position_pct` | `test_sizing_respects_max_position_pct` | `radar_exposure_cents` |
| Cap por mercado | `max_market_exposure_pct` | `test_market_cap_blocks_repeats` | idem |
| Cap total | `max_total_exposure_pct` | `test_total_cap_blocks_global_exposure` | `radar_exposure_cents` |
| Drawdown diario | `daily_dd_limit_pct` (`_check_breakers`) | `test_daily_dd_breaker` | `radar_safe_mode`, `risk.breakers` |
| Drawdown semanal | `weekly_dd_limit_pct` | `test_weekly_dd_breaker` | idem |
| Pérdidas consecutivas | `max_consecutive_losses` | `test_consecutive_losses_breaker` | `risk.consecutive_losses` |
| Posiciones abiertas | `max_open_positions` | `test_max_open_positions_breaker` | `risk.open_positions` |
| Rate limit órdenes | `max_orders_per_minute` | `test_rate_limit_breaker` | `risk.breakers="rate_limit"` |
| Kill manual | `RiskEngine.kill()` + `POST /api/radar/kill` | `test_kill_manual` | `radar_kill_switch` |
| Kill via env | `ATLAS_RADAR_KILL=1` | `test_kill_via_env` | idem |
| Take-profit | `ExitManager.evaluate` (precio cruza target) | `test_take_profit_yes`, `test_take_profit_no` | `journal.exits.reason="take_profit"` |
| Stop-loss en ticks | `sl_ticks` | `test_stop_loss_ticks` | `journal.exits.reason="stop_loss_ticks"` |
| Stop por edge invertido | `sl_edge_revert` | `test_edge_revert` | `journal.exits.reason="edge_revert"` |
| Time-stop | `time_stop_seconds` | `test_time_stop` | `journal.exits.reason="time_stop"` |
| Forced exit (kill) | `evaluate(forced=True)` | `test_forced_overrides` | `journal.exits.reason="forced"` |
| Data degraded | `evaluate(data_degraded=True)` | `test_data_degraded` | `health.degraded` |
| Idempotency key | `KalshiExecutorV2.make_client_order_id` (sha1 + bucket ms) | (cubierto por compilación + audit) | `client_order_id` en `radar_orders.jsonl` |
| Retries con backoff | `submit()` (exponencial + jitter) | (cubierto en backtest) | `metrics.attempts`, `metrics.errors` |
| Reconcile post-crash | `executor_v2.reconcile()` | (smoke en main loop) | salida JSON en logs |
| Watchdog | `Orchestrator._watchdog` | (smoke) | `health.degraded` |

## 3. Auditoría operativa

Todos los eventos relevantes quedan persistidos en JSONL bajo `${ATLAS_LOG_DIR}`:

- `radar_decisions.jsonl`: `{ts, ticker, decision, readout}`
- `radar_orders.jsonl`: `{ts, request, result}`
- `radar_exits.jsonl`: `{ts, ticker, reason, entry, exit, size, pnl_cents, fees_cents, slippage_cents}`
- `radar_risk.jsonl`: `{ts, event, rationale}`
- `radar_calibration.jsonl`: `{ts, p_raw, outcome}`

El endpoint `/api/radar/prometheus` emite métricas en formato OpenMetrics:

```
radar_pnl_cents
radar_trades_total
radar_hit_rate
radar_profit_factor
radar_max_drawdown_cents
radar_latency_p95_ms
radar_fill_ratio
radar_safe_mode
radar_kill_switch
radar_exposure_cents
```

## 4. Procedimientos de respuesta

| Síntoma | Acción inmediata | Validación |
| --- | --- | --- |
| `radar_safe_mode = 1` | Revisar `risk.breakers` en `/api/radar/risk`. Esperar `roll_day` o desactivar manual. | `radar_safe_mode = 0` |
| `radar_kill_switch = 1` no esperado | `POST /api/radar/resume` o `unset ATLAS_RADAR_KILL`. | `radar_kill_switch = 0` |
| Latencia p95 > 1500 ms | Reducir tráfico (`RADAR_MAX_OPM`), revisar Kalshi status. | `radar_latency_p95_ms` baja |
| Fill ratio < 50% | Subir `RADAR_MAX_CHASE` o cambiar a market orders puntualmente. | `radar_fill_ratio` sube |
| `health.degraded = true` | Reiniciar scanner. Forzar exits (`forced=true`). | `health.degraded = false` |
| DD diario rebasa cap | El sistema entra en safe-mode automáticamente. **Detener trading manual**. | `roll_day` al día siguiente |

## 5. Pruebas automatizadas

```bash
PYTHONPATH=. pytest \
  tests/unit/test_radar_kalshi_risk.py \
  tests/unit/test_radar_kalshi_gating.py \
  tests/unit/test_radar_kalshi_signals.py \
  tests/unit/test_radar_kalshi_exits.py \
  tests/integration/test_radar_kalshi_routes.py
```

Resultado esperado (commit actual): **65 passed**.

## 6. Limitaciones

- La calibración isotónica requiere `scikit-learn`. Sin ella se hace **fallback a Platt**.
- El paper trader no simula latencia variable; las métricas p50/p95 sólo son
  representativas en `ATLAS_RADAR_LIVE=1`.
- `executor_v2.reconcile()` depende del cumplimiento del contrato REST de Kalshi v2; un
  cambio de schema requeriría revisar `_submit_once`.

## 7. Aceptación

- [x] Todos los breakers tienen test unitario verde.
- [x] El kill-switch puede activarse manualmente y por env.
- [x] Cada decisión / orden / salida queda persistida en JSONL.
- [x] Existe endpoint Prometheus con SLI mínimos.
- [x] El dashboard expone botones kill/resume y muestra estado de breakers.
