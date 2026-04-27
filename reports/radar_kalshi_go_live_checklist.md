# Radar Kalshi — Checklist Go-Live

> Rama: `feat/atlas-radar-kalshi-autonomous-professional`
> Última actualización: este commit.
> **Regla de oro**: nada se considera listo hasta que tenga evidencia auditada
> (logs JSONL, métricas Prometheus, captura de dashboard o output de pytest).

## 0. Pre-flight

- [ ] Confirmar rama mergeada en `main` y tag `radar-v0.2.0` creado.
- [ ] CI verde — `pytest tests/` debe pasar **188/188** en CI.
- [ ] `python -m py_compile modules/atlas_radar_kalshi/*.py modules/atlas_radar_kalshi/state/*.py modules/atlas_radar_kalshi/dashboard/*.py` → exit 0.
- [ ] Variables sensibles (`KALSHI_API_KEY`, `KALSHI_PRIVATE_KEY_PATH`) cargadas en
  `.env` y nunca versionadas.

## 1. Configuración por entorno

### Paper (recomendado para arranque)

```env
ATLAS_ENABLE_RADAR_KALSHI=1
ATLAS_RADAR_LIVE=0
ATLAS_RADAR_KILL=0
KALSHI_ENV=demo

# Gating
RADAR_EDGE_NET_MIN=0.03
RADAR_CONFIDENCE_MIN=0.62
RADAR_SPREAD_MAX=5
RADAR_MIN_DEPTH_YES=50
RADAR_MIN_DEPTH_NO=50
RADAR_MAX_QUOTE_AGE_MS=3000
RADAR_MAX_LATENCY_MS=1500
RADAR_COOLDOWN_S=30

# Risk
RADAR_KELLY_FRACTION=0.25
RADAR_MAX_POSITION_PCT=0.05
RADAR_MAX_MARKET_EXP=0.10
RADAR_MAX_TOTAL_EXP=0.50
RADAR_DAILY_DD=0.05
RADAR_WEEKLY_DD=0.10
RADAR_MAX_CL=5
RADAR_MAX_OPEN=8
RADAR_MAX_OPM=30

# Exits
RADAR_TP_PCT=0.6
RADAR_SL_TICKS=4
RADAR_SL_EDGE=-0.02
RADAR_TIME_STOP_S=1800

# Ensemble weights (deben sumar > 0; se normalizan)
RADAR_W_MICRO=0.30
RADAR_W_MARKOV=0.20
RADAR_W_LLM=0.40
RADAR_W_MOM=0.10

# LLM
OLLAMA_ENDPOINT=http://127.0.0.1:11434
OLLAMA_MODEL=qwen3:8b
```

### Live (sólo tras ≥ 1 semana en paper con criterios cumplidos)

```env
ATLAS_RADAR_LIVE=1
KALSHI_ENV=prod
KALSHI_API_KEY=...
KALSHI_PRIVATE_KEY_PATH=/secrets/kalshi.pem
```

## 2. Criterios de Go-Live (pre-condición obligatoria)

Verificar **sobre paper en demo durante ≥ 7 días**:

| Criterio | Umbral | Fuente |
| --- | --- | --- |
| Trades ejecutados | ≥ 30 | `radar_trades_total` |
| Profit factor | ≥ 1.20 | `/api/radar/metrics` → `performance.profit_factor` |
| Hit-rate | ≥ 50% | idem |
| Expectancy | > 0 | idem |
| Max drawdown / equity inicial | ≤ 10% | `/api/radar/metrics` → `performance.max_drawdown_cents` |
| Risk violations | 0 | `journal.risk` |
| Crashes (PID muere) | 0 | systemd / PM2 logs |
| Latencia p95 órdenes | ≤ 1 500 ms | `radar_latency_p95_ms` |
| Fill ratio | ≥ 60% | `radar_fill_ratio` |

## 3. Plan de arranque

```bash
# 1) Activar opt-in
export ATLAS_ENABLE_RADAR_KALSHI=1
export ATLAS_RADAR_LIVE=0

# 2) Lanzar atlas_adapter (puerto 8791) — el radar se monta automático
cd /path/to/atlas-core
python -m atlas_adapter.atlas_http_api  # o el comando habitual

# 3) Verificar dashboard
curl http://localhost:8791/api/radar/status
# Abrir UI: http://localhost:8791/ui/radar
```

## 4. Monitoreo continuo

- Dashboard: `http://localhost:8791/ui/radar`
  - KPIs visibles: PnL neto, Hit-rate, Profit factor, Drawdown, Expectancy,
    Latencia p95, Fill ratio, Exposición, Estado de riesgo (healthy/safe/kill).
  - WebSocket en vivo en `/api/radar/stream`.
- Prometheus scrape: `GET /api/radar/prometheus`.
- Journal en disco: `${ATLAS_LOG_DIR}/radar_*.jsonl`.

## 5. Plan de respuesta (rollback / kill)

| Escenario | Acción |
| --- | --- |
| Anomalía operativa puntual | `POST /api/radar/kill` (botón en dashboard). |
| Anomalía persistente | `export ATLAS_RADAR_KILL=1` y reiniciar adapter. |
| Regresión confirmada del módulo | `unset ATLAS_ENABLE_RADAR_KALSHI` y reiniciar adapter — atlas-core sigue corriendo sin el radar. |
| Necesidad de revertir código | `git checkout feat/atlas-radar-kalshi` (= PR #14). |

## 6. Salida controlada (off-boarding)

Antes de apagar el proceso:

1. Cancelar órdenes vivas: `executor.reconcile()` y revisar `radar_orders.jsonl`.
2. Forzar exits con `ATLAS_RADAR_KILL=1` para que `ExitManager` cierre todas las
   posiciones con `reason="forced"`.
3. Snapshot final de métricas (`/api/radar/metrics`).
4. Apagar adapter.

## 7. Aceptación final

- [ ] Reportes commiteados en `reports/`.
- [ ] Runbook publicado en `docs/atlas_radar_kalshi/RUNBOOK.md`.
- [ ] CI verde en la PR.
- [ ] Aprobación del owner del módulo (Raul / `mramirezraul71@gmail.com`).
- [ ] Captura del dashboard adjunta a la PR mostrando KPIs no-cero (paper).
- [ ] Decisión registrada para pasar de paper → live (o no), con criterios verificados.
