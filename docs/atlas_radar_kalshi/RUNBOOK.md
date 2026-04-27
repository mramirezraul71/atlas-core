# Atlas Radar Kalshi — Runbook operativo

> Sistema de trading autónomo sobre mercados Kalshi v2.
> Rama: `feat/atlas-radar-kalshi-autonomous-professional`.
> Adapter principal: `atlas_adapter` (puerto 8791).
> El radar es **opt-in y no invasivo**: se activa con
> `ATLAS_ENABLE_RADAR_KALSHI=1`. Si está apagado, el adapter funciona idéntico
> a antes de PR #14.

---

## 1. Modos de arranque

### 1.1 Modo integrado (recomendado)

El módulo se monta sobre el adapter existente.

```bash
# variables mínimas
export ATLAS_ENABLE_RADAR_KALSHI=1
export ATLAS_RADAR_LIVE=0          # paper
export KALSHI_ENV=demo

# arranque normal del adapter (no cambia)
python -m atlas_adapter.atlas_http_api
```

Endpoints disponibles tras arrancar:

- UI: `http://localhost:8791/ui/radar`
- API: `http://localhost:8791/api/radar/{status,markets,decisions,orders,metrics,risk,health,prometheus}`
- WebSocket en vivo: `ws://localhost:8791/api/radar/stream`
- Kill manual: `POST /api/radar/kill` — Resume: `POST /api/radar/resume`

### 1.2 Modo standalone (sólo el módulo, puerto 8792)

Útil para pruebas aisladas sin tocar el adapter:

```bash
python -m modules.atlas_radar_kalshi.main --serve --host 0.0.0.0 --port 8792
```

### 1.3 Sólo trading loop (sin servidor HTTP)

```bash
python -m modules.atlas_radar_kalshi.main --loop          # Orchestrator v2
python -m modules.atlas_radar_kalshi.main --loop --legacy # PR #14 trading_loop
```

## 2. Variables de entorno

Defaults seguros — **todos** son configurables.

```env
# === Activación / kill ====================================================
ATLAS_ENABLE_RADAR_KALSHI=0|1   # (default 0) opt-in del adapter
ATLAS_RADAR_LIVE=0|1            # (default 0) 1 = órdenes reales
ATLAS_RADAR_KILL=0|1            # (default 0) kill global
ATLAS_RADAR_LEGACY=0|1          # (default 0) usa trading_loop de PR #14

# === Kalshi ==============================================================
KALSHI_ENV=demo|prod
KALSHI_API_KEY=...
KALSHI_PRIVATE_KEY_PATH=/secrets/kalshi.pem

# === Gating ==============================================================
RADAR_EDGE_NET_MIN=0.03         # edge mínimo neto de fees+slippage
RADAR_CONFIDENCE_MIN=0.62
RADAR_SPREAD_MAX=5
RADAR_MIN_DEPTH_YES=50
RADAR_MIN_DEPTH_NO=50
RADAR_MAX_QUOTE_AGE_MS=3000
RADAR_MAX_LATENCY_MS=1500
RADAR_COOLDOWN_S=30

# === Risk engine =========================================================
RADAR_KELLY_FRACTION=0.25
RADAR_MAX_POSITION_PCT=0.05
RADAR_MAX_MARKET_EXP=0.10
RADAR_MAX_TOTAL_EXP=0.50
RADAR_DAILY_DD=0.05
RADAR_WEEKLY_DD=0.10
RADAR_MAX_CL=5                  # max consecutive losses
RADAR_MAX_OPEN=8
RADAR_MAX_OPM=30                # orders per minute

# === Exits ===============================================================
RADAR_TP_PCT=0.6
RADAR_SL_TICKS=4
RADAR_SL_EDGE=-0.02
RADAR_TIME_STOP_S=1800

# === Executor v2 =========================================================
RADAR_PREFER_MAKER=1
RADAR_MAX_CHASE=1
RADAR_MAX_RETRIES=3

# === Ensemble ============================================================
RADAR_W_MICRO=0.30
RADAR_W_MARKOV=0.20
RADAR_W_LLM=0.40
RADAR_W_MOM=0.10
RADAR_CAL_METHOD=platt|isotonic|identity

# === LLM (Ollama) ========================================================
OLLAMA_ENDPOINT=http://127.0.0.1:11434
OLLAMA_MODEL=qwen3:8b
```

## 3. Operación día a día

### 3.1 Arranque

1. Cargar `.env`.
2. Lanzar adapter (o `--serve`).
3. Verificar `GET /api/radar/status` → `ok=true, environment=demo|prod`.
4. Abrir `/ui/radar` y comprobar:
   - KPIs cargan,
   - WebSocket se conecta (badge "live"),
   - Tabla de mercados se llena.

### 3.2 Monitoreo

| Señal | Cómo verla | Acción si |
| --- | --- | --- |
| `radar_safe_mode = 1` | Prometheus, dashboard | Revisar `risk.breakers`. Esperar `roll_day` o reset. |
| `radar_kill_switch = 1` | Prometheus, dashboard | Confirmar si fue manual; si no, investigar y `POST /api/radar/resume`. |
| Latencia p95 alta | KPI `Latencia p95` | Revisar Kalshi status, reducir `RADAR_MAX_OPM`. |
| Fill ratio bajo | KPI `Fill ratio` | Subir `RADAR_MAX_CHASE` o revisar liquidez. |
| `health.degraded = true` | `/api/radar/health` | Reiniciar adapter; el ExitManager cerrará posiciones por `data_degraded`. |

### 3.3 Kill / Resume

```bash
# Kill desde shell
curl -XPOST http://localhost:8791/api/radar/kill

# Kill global persistente (sobrevive a reinicios)
export ATLAS_RADAR_KILL=1

# Resume
curl -XPOST http://localhost:8791/api/radar/resume
unset ATLAS_RADAR_KILL
```

### 3.4 Rollback al estado pre-hardening

Si el módulo nuevo presenta regresiones graves:

```bash
# Opción A: apagar sólo el radar, mantener adapter
unset ATLAS_ENABLE_RADAR_KALSHI    # o =0
# reiniciar adapter

# Opción B: volver al trading_loop de PR #14 (sin tocar código)
export ATLAS_RADAR_LEGACY=1
# reiniciar adapter

# Opción C: revertir a la rama anterior
git checkout feat/atlas-radar-kalshi
```

## 4. Logs y journal

Toda la auditoría queda en `${ATLAS_LOG_DIR}` (por defecto `logs/`):

```
logs/
├── radar.log                     # logs estructurados (RotatingFileHandler)
├── radar_decisions.jsonl
├── radar_orders.jsonl
├── radar_exits.jsonl
├── radar_risk.jsonl
└── radar_calibration.jsonl
```

Para reconstruir métricas a posteriori:

```python
from pathlib import Path
from modules.atlas_radar_kalshi.state.journal import Journal
from modules.atlas_radar_kalshi.metrics import compute

j = Journal(Path("logs"))
print(compute(j).model_dump())
```

## 5. Tests

```bash
PYTHONPATH=. pytest \
  tests/unit/test_radar_kalshi_*.py \
  tests/integration/test_radar_kalshi_*.py
```

Resultado esperado: **65 passed**. Si falla algo:

1. Verificar `python -m py_compile modules/atlas_radar_kalshi/*.py`.
2. Revisar `pip install -r modules/atlas_radar_kalshi/requirements.txt`.

## 6. Backtest

```bash
PYTHONPATH=. python - <<'PY'
import json
from modules.atlas_radar_kalshi.backtest import run_experiment, BTConfig
out = run_experiment(BTConfig(seed=42))
print(json.dumps({k: {kk: vv for kk, vv in v.items() if kk != "equity_curve"}
                  for k, v in out.items() if k != "config"}, indent=2))
PY
```

## 7. Troubleshooting

| Síntoma | Causa probable | Solución |
| --- | --- | --- |
| `ImportError: scikit-learn` | Calibrador isotonic sin sklearn. | `pip install scikit-learn` o `RADAR_CAL_METHOD=platt`. |
| `KalshiSigner` falla | API key/path mal configurados. | Verificar `KALSHI_API_KEY`, `KALSHI_PRIVATE_KEY_PATH`. |
| `WS no conecta` en `/ui/radar` | Proxy / CORS bloquea WSS. | Servir tras reverse proxy con `proxy_set_header Upgrade`. |
| `risk.breakers` se reactiva tras `roll_day` | Daily DD aún en cap. | Ajustar `RADAR_DAILY_DD` o esperar recuperación de equity. |
| `degraded=true` permanente | Scanner sin tráfico (Kalshi caído). | Validar status de Kalshi, reiniciar scanner. |

## 8. Contactos / propietarios

- Owner módulo: Raul Ramírez (`mramirezraul71@gmail.com`).
- Repositorio: `mramirezraul71/atlas-core`.
- PR: ver descripción de `feat/atlas-radar-kalshi-autonomous-professional`.
