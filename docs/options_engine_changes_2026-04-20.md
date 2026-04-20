# Options Engine Paper Stack - Changes (2026-04-20)

Documento de cierre técnico para revisión, commit y PR interno del stack paper-only del Options Engine.

## 1) Estado habilitado en paper mode

- Loop paper end-to-end activo en `8795` (session tick + autoclose tick).
- Journal paper vivo con eventos:
  - `session_plan`
  - `entry_execution`
  - `close_decision`
  - `close_execution`
- Métricas Prometheus P0 + performance paper activas.
- Endpoint interno de performance paper disponible con cache TTL corto en memoria.
- Hub `8791` consumiendo snapshot de performance para `trades_completed_total`, `wr_basic`, `pf_basic` con fallback seguro.
- Alcance estricto: `paper_only` (sin ejecución live).

## 2) Servicios y puertos implicados

- Hub ATLAS: `http://127.0.0.1:8791`
- Options/Quant backend: `http://127.0.0.1:8795`
- Prometheus: `http://127.0.0.1:9090`
- Grafana: `http://127.0.0.1:3002`

## 3) Endpoints clave

- Hub:
  - `GET /api/options-engine-status`
  - `GET /ui`
- Backend options:
  - `GET /options/paper-performance` (interno, paper Fase 0)
  - `GET /metrics`

## 4) Variables de entorno relevantes

- Runtime loop paper:
  - `QUANT_OPTIONS_RUNTIME_LOOP_ENABLED`
  - `QUANT_OPTIONS_RUNTIME_SYMBOL`
  - `QUANT_OPTIONS_RUNTIME_SESSION_INTERVAL_SEC`
  - `QUANT_OPTIONS_RUNTIME_AUTOCLOSE_INTERVAL_SEC`
  - `QUANT_OPTIONS_RUNTIME_CLOSE_AFTER_SEC`
  - `QUANT_OPTIONS_RUNTIME_CAPITAL`
  - `QUANT_OPTIONS_RUNTIME_JOURNAL_PATH` (opcional)
- Performance endpoint cache:
  - `QUANT_OPTIONS_PERF_CACHE_TTL_SEC` (rango efectivo 5..15s; default 10s)

## 5) Validaciones recomendadas antes de push

### Tests principales

```bash
python -m pytest -q atlas_code_quant/tests/test_options_paper_runtime_loop.py
python -m pytest -q atlas_code_quant/tests/test_paper_journal_stats.py
python -m pytest -q atlas_code_quant/tests/test_options_paper_performance_endpoint.py
python -m pytest -q tests/atlas_adapter/test_options_engine_status_endpoint.py
python -m pytest -q atlas_code_quant/tests/test_options_paper_journal.py
```

### Endpoints HTTP

```bash
curl -sS http://127.0.0.1:8795/options/paper-performance
curl -sS http://127.0.0.1:8791/api/options-engine-status
curl -I   http://127.0.0.1:8791/ui
```

### Métricas clave (ejemplos)

- `atlas_options_session_go_nogo`
- `atlas_options_pipeline_module_status{module="autoclose"}`
- `atlas_options_journal_last_write_age_seconds`
- `atlas_options_paper_closed_total`
- `atlas_options_paper_win_rate_pct`
- `atlas_options_paper_profit_factor`
- `atlas_options_paper_equity_usd`
- `atlas_options_paper_win_rate_by_strategy_regime`

## 6) Notas de implementación relevantes

- `close_execution` nuevo materializa campos semánticos:
  - `strategy_type`
  - `gamma_regime`
  - `dte_mode`
- El agregador de paper performance prioriza esos campos directos en cierre.
- Compatibilidad histórica: si faltan campos en eventos viejos, se usa fallback por `trace_id`.

## 7) Limitaciones actuales (esperadas)

- `paper_only`: sin live trading.
- Eventos históricos pueden mantener `gamma_regime="unknown"` si fueron escritos antes de la materialización.
- Cache de `/options/paper-performance` es en memoria del proceso (no distribuido).

## 8) Artefactos runtime/local

- `atlas_code_quant/data/options_paper_stats.json` es artefacto runtime/debug.
- Debe quedar fuera de versionado (`.gitignore` actualizado).
