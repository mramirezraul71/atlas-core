# Options Engine Observability P0 — Handoff Técnico

## 1. Resumen del sprint

- Se cerró el sprint P0 de observabilidad para Options Engine en modo paper con foco en cuatro entregables: contrato canónico de journal, instrumentación de métricas P0, bloque de estado para `8791/ui` y dashboard Health operativo.
- Archivos principales actualizados:
  - `atlas_code_quant/options/options_paper_journal.py`
  - `atlas_code_quant/scripts/options_trade_log_cli.py`
  - `atlas_code_quant/options/options_engine_metrics.py`
  - `grafana/dashboards/atlas-options-health.json`
  - `docs/options_observability_implementation_notes.md`
  - `docs/observability_architecture.md` (versionado en repo)
- Validación automatizada relevante:
  - `atlas_code_quant/tests/test_options_paper_journal.py`
  - `atlas_code_quant/tests/test_options_trade_log_cli.py`
  - `atlas_code_quant/tests/test_options_engine_metrics.py`
- Resultado de verificación reportado para este bloque: `42 passed` (suite acotada al alcance P0).

## 2. Cambios por dominio

### Journal contract

- Se formalizó contrato `journal_version=1.0` en cada evento con campos canónicos para reconstrucción (`trace_id`, `timestamp_utc`, `mode`, `source`, `status`, `autoclose_applied`, `underlying`, `legs`, campos económicos de entrada/cierre, `pnl_usd`, `pnl_pct`).
- Se añadió `OptionsPaperJournal.read_events()` como reader mínimo para consumo offline y auditoría.

### CLI / logging

- `options_trade_log_cli.py` ahora escribe eventos alineados al contrato (`--mode`, `--source`, `close_type`, `pnl_pct`, campos de entrada extendidos).
- Se mantiene validación estricta de cierre (`trace_id`/`symbol`) y secuencia `close_decision` → `close_execution` con mismo `trace_id`.

### Metrics / exporter

- Se consolidaron métricas P0 Health de pipeline, journal, trades, errores y autoclose.
- IV Rank quedó expuesto en dos capas:
  - canónica operativa: `atlas_options_iv_rank_quality` (0/1/2),
  - compatibilidad: `atlas_options_iv_rank_quality_score` (escala 0..1).
- Se expone también `atlas_options_iv_rank_value` para consumo de Signals/Health.
- `get_ui_snapshot()` agrega bloque `options_engine` con claves estables para estado, progreso de trades e IV.

### Grafana Health dashboard

- `atlas-options-health` quedó alineado a métricas P0 y sentinelas.
- H-11 consume `atlas_options_iv_rank_quality` (tier 0/1/2) para evitar ambigüedad con score legacy.

### `8791/ui` payload hook

- El endpoint `GET /api/options-engine-status` ya entrega payload de snapshot con:
  - estado GO/NO-GO/DEGRADED,
  - progreso (`trades_completed_total`, `trades_closed_today`, `trades_target`),
  - WR/PF básicos,
  - IV Rank y su calidad.
- Se conservaron claves legacy previas del snapshot para no romper consumidores existentes.

## 3. Compatibilidad y migración

- **Backward compatibility mantenida:** se conservan `timestamp` y `symbol` junto a `timestamp_utc` y `underlying`; se mantiene `payload` en journal; se mantiene `atlas_options_iv_rank_quality_score` y campos top-level del snapshot.
- **Deprecated pero soportado en P0:** `atlas_options_iv_rank_quality_score` se mantiene por compatibilidad; el tier canónico para dashboards nuevos es `atlas_options_iv_rank_quality`.
- **Migración recomendada para sprint siguiente:** consumidores nuevos deben leer `timestamp_utc`, `underlying`, `atlas_options_iv_rank_quality` y el bloque `options_engine` del snapshot; los aliases legacy quedan en ventana de transición y deben retirarse de forma controlada en una fase posterior.
