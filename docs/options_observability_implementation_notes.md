# Options Engine observability — notas de implementación (sprint P0)

El documento de diseño está versionado en `docs/observability_architecture.md`. Esta entrega sigue la especificación funcional del ticket (métricas P0 Health, dashboard UID `atlas-options-health`, bloque UI en `8791`).

## Addendum — contrato canónico `OptionsPaperJournal` (P0)

- **Versión de contrato:** `journal_version="1.0"` en cada evento JSONL.
- **Campos identitarios comunes:** `trace_id`, `timestamp_utc` (se mantiene alias legado `timestamp`), `mode`, `source`, `status`, `autoclose_applied`, `notes`.
- **Campos de reconstrucción de trade:** `underlying`, `structure_type`, `dte_at_entry`, `legs`, `entry_credit|entry_debit|entry_mid`, `max_loss|max_profit`, `close_type`, `close_reason`, `close_debit|close_credit|close_mid`, `pnl_usd`, `pnl_pct`.
- **Compatibilidad:** se conservan `symbol` y `payload` para readers existentes.
- **Reader mínimo:** `OptionsPaperJournal.read_events()` para reconstrucción offline sin dependencias externas.

## `market_open_config.json` → runtime paper options

- **Carga:** `TradingConfig._apply_market_open_operational_config` en `atlas_code_quant/config/settings.py` (singleton `settings` al importar). Ruta por defecto: `atlas_code_quant/config/market_open_config.json` (`market_open_config_path`).
- **Precedencia por campo:** variable de entorno explícita (no vacía) > JSON > defaults del dataclass (`_fenv` / `_ienv` en definición de campos).
- **Riesgo (JSON `risk`):** `max_positions` **o** alias `max_open_positions` → `market_open_max_positions`; `kelly_fraction` → `kelly_fraction`; `max_risk_per_trade` → `equity_kelly_max_risk_per_trade_pct`; `max_position_pct` → `kelly_max_position_pct`; `daily_loss_limit` → `market_open_daily_loss_limit_pct`. Valores inválidos o fuera de rango: `_risk_*_clamped` + warning, sin crash.
- **Horario (`schedule_et`):** `market_open` / `market_close` (HH:MM); `scan_interval_min`; opcionales `warmup_start` / `premarket_scan` → `market_open_schedule_warmup_et` / `market_open_schedule_premarket_scan_et` (vacíos si ausentes o inválidos; en fallo de archivo/parse se limpian).
- **Exposición al pipeline:** `get_options_engine_market_open_runtime()` / `build_options_engine_market_open_runtime(cfg)` devuelven un dict con valores **efectivos** ya resueltos. `PaperSessionOrchestrator.build_session_plan` adjunta `market_open_runtime` en cada session plan (trazabilidad; no cambia scoring en esta ronda).
- **Re-lectura:** `refresh_market_open_operational_config()` para hot-reload del singleton.

## Procedencia ``atlas_core`` (reproducibilidad mínima, sin migración)

- **Módulo:** `atlas_code_quant/atlas_core_provenance.py` — única API de resolución: `resolve_atlas_core_provenance()`, `get_last_atlas_core_provenance()`, `attach_atlas_core_provenance_to_plan(plan)`.
- **Raíz del repo:** `repo_root_from_atlas_code_quant()` = padre del paquete `atlas_code_quant` (misma convención que el shim `atlas_code_quant/atlas_core/__init__.py`).
- **Canónico declarado:** solo `<repo_root>/atlas_core` (el shim `atlas_code_quant/atlas_core` **no** cuenta como segunda copia del núcleo).
- **Evidencia en el plan paper:** `PaperSessionOrchestrator.build_session_plan` adjunta `atlas_core_provenance` (dict) tras `options_self_audit`. El último dict también se guarda en el módulo de procedencia vía `get_last_atlas_core_provenance()` **(no se duplica dentro de `get_ui_snapshot()` en esta fase; el plan JSON/journal es la traza principal).**
- **Campos:** `resolved_path`, `exists`, `importable`, `source_kind` (`repo_local` / `sibling` / `external` / `unknown`), `canonical_status` (`ok` / `missing` / `ambiguous` / `partial` / `error` / `unknown`), `timestamp`, `notes` (máx. 8 cadenas).
- **Conservador:** no se marca `ok` si el módulo cargado no coincide con `<repo>/atlas_core`, si hay otra carpeta `*/atlas_core` detectable a un nivel bajo la raíz, o si falta el árbol canónico pero `atlas_core` sigue importable desde otra ruta (`ambiguous` + nota). Detección de copias extra: solo `glob("*/atlas_core/__init__.py")` (sin barrido profundo del repo en esta ronda).

## Métricas Prometheus (`atlas_code_quant/options/options_engine_metrics.py`)

| Métrica | Tipo | Descripción breve |
|---------|------|---------------------|
| `atlas_options_session_go_nogo` | Gauge | 0 / 0.5 / 1 según NO-GO, GO degradado, GO |
| `atlas_options_session_go_nogo_count{decision}` | Counter | `go`, `no_go`, `force_no_trade` |
| `atlas_options_pipeline_module_status{module}` | Gauge | `briefing`, `intent_router`, `entry_planner`, `journal`, `autoclose` |
| `atlas_options_pipeline_module_last_run_seconds{module}` | Gauge | **Edad en segundos** desde la última actividad del módulo |
| `atlas_options_journal_events_today` | Gauge | Eventos JSONL con `timestamp` UTC hoy |
| `atlas_options_journal_sessions_today` | Gauge | `event_type=session_plan` hoy |
| `atlas_options_journal_last_write_age_seconds` | Gauge | Edad desde última escritura o `mtime` del archivo |
| `atlas_options_journal_file_size_bytes` | Gauge | Tamaño del JSONL |
| `atlas_options_paper_trades_open` | Gauge | `trace_id` con `entry_execution` y sin `close_execution` |
| `atlas_options_paper_trades_closed_today` | Gauge | `close_execution` hoy |
| `atlas_options_paper_trades_phantom_today` | Gauge | Placeholder `0` |
| `atlas_options_paper_debit_positions_no_stop` | Gauge | Placeholder `0` |
| `atlas_options_autoclose_triggers_total{reason}` | Counter | Razones al cerrar (`take_profit`, `dte_gate`, …) |
| `atlas_options_errors_total{type}` | Counter | Ej.: `paper_session_plan_api` |
| `atlas_options_iv_rank_value` | Gauge | Último IV Rank del briefing (0..100) |
| `atlas_options_iv_rank_quality` | Gauge | Tier canónico de calidad IV: **2=ok**, **1=approx**, **0=insufficient/error** |
| `atlas_options_iv_rank_quality_score` | Gauge | Calidad IV del briefing: **1.0** ok neto, **0.5** aprox / flags IV, **0.25** desconocido, **0.0** `iv_source.error` |
| `atlas_options_flow_payload_available{symbol}` | Gauge | **1** si el snapshot de flow es coherente (`available` y numéricos clave); **0** si falta, incompleto o no interpretable |
| `atlas_options_flow_put_call_volume_ratio{symbol}` | Gauge | Ratio put/call volumen (front). **-1** = sin dato útil (no tratar como señal neutral) |
| `atlas_options_flow_gamma_bias_pct{symbol}` | Gauge | `gamma_bias_pct` del snapshot. **-1** = sin dato |
| `atlas_options_flow_score_pct{symbol}` | Gauge | `score_pct` interno del provider. **-1** = sin dato |
| `atlas_options_flow_confidence_pct{symbol}` | Gauge | `confidence_pct` del snapshot; **0** si payload no coherente |
| `atlas_options_visual_signal_state` | Gauge | Estado **VisualSignalAdapter** en el último session plan: **0** error, **0.2** sin spot/input útil, **0.5** ok sin breach, **0.75** breach no crítico, **1.0** breach crítico |
| `atlas_options_self_audit_state` | Gauge | Self-audit operativo (`operational_self_audit`) en el último session plan: ver mapping conservador en `compute_options_self_audit_state_score` |
| `atlas_options_sentinel_metrics_freshness` | Gauge | **Sentinela** freshness pipeline núcleo (briefing/intent/entry/journal): **1.0** reciente … **0** >24h |
| `atlas_options_sentinel_journal_heartbeat` | Gauge | **Sentinela** edad de escritura journal |
| `atlas_options_sentinel_autoclose_activity` | Gauge | **Sentinela** AutoClose vs silencio (**best-effort**; ver texto abajo) |
| `atlas_options_sentinel_options_flow` | Gauge | **Sentinela** bridge flow: **0.25** si `ATLAS_OPTIONS_FLOW_BRIDGE` off (N/A); **1** snapshot coherente; **0** bridge on sin payload coherente |
| `atlas_options_sentinel_iv_rank_quality` | Gauge | **Sentinela** degradación IV derivada del último `iv_rank_quality_score` |
| `atlas_options_paper_win_rate_ratio` | Gauge | Win rate = ``#(pnl>0) / n`` sobre cierres con ``pnl_realized`` finito; **-1** si ``n < 2`` |
| `atlas_options_paper_profit_factor` | Gauge | ``sum(pnl>0) / abs(sum(pnl<0))``; **-1** si no aplicable (p.ej. todos flat); **99** tope si hay ganancias y **ninguna** pérdida en la muestra |
| `atlas_options_paper_net_realized_pnl_usd` | Gauge | Suma de ``pnl_realized`` en eventos ``close_execution`` (journal completo releído) |
| `atlas_options_paper_max_drawdown_usd` | Gauge | Máximo **peak − equity** sobre equity acumulada (orden archivo), **≥ 0** |
| `atlas_options_paper_performance_close_count` | Gauge | ``n`` de cierres con PnL numérico usados en los cálculos anteriores |

**Paper performance (journal → Prometheus)**

- **Fuente:** mismo JSONL que ``refresh_journal_from_disk``: eventos ``close_execution`` con ``payload.pnl_realized`` numérico finito; orden = orden de líneas del archivo.
- **No contabilidad paralela:** solo lectura; cierres sin ``pnl_realized`` no entran en la muestra (no se imputa 0).
- **Actualización:** cada ``refresh_journal_from_disk`` (orquestador, ``get_ui_snapshot``, sync Grafana, etc.) recalcula y publica gauges + ``_state["last_paper_performance"]`` (``get_last_paper_performance()``).

**VisualSignalAdapter → briefing + métrica**

- **Orquestador:** tras `SessionBriefingEngine.build_briefing`, `PaperSessionOrchestrator._attach_visual_signal_to_briefing` invoca `VisualSignalAdapter.analyze_price_context` con `spot` del briefing, niveles del briefing, strikes opcionales desde `manual_entry_overrides` (`short_put_strike`, `short_call_strike`, `breach_tolerance`), `is_0dte` desde `dte_mode`.
- **Briefing:** se añade `visual_signal` (dict pequeño: `availability` ∈ {`ok`,`unavailable`,`error`}, flags de breach, `breached_level_type`, `source`, `notes`). Se fusiona `breach_context` con el caller vía OR en flags booleanos antes de `build_intent` (misma estructura que espera `OptionsIntentRouter`).
- **Métrica:** `record_session_plan` lee `briefing["visual_signal"]` y publica `atlas_options_visual_signal_state` + `_state["last_visual_signal"]` (`get_last_visual_signal()`).

**Self-audit operativo (paper, no bloqueante)**

- **Código:** `atlas_code_quant/operations/operational_self_audit.py` (`run_operational_self_audit`, `OperationalSelfAuditContext`). Protocolo JSON: `reports/trading_self_audit_protocol.json` vía `read_trading_self_audit_protocol`.
- **Orquestador:** Tras armar `data` (incl. `market_open_runtime` y opcional `options_flow_snapshot`), `PaperSessionOrchestrator` llama `_attach_options_self_audit(data)` **antes** del append al journal. El resultado normalizado va en `options_self_audit` en el session plan.
- **Habilitación:** `QUANT_OPERATIONAL_SELF_AUDIT_ENABLED` explícito; si no está definido, por defecto solo **paper** (`operational_self_audit_enabled`). Si está deshabilitado → `status=skipped`, `passed=null` (no se afirma éxito).
- **Éxito / hallazgos / error:** `status=ok` con `passed`, `overall_severity`, `findings_count`, `blocking_findings` (conteo severidad `BLOCK`). Excepción al ejecutar → `status=error`, `error` con tipo+mensaje, contador `atlas_options_errors_total{type=options_self_audit_exception}`; el ciclo **no** se aborta.
- **Métrica única:** `atlas_options_self_audit_state` = `compute_options_self_audit_state_score(plan["options_self_audit"])`. Snapshot runtime: `_state["last_options_self_audit"]`, `get_last_options_self_audit()`; `get_ui_snapshot()` expone `options_self_audit` (mismo dict reducido).
- **Conservador:** sin bloque `options_self_audit` en el plan → score **0.1** (desconocido). No se usa como gate duro de trading en esta ronda.

**Bridge OptionsFlow → Prometheus**

- **Fuente:** dict devuelto por `OptionsFlowProvider.build_snapshot` (mismo contrato que el scanner). Extracción defensiva en `apply_options_flow_snapshot_to_metrics` + `_options_flow_snapshot_coherent`.
- **Registro en plan:** si `ATLAS_OPTIONS_FLOW_BRIDGE` ∈ `{1,true,yes,on}`, `PaperSessionOrchestrator.build_session_plan` adjunta `options_flow_snapshot` al plan (Tradier paper) antes del journal; si la variable no está activa, no se llama al provider y el plan no lleva clave (métricas quedan en estado conservador al persistir el plan).
- **Persistencia:** `record_session_plan` siempre llama `apply_options_flow_snapshot_to_metrics(symbol, plan.get("options_flow_snapshot"))`, de modo que cada `session_plan` deja reflejado el último estado en gauges y en `_state["last_options_flow"]` (tests / depuración vía `get_last_options_flow_metrics()`).

**Sentinelas explícitos (sin Alertmanager)**

- **API:** `update_options_engine_sentinels()` recalcula cinco gauges y `get_last_sentinel_snapshot()` (dict en runtime). Se invoca al final de `record_session_plan`, `tick_pipeline_ages`, `refresh_journal_from_disk`, `apply_options_flow_snapshot_to_metrics`, `record_autoclose_triggers`.
- **Escala 0..1 (alto = mejor):** **0.25** se usa como *unknown / N/A* (no declarar “salud” plena). Umbrales Grafana sugeridos en paneles H-12…H-16: rojo &lt;0.26, naranja &lt;0.51, amarillo &lt;0.76, verde ≥0.76.
- **metrics freshness:** max edad entre módulos `briefing`, `intent_router`, `entry_planner`, `journal` (`module_last_ts`). Sin marca → 0.25; &gt;24h → 0; &gt;1h → 0.25; &gt;15m → 0.5; &gt;2m → 0.75; else 1.
- **journal heartbeat:** edad desde `last_journal_write_ts`; sin ts → 0.25; &gt;24h → 0; &gt;1h → 0.33; &gt;5m → 0.66; else 1.
- **AutoClose (best-effort):** sin posiciones abiertas (`paper_open_trades`): silencio tolerable (1.0 si edad autoclose &lt;7d; nunca evaluado → 0.5). Con posiciones abiertas: nunca corrido → 0.25; &gt;24h sin actividad → 0; umbrales intermedios por edad. **Limitación:** no se conoce el intervalo esperado de evaluación del motor; es heurística conservadora, no SLA.
- **options flow:** bridge desactivado → **0.25** (no es error, es “no instrumentado”). Bridge on + último `last_options_flow.coherent` → 1 o 0.
- **IV rank quality:** mapeo desde `last_iv_rank_quality_score`: ≥0.9→1; ≥0.5→0.66; ≥0.25→0.33; else 0.

**Coherencia / fallback (conservador)**

- Sin snapshot, no-dict, `available != True`, o falta alguno de `put_call_volume_ratio`, `gamma_bias_pct`, `score_pct`, `confidence_pct` como float finito → `payload_available=0`, ratios/score en **-1**, `confidence_pct=0`.
- No se reutilizan los defaults “neutral” del provider (p. ej. 50 score, 1.0 PCR) cuando el payload no es válido: el centinela **-1** indica explícitamente “no usar como señal”.
- `expected_move` no está en el snapshot actual del provider → **no** hay métrica dedicada en esta ronda.

**IV Rank quality canónico + legacy (conservador)**

- `atlas_options_iv_rank_quality` (**canónico para dashboards operativos P0**): 2=ok, 1=approx, 0=insufficient/error.
- `atlas_options_iv_rank_quality_score` (**legacy soportado**): escala 0..1 conservadora para compatibilidad con consumidores existentes.

**Mapping `atlas_options_iv_rank_quality_score` (conservador)**

- Fuente: dict **briefing** (`SessionBriefingEngine`): `iv_rank_payload_quality`, `iv_source.{quality,error}`, `quality_flags`.
- `iv_source.error` presente → `0.0`.
- Sin `quality` efectiva (vacía) o briefing vacío → `0.25`.
- `quality == "ok"` sin flags IV-degradados → `1.0`; con flags (`iv_rank_quality_*`, `approx_iv_rank`, `iv_rank_fallback_mid`, etc.) → `0.5`.
- `approx` o `insufficient_history` → `0.5`.
- Otro valor de `quality` → `0.25`.

**Puntos de actualización**

- `PaperSessionOrchestrator.build_session_plan`: briefing → **VisualSignalAdapter** (`visual_signal` + `breach_context` fusionado) → intent → entry; opcional `options_flow_snapshot` si `ATLAS_OPTIONS_FLOW_BRIDGE`; **`_attach_options_self_audit`** → **`attach_atlas_core_provenance_to_plan`** → journal si aplica; `record_session_plan`, `refresh_journal_from_disk`, `tick_pipeline_ages`.
- `OptionsPaperJournal._append`: `on_journal_record` (contadores in-memory + gauges si Prometheus activo).
- ``refresh_journal_from_disk``: agrega agregados **paper performance** desde ``close_execution`` / ``pnl_realized``.
- `AutoCloseEngine.evaluate_position`: `record_autoclose_triggers` si `should_close`.
- `GrafanaDashboard.sync_from_canonical`: refresco journal por defecto + edades pipeline.
- `POST /options/paper-session-plan` (except): `record_options_error`.

Requisito: misma librería que el resto de Quant — `prometheus_client` (opcional; si no está instalado, las funciones no rompen el flujo).

## Dashboard Grafana

- **Archivo:** `grafana/dashboards/atlas-options-health.json`
- **UID:** `atlas-options-health`
- **Título:** `Options Engine — Health`
- Paneles **H-01 … H-11** (sesión, pipeline, journal, autoclose+errores, IV raw) más **H-12 … H-16** gauges **sentinelas** (`atlas_options_sentinel_*`).
- Provisioning: misma carpeta `grafana/dashboards` que `atlas.json` (proveedor `atlas` / `atlas-local` ya apunta al directorio).

### Dashboard Signals & Intent (v1)

- **Archivo:** `grafana/dashboards/atlas-options-signals-intent.json`
- **UID:** `atlas-options-signals-intent`
- **Título:** `Options Engine — Signals & Intent`
- Paneles **S-01 … S-05** (`atlas_options_flow_*`) más **S-06** `atlas_options_visual_signal_state`.

### Dashboard Paper Performance (v1)

- **Archivo:** `grafana/dashboards/atlas-options-paper-performance.json`
- **UID:** `atlas-options-paper-performance`
- **Título:** `Options Engine — Paper Performance`
- Paneles **P-01 … P-05** sobre ``atlas_options_paper_*`` listados arriba (sin placeholders).

Variable de entorno sugerida para enlaces UI: `ATLAS_GRAFANA_BASE_URL` (default `http://localhost:3002`).

**`get_ui_snapshot()` (runtime, consumible vía bridge):** además de GO/NO-GO e `iv_rank_quality_score`, incluye `options_self_audit`, `sentinel_snapshot` (mismos cinco sentinelas que `atlas_options_sentinel_*` / H-12…H-16), `win_rate_approx` y `paper_performance_summary` (subconjunto alineado con paneles P-01…P-05: `n_closes`, `win_rate_ratio`, `profit_factor`, `net_realized_pnl_usd`, `max_drawdown_usd`) tras releer el journal; enlaces Grafana a los **tres** dashboards: `grafana_health_dashboard_url`, `grafana_signals_intent_dashboard_url`, `grafana_paper_performance_dashboard_url`.

## UI hub `http://localhost:8791/ui`

- **HTML:** `atlas_adapter/static/dashboard.html` (mínimo; si existía un dashboard completo en otro entorno, restaurar desde `dashboard.html.backup`).
- **API:** `GET /api/options-engine-status` en `bridge/atlas_api_min.py` → delega en `get_ui_snapshot()`.

## CLI `options_trade_log_cli` (cierre)

- **Script:** `atlas_code_quant/scripts/options_trade_log_cli.py`
- **Subcomando:** `close` — antes de `log_close_execution`, llama a `OptionsPaperJournal.log_close_decision` con el mismo `timestamp` UTC y `trace_id`/`symbol` validados (rechazo con código **2** si `trace_id` o `symbol` están vacíos o solo espacios).
- **Payload `close_decision`:** `symbol`, `decision` fija `execute_close` (acción registrada, **no** juicio de calidad), `source`=`options_trade_log_cli.close`, `reason` solo si el usuario pasa `--reason`, y `cli_context` solo con claves presentes en CLI (`debit`, `exit_mid`, `pnl_realized_declared` si se informaron).
- **Journal:** mismo JSONL append-only (`event_type`: primero `close_decision`, luego `close_execution`); `on_journal_record` sigue aplicando por línea.

## Tests

- `atlas_code_quant/tests/test_options_engine_metrics.py` — snapshot UI (enlaces Grafana a los 3 dashboards, `sentinel_snapshot` vs `get_last_sentinel_snapshot`, `win_rate_approx` vs `get_last_paper_performance`), orquestador (incl. **visual_signal** en briefing), **mapping IV / visual signal state / self-audit state**, **options flow**, `record_session_plan` persistencia métricas.
- `atlas_code_quant/tests/test_paper_session_orchestrator.py` — **options_self_audit** en plan (corrida real, skipped, WARN mockeado, excepción degradada + gauge).
- `atlas_code_quant/tests/test_atlas_core_provenance.py` — resolución **ok** / error repo_root / canónico ausente / ambigüedad extra / módulo falso en `sys.modules`, adjunto a plan y snapshot `get_last_*`.
- `atlas_code_quant/tests/test_options_engine_sentinels.py` — snapshot sentinela sano, pipeline stale, journal viejo, IV degradado, flow on/off coherente/incoherente, registry + `record_session_plan`.
- `atlas_code_quant/tests/test_options_paper_journal.py` — **orquestador + VisualSignalAdapter**: breach estructural, excepción del adaptador, spot ausente (`unavailable`).
- **Paper performance:** ``compute_paper_performance_from_pnls``, ``extract_close_realized_pnls_from_journal_text``, ``refresh_journal_from_disk`` → ``get_last_paper_performance`` (casos insuficientes, WR/PF/net/DD, tope sin pérdidas, journal JSONL).
- **`test_market_open_config_runtime.py`:** alias `max_open_positions`, warmup/premarket, snapshot ``build_options_engine_market_open_runtime``, JSON corrupto; **`test_options_paper_journal`:** `market_open_runtime` en session plan.
- `atlas_code_quant/tests/test_options_trade_log_cli.py` — parse/run entry, **close con decisión + ejecución**, rechazo `trace_id` en blanco, **close mínimo sin `reason` inventada** (`entry` sin cambios de contrato).

## Cierre de fase — estado del bloque observabilidad (auditoría humana)

Esta sección no sustituye métricas ni dashboards; fija expectativas para revisión. **No** implica cobertura “completa” de producto fuera del Options Engine paper.

| Bloque | Estado | Notas breves |
|--------|--------|----------------|
| Dashboard Health (`atlas-options-health.json`) | **closed** | Paneles H-01…H-16 y `expr` revisados frente a gauges publicados; IV raw (H-11) + sentinelas (H-12…H-16) alineados con código. |
| Dashboard Signals & Intent | **closed** | S-01…S-06 y series `atlas_options_flow_*` / `atlas_options_visual_signal_state` coinciden con el código. |
| Dashboard Paper Performance | **closed** | P-01…P-05 y `atlas_options_paper_*` coinciden; `get_ui_snapshot().paper_performance_summary` refleja el mismo subconjunto agregado. |
| Sentinelas (`atlas_options_sentinel_*`) | **closed** | Gauges + `sentinel_snapshot` en `get_ui_snapshot()` + documentación de umbrales; sin Alertmanager (explícito). |
| Options flow bridge | **closed** (instrumentación) / **best_effort** (datos) | Lógica y métricas cerradas; calidad del snapshot depende de Tradier/red/provider (fallos → estado conservador documentado). |
| IV rank quality (`atlas_options_iv_rank_quality_score` + sentinela derivado) | **closed** | Briefing → gauge → H-11; sentinela IV en H-16. |
| Visual signal adapter | **closed** | Briefing + gauge + panel S-06; mapping documentado. |
| Self-audit operativo | **partial** | Gauge `atlas_options_self_audit_state`, plan `options_self_audit`, campo en `get_ui_snapshot()`; **no** hay panel Grafana dedicado (la serie existe en Prometheus; no se añadió panel en esta línea para no inflar Health). |
| Paper performance (journal → métricas) | **closed** | Reglas documentadas; gauges y dashboard P-01…P-05 coherentes con `refresh_journal_from_disk`. |
| Phantoms / debit sin stop (gauges) | **partial** | Series presentes; valores aún placeholder hasta reglas de negocio (fuera de esta fase). |
| `atlas_core` provenance | **partial** | Evidencia en session plan + `get_last_atlas_core_provenance()`; **out_of_scope** para `get_ui_snapshot()` salvo evolución futura explícita. |
| Hub 8791 / `dashboard.html` | **out_of_scope** | Contrato expuesto vía bridge no modificado en estas rondas. |
| Alertmanager / alertas externas | **out_of_scope** | No implementado; sentinelas son gauges explícitos para Grafana/PromQL manual. |

**Verificación final (ronda estabilización):** las queries de los tres JSON bajo `grafana/dashboards/atlas-options-*.json` coinciden con nombres `atlas_options_*` definidos en `options_engine_metrics.py`; `get_ui_snapshot()` documentado arriba alinea paper perf y sentinelas con `_state` tras `refresh_journal_from_disk`. No se detectó discrepancia de nombre o `expr` que requiera cambio de código en esta revisión.

## Audit-ready closure

Esta subsección cierra la documentación del bloque de observabilidad **tal como está en el repositorio**; no certifica despliegue en producción ni ausencia de fallos operativos. Sirve para que un revisor humano sepa qué contrastar sin reimplementar el trabajo.

### Alcance real de la fase

- **Incluye:** instrumentación Prometheus (`atlas_options_*` en `atlas_code_quant/options/options_engine_metrics.py`), estado interno `_state` y getters (`get_last_*`, `get_ui_snapshot()`), campos del session plan paper generados por `PaperSessionOrchestrator` (p. ej. `market_open_runtime`, `options_self_audit`, `atlas_core_provenance`, `options_flow_snapshot` condicionado), tres dashboards versionados bajo `grafana/dashboards/atlas-options-*.json`, y tests automatizados referenciados en la sección **Tests** de este documento.
- **Excluye explícitamente:** cambios al hub 8791 o a `atlas_adapter/static/dashboard.html`, Alertmanager u orquestación externa de alertas, integración con Code Quant / OptionStrat / Escaner Quant, reglas de trading nuevas, y habilitación live fuera del modo paper descrito aquí.

### Listas por estado (coherente con la tabla “Cierre de fase” arriba)

- **Cerrados (closed):** dashboards Health, Signals & Intent y Paper Performance (queries ↔ métricas); sentinelas explícitos (`atlas_options_sentinel_*` + actualización documentada); IV rank quality (gauge + H-11 + sentinela H-16); observabilidad VisualSignalAdapter (gauge + S-06); paper performance desde journal hasta gauges P-01…P-05 y resumen en `get_ui_snapshot()`.
- **Best-effort:** **options flow bridge** — la instrumentación y el fallback conservador están cerrados; la **disponibilidad y calidad del snapshot** dependen de red, credenciales Tradier/paper y del `OptionsFlowProvider` (comportamiento degradado ya descrito, no SLA).
- **Parciales (partial):** **self-audit** — observable en gauge, session plan y `get_ui_snapshot().options_self_audit`, **sin panel Grafana dedicado** (la serie existe en Prometheus). **`atlas_core` provenance** — evidencia en session plan y `get_last_atlas_core_provenance()` en `atlas_core_provenance.py`, **no** expuesta en `get_ui_snapshot()`. **Phantoms / debit sin stop** — gauges reservados; lógica de negocio y valores no placeholder **no** cerrados en esta fase.
- **Fuera de alcance (out_of_scope):** hub 8791 / contrato HTML consumidor; **Alertmanager** y gobernanza de alertas gestionadas; expansión de paneles o dashboards más allá de los tres JSON ya existentes salvo trabajo futuro explícito.

### Evidencia verificable (dónde mirar)

| Bloque | Gauge / runtime | Session plan / snapshot | Dashboard | Tests (indicativos) |
|--------|-----------------|-------------------------|-----------|---------------------|
| Sesión GO/NO-GO, pipeline, journal crudo | `atlas_options_session_go_nogo`, `atlas_options_pipeline_*`, `atlas_options_journal_*` | `last_session_plan` vía `_state`; `get_ui_snapshot()` | H-01…H-09 | `test_options_engine_metrics`, orquestador |
| IV rank quality | `atlas_options_iv_rank_quality_score` | briefing en plan; `get_ui_snapshot().iv_rank_quality_score` | H-11 | `test_options_engine_metrics` (IV mapping) |
| Flow / scanner | `atlas_options_flow_*{symbol}` | `options_flow_snapshot` en plan si bridge activo; `_state` `last_options_flow` | S-01…S-05 | `test_options_engine_metrics` (flow), orquestador con bridge |
| Visual signal | `atlas_options_visual_signal_state` | `briefing.visual_signal` | S-06 | `test_options_paper_journal`, orquestador |
| Self-audit | `atlas_options_self_audit_state` | `options_self_audit` en plan; `get_ui_snapshot().options_self_audit` | *(ningún panel en los 3 JSON)* | `test_paper_session_orchestrator`, métricas self-audit |
| Paper performance | `atlas_options_paper_*` | N/A directo; `get_ui_snapshot()` resume agregados | P-01…P-05 | journal + `test_options_engine_metrics` (refresh) |
| Sentinelas | `atlas_options_sentinel_*` | `get_ui_snapshot().sentinel_snapshot` | H-12…H-16 | `test_options_engine_sentinels` |
| Provenance atlas_core | *(no hay gauge dedicado)* | `atlas_core_provenance` en plan; `get_last_atlas_core_provenance()` | — | `test_atlas_core_provenance` |

### Limitaciones explícitas (sin ambigüedad)

- **Options flow bridge:** usable a nivel de código y métricas; la **muestra útil** depende de disponibilidad externa (API/red) y del payload; ausencia o incoherencia se refleja en gauges centinela (-1, `payload_available=0`, sentinela flow bajo), no como “OK silencioso”.
- **Self-audit:** observable en Prometheus (`atlas_options_self_audit_state`) y en API snapshot; **no** hay panel Grafana dedicado en los tres dashboards actuales.
- **`atlas_core` provenance:** trazabilidad en **session plan** y getter de módulo; **no** forma parte del dict `get_ui_snapshot()` en esta fase.
- **Phantoms / debit-without-stop:** fuera del cierre funcional de esta línea; gauges pueden quedar en placeholder hasta reglas reales.
- **Alertmanager / alertas gestionadas:** fuera de alcance; los sentinelas son gauges consultables (Grafana/PromQL manual), no política de paging.

### What a human reviewer should verify next

- Cargar los tres dashboards en un Grafana apuntando al mismo Prometheus que el proceso Quant y confirmar que las series `atlas_options_*` esperadas **aparecen y evolucionan** bajo uso real del orquestador (no solo en tests).
- Revisar un **journal JSONL** real (o de staging): `refresh_journal_from_disk` / contadores y paper performance deben cuadrar cualitativamente con lo visto en P-01…P-05 y en `paper_performance_summary`.
- Con **`ATLAS_OPTIONS_FLOW_BRIDGE`** activado, validar que el bridge produce al menos un snapshot **coherente** o, si falla, que los gauges reflejan degradación (no valores neutros engañosos).
- Revisar **severidad y utilidad** del self-audit operativo (hallazgos WARN vs bloqueos) en logs/plan; el gauge es observacional, no gate de ejecución en esta fase.
- Interpretar **sentinelas** (H-12…H-16) bajo condiciones reales (sesión inactiva vs pipeline parado): umbrales documentados son heurísticos; no sustituyen runbooks.
- Confirmar que **`get_ui_snapshot()`** expone los campos documentados cuando el bridge consume el endpoint (sin asumir que el HTML del hub los muestra todos).
- Si aplica, contrastar **`atlas_core_provenance`** en un session plan exportado con el árbol `atlas_core/` del repo.

## TODOs (siguiente sprint sugerido)

1. Paneles adicionales o variables de símbolo en Grafana; **Paper Performance** y **Signals & Intent** v1 ya existen en `grafana/dashboards/` con queries alineadas a `atlas_options_*`.
2. ~~Métrica **IV rank quality** y panel H-11~~ **(P0 parcial cerrado: gauge + panel Health + tests).**
3. Phantoms y **debit sin stop** con reglas reales (journal + portafolio).
4. Integración Code Quant / OptionStrat / Escaner (solo hooks baratos aquí).
5. Copiar `observability_architecture.md` al repo bajo `docs/` para trazabilidad versionada.
