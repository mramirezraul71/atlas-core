# Informe ATLAS AUTONOMOUS – Pegable para Claude

**Objetivo de este documento:** Pegar este informe completo en una conversación con Claude para transferir contexto sobre lo implementado en ATLAS AUTONOMOUS (auto-inspección, auto-corrección, auto-actualización, telemetría, resiliencia y auto-aprendizaje).

---

## 1. Contexto del sistema

- **Repositorio:** ATLAS_PUSH (monorepo), raíz `c:\ATLAS_PUSH`.
- **Componentes:**
  - **PUSH (cerebro):** API FastAPI en puerto **8791**. Entry: `atlas_adapter/atlas_http_api.py`. Dashboard: `http://127.0.0.1:8791/ui`.
  - **NEXUS (cuerpo):** API en **8000**. Entry: `nexus/atlas_nexus/nexus.py --mode api`. Neural Router, Autonomous Engine, Tools Registry, Directives.
  - **Robot (sensores):** Backend en **8002**. Entry: `nexus/atlas_nexus_robot/backend/main.py`. Cámaras, YOLO, visión.
- **Diagnóstico previo:** Se documentaron fortalezas (arquitectura, gobernanza, heartbeat básico) y debilidades (auto-inspección limitada, auto-corrección primitiva, telemetría insuficiente, resiliencia limitada, falta auto-aprendizaje). Ver `docs/DIAGNOSTICO_FORTALEZAS_DEBILIDADES.md` y `docs/MODULOS_PROPUESTOS_ATLAS_AUTONOMOUS.md`.

---

## 2. Misión ejecutada

Se implementaron los **6 subsistemas** de ATLAS AUTONOMOUS bajo la raíz del repo en la carpeta **`autonomous/`**, con **módulos primero e integración después**. La especificación detallada está en `docs/MISION_ATLAS_AUTONOMOUS_PROMPT.md`.

---

## 3. Árbol de archivos creados

```
c:\ATLAS_PUSH\
├── config\
│   └── autonomous.yaml                    # Config de los 6 subsistemas (thresholds, intervals, DB paths)
├── autonomous\
│   ├── __init__.py
│   ├── api_routes.py                     # Router FastAPI: /api/health/*, /api/healing/*, /api/evolution/*, /api/telemetry/*, /api/resilience/*, /api/learning/*
│   ├── health_monitor\
│   │   ├── __init__.py
│   │   ├── system_metrics.py             # CPU, RAM, GPU(opcional), Disk, Network; get_health_score(), detect_anomaly()
│   │   ├── service_health.py             # check_service(), get_all_services_status(), get_bottlenecks()
│   │   ├── anomaly_detector.py           # Isolation Forest / z-score; train(), detect(), predict_failure()
│   │   └── health_aggregator.py          # get_global_health(), start_monitoring(), save_to_db(), get_historical_trend()
│   ├── self_healing\
│   │   ├── __init__.py
│   │   ├── error_classifier.py           # TRANSIENT/CONFIG/RESOURCE/DEPENDENCY/FATAL; classify_error(), get_recovery_strategy()
│   │   ├── recovery_strategies.py        # retry_with_backoff(), restart_service(nexus|robot), rollback, degrade
│   │   ├── circuit_breaker.py           # CLOSED/OPEN/HALF_OPEN; decorator @circuit_breaker
│   │   ├── failure_memory.py             # SQLite: record_failure(), suggest_recovery(), get_recurring_errors()
│   │   └── healing_orchestrator.py       # handle_error(), get_healing_stats(), get_history()
│   ├── evolution\
│   │   ├── __init__.py
│   │   ├── backup_manager.py             # create_snapshot(), list_snapshots(), restore_snapshot(), cleanup_old_snapshots()
│   │   ├── regression_tester.py         # run_api_tests(), run_performance_benchmark(), compare_with_baseline()
│   │   ├── staged_rollout.py             # CANARY→BETA→STABLE→FULL; start_rollout(), advance_phase(), rollback_phase()
│   │   ├── metrics_comparator.py         # capture_baseline(), capture_post_update(), compare_metrics(), should_rollback()
│   │   └── evolution_orchestrator_v2.py  # execute_full_update_pipeline(), validate_update_safety(), get_status()
│   ├── telemetry\
│   │   ├── __init__.py
│   │   ├── metrics_aggregator.py         # collect_metric(), get_metrics(), aggregate(); SQLite
│   │   ├── logs_collector.py             # log(), search_logs(), aggregate_errors(), export_logs()
│   │   ├── tracing_system.py             # start_trace(), add_span(), get_trace(), analyze_slowest_traces()
│   │   ├── dashboard_engine.py           # generate_dashboard_data(health_overview|services_status|self_healing|evolution)
│   │   └── alert_manager.py             # create_alert_rule(), evaluate_rules(), trigger_alert(), acknowledge_alert()
│   ├── resilience\
│   │   ├── __init__.py
│   │   ├── survival_mode.py              # enter_survival_mode(), exit_survival_mode(), is_in_survival()
│   │   ├── priority_queue.py            # CRITICAL/HIGH/MEDIUM/LOW; enqueue(), dequeue(), get_queue_stats()
│   │   ├── resource_throttler.py        # should_throttle(), acquire()/release(), get_throttling_stats()
│   │   └── disaster_recovery.py         # create_full_backup(), create_incremental_backup(), restore_from_disaster()
│   └── learning\
│       ├── __init__.py
│       ├── pattern_analyzer.py           # record_event(), analyze_usage_patterns(), generate_insights()
│       ├── performance_optimizer.py     # suggest_optimizations(), run_optimization_cycle()
│       ├── route_optimizer.py           # Stub para Neural Router
│       ├── feedback_loop.py             # record_feedback(), analyze_feedback(), identify_improvement_areas(); SQLite
│       ├── knowledge_graph.py           # add_entity(), add_relation(), find_related()
│       └── learning_orchestrator.py      # run_learning_cycle(), get_learning_insights()
├── atlas_adapter\
│   └── atlas_http_api.py                 # MODIFICADO: include_router(autonomous_router) con try/except
├── requirements.txt                      # MODIFICADO: psutil, scikit-learn, py-cpuinfo, PyYAML
└── docs\
    ├── DIAGNOSTICO_FORTALEZAS_DEBILIDADES.md
    ├── MODULOS_PROPUESTOS_ATLAS_AUTONOMOUS.md
    ├── MISION_ATLAS_AUTONOMOUS_PROMPT.md
    ├── DESCRIPCION_ESTRUCTURA_ROBOT_IA_A_IA.md
    └── INFORME_ATLAS_AUTONOMOUS_PEGABLE_CLAUDE.md  # Este informe
```

**Bases de datos / logs creados en runtime (bajo `logs/`):**
- `logs/autonomous_health.sqlite` – Health Aggregator (time series de salud).
- `logs/autonomous_failure_memory.sqlite` – Failure Memory (self-healing).
- `logs/autonomous_metrics.sqlite` – Metrics Aggregator (telemetría).
- `logs/autonomous_logs.sqlite` – LogsCollector (logs estructurados).
- `logs/autonomous_learning.sqlite` – FeedbackLoop (learning).

**Snapshots:**
- `snapshots/autonomous_backups/` – BackupManager (Evolution).
- `snapshots/disaster_recovery/` – DisasterRecovery.

---

## 4. Configuración relevante

**config/autonomous.yaml** (resumen):
- `health_monitor`: check_interval_seconds 10, thresholds cpu/ram/disk warning/critical, anomaly_detection enabled, services push/nexus/robot URLs.
- `self_healing`: circuit_breaker (failure_threshold, timeout_seconds, success_threshold), retry_policy (max_attempts, initial_delay, exponential_base), failure_memory_db.
- `evolution`: regression_testing, staged_rollout phase_duration_minutes, auto_rollback degradation_threshold, backup_dir.
- `telemetry`: metrics retention_days, logs retention_days, timeseries_db, alerts webhook_enabled/bitacora_enabled.
- `resilience`: priority_queue max_queue_size, throttling max_concurrent_requests/rate_limit_per_second, survival_mode triggers, backup_dir.
- `learning`: pattern_analysis min_data_points, auto_optimization, knowledge_graph max_nodes, feedback_db.

**requirements.txt** (añadidos):  
`psutil>=5.9.0`, `scikit-learn>=1.3.0`, `py-cpuinfo>=9.0.0`, `PyYAML>=6.0`.  
GPU opcional: `GPUtil` no está en requirements; si se instala, SystemMetrics usa GPU.

---

## 5. Endpoints API (PUSH :8791)

Todos bajo el router montado desde `autonomous.api_routes` (prefix `/api`). Si el módulo autonomous no carga, PUSH sigue funcionando y se registra un warning en log.

| Método | Ruta | Descripción |
|--------|------|-------------|
| GET | /api/health/comprehensive | Salud global (score, components, anomalies, recommendations) |
| GET | /api/health/metrics/system | CPU, RAM, disk |
| GET | /api/health/metrics/services | Estado push/nexus/robot (online, latency_ms) |
| GET | /api/health/anomalies | Anomalías últimas 24h |
| POST | /api/healing/trigger | Trigger healing manual (stub) |
| GET | /api/healing/history | Historial de intentos de healing |
| GET | /api/healing/strategies | Lista de estrategias (retry, restart_service, etc.) |
| GET | /api/healing/stats | total_attempts, success_rate, strategies_used |
| POST | /api/evolution/scan | Stub |
| POST | /api/evolution/update | Ejecuta pipeline: test → backup → baseline → rollout → compare → rollback si degrada |
| GET | /api/evolution/status | in_progress, last_report |
| GET | /api/evolution/history | Stub |
| GET | /api/telemetry/metrics | Métricas (source, time_range opcionales) |
| GET | /api/telemetry/dashboards | Lista de dashboards |
| GET | /api/telemetry/dashboard/{name} | Datos para health_overview, services_status, self_healing, evolution |
| GET | /api/resilience/queue | Estadísticas de la cola prioritaria |
| GET | /api/resilience/throttling | Estado de throttling |
| GET | /api/resilience/survival | active (SurvivalMode) |
| GET | /api/learning/patterns | Patrones de uso |
| GET | /api/learning/insights | Insights del learning orchestrator |
| POST | /api/learning/feedback | body: context, feedback_type, value |

---

## 6. Integración con código existente

- **atlas_http_api.py:** Tras cargar los routers de humanoid/ans/governance, se hace `sys.path.insert(0, BASE_DIR)` y `app.include_router(autonomous_router)` dentro de un `try/except`; si falla, se loguea y se sigue.
- **modules/nexus_heartbeat.py:** No modificado en esta fase. En una siguiente iteración se puede: enviar métricas a HealthAggregator, usar CircuitBreaker en el ping, y llamar a HealingOrchestrator cuando el heartbeat falle de forma consistente.
- **modules/humanoid/governance/state.py:** No modificado. Previsto: emergency_stop → activar SurvivalMode; growth → habilitar auto-optimizaciones; governed → exigir aprobación para optimizaciones.
- **recovery_strategies.py:** Llama a `BackupManager` y `SurvivalMode` si están disponibles (import opcional).

---

## 7. Cómo ejecutar y probar

1. **Dependencias:**  
   `pip install psutil scikit-learn py-cpuinfo PyYAML` (o `pip install -r requirements.txt`).

2. **Arrancar PUSH:**  
   Desde la raíz del repo:  
   `python -m uvicorn atlas_adapter.atlas_http_api:app --host 0.0.0.0 --port 8791`

3. **Probar Autonomous:**  
   - Salud global: `GET http://127.0.0.1:8791/api/health/comprehensive`  
   - Servicios: `GET http://127.0.0.1:8791/api/health/metrics/services`  
   - Healing stats: `GET http://127.0.0.1:8791/api/healing/stats`  
   - Dashboard data: `GET http://127.0.0.1:8791/api/telemetry/dashboard/health_overview`

4. **Si autonomous no carga:** En el log de arranque aparecerá algo como:  
   `Autonomous module not loaded: <mensaje>`. Revisar que la raíz del repo esté en `sys.path` y que no falte ninguna dependencia.

---

## 8. Estado por subsistema

| Subsistema | Estado | Notas |
|------------|--------|--------|
| Health Monitor | Completo | SystemMetrics, ServiceHealth, AnomalyDetector, HealthAggregator; persistencia SQLite |
| Self-Healing | Completo | ErrorClassifier, RecoveryStrategies, CircuitBreaker, FailureMemory, HealingOrchestrator; restart_service usa scripts del repo |
| Evolution 2.0 | Completo | RegressionTester, BackupManager, StagedRollout, MetricsComparator, EvolutionOrchestratorV2; pipeline sin instalación real de paquetes |
| Telemetry Hub | Completo | MetricsAggregator, LogsCollector, TracingSystem, DashboardEngine, AlertManager |
| Resilience | Completo | SurvivalMode, PriorityQueue, ResourceThrottler, DisasterRecovery |
| Learning Engine | Completo | PatternAnalyzer, PerformanceOptimizer, RouteOptimizer(stub), FeedbackLoop, KnowledgeGraph, LearningOrchestrator |

---

## 9. Pendientes / mejoras sugeridas

- **Heartbeat:** Integrar HealthAggregator y CircuitBreaker en `nexus_heartbeat.py`; disparar HealingOrchestrator ante fallos repetidos.
- **Governance:** Enlazar emergency_stop con SurvivalMode y growth/governed con auto-optimización y aprobaciones.
- **Dashboard UI:** Añadir en el dashboard HTML de PUSH vistas que consuman `/api/health/comprehensive` y `/api/telemetry/dashboard/*`.
- **RouteOptimizer:** Conectar con NEXUS Neural Router (métricas por modelo, sugerencia de modelo por query).
- **AlertManager:** Configurar webhook_url y/o bitácora ANS en producción; evaluar reglas periódicamente (p. ej. desde un background task).
- **Tests:** Añadir `tests/autonomous/test_*.py` por subsistema según la especificación del prompt.

---

## 10. Referencias en el repo

- Especificación de la misión: `docs/MISION_ATLAS_AUTONOMOUS_PROMPT.md`
- Descripción técnica IA-a-IA del sistema: `docs/DESCRIPCION_ESTRUCTURA_ROBOT_IA_A_IA.md`
- Fortalezas y debilidades: `docs/DIAGNOSTICO_FORTALEZAS_DEBILIDADES.md`
- Módulos propuestos y mapa de integración: `docs/MODULOS_PROPUESTOS_ATLAS_AUTONOMOUS.md`

---

*Informe generado para handoff a Claude. Repo: ATLAS_PUSH, raíz c:\ATLAS_PUSH. Fecha de referencia: implementación ATLAS AUTONOMOUS (6 subsistemas + integración PUSH).*
