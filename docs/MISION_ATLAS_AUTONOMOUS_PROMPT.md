# MISIÓN ARQUITECTÓNICA: ATLAS AUTONOMOUS - Especificación completa

**Nota de ubicación:** En el monorepo actual la base es **`c:\ATLAS_PUSH`** (no `C:\ATLAS_NEXUS\atlas_nexus_robot`). Los módulos `autonomous/` deben crearse bajo la **raíz del repo** (`c:\ATLAS_PUSH\autonomous\`). La integración principal es vía **PUSH** (`atlas_adapter/atlas_http_api.py`). NEXUS y Robot viven en `nexus/atlas_nexus/` y `nexus/atlas_nexus_robot/backend/` dentro del mismo repo.

---

═══════════════════════════════════════════════════════════════════════════
MISIÓN ARQUITECTÓNICA: ATLAS AUTONOMOUS - Sistema de Auto-Gestión Completo
═══════════════════════════════════════════════════════════════════════════

CONTEXTO: ATLAS es un sistema robótico inteligente con 3 componentes (PUSH cerebro :8791, NEXUS cuerpo :8000, Robot sensores :8002). Actualmente tiene gobernanza básica pero carece de autonomía verdadera. Debes implementar 6 subsistemas que le den capacidades de auto-inspección, auto-corrección, auto-actualización, telemetría avanzada, resiliencia y auto-aprendizaje.

UBICACIÓN BASE: **c:\ATLAS_PUSH** (raíz del monorepo). Crear **autonomous/** en la raíz.

ARQUITECTURA EXISTENTE:
- PUSH (8791): atlas_adapter/atlas_http_api.py, modules/nexus_heartbeat.py, modules/humanoid/ans/*, modules/humanoid/governance/state.py
- NEXUS (8000): nexus/atlas_nexus/nexus.py, api/rest_api.py, brain/neural_router.py, brain/autonomous_engine.py, tools/tools_registry.py
- Robot (8002): nexus/atlas_nexus_robot/backend/main.py, api/vision_routes.py, vision/yolo_detector.py

═══════════════════════════════════════════════════════════════════════════
SUBSISTEMA 1: HEALTH MONITOR - Auto-Inspección Total
═══════════════════════════════════════════════════════════════════════════

Crear: **autonomous/health_monitor/system_metrics.py**

Clase SystemMetrics que monitorea cada 10 segundos:
- CPU: uso por core, temperatura, throttling
- RAM: total, usada, disponible, swap
- GPU: uso VRAM, temperatura (si disponible)
- Disk: espacio usado/libre en C:, I/O operations
- Network: bandwidth uso, latency, packet loss

Usar: psutil para métricas de sistema, GPUtil para GPU

Métodos:
- get_current_metrics() → dict con todas las métricas
- is_healthy() → bool (CPU <80%, RAM <85%, Disk <90%)
- get_health_score() → 0-100 basado en thresholds
- detect_anomaly(historical_data) → bool si hay desviación >2σ

Crear: **autonomous/health_monitor/service_health.py**

Clase ServiceHealth que monitorea cada servicio (PUSH, NEXUS, Robot):
- Latency: tiempo de respuesta /health endpoint
- Error rate: % de requests fallidas últimos 5 min
- Throughput: requests/segundo
- Uptime: tiempo desde último restart
- Dependencies: estado de LLMs, DB, cache

Métodos:
- check_service(url, timeout=5) → ServiceStatus(online, latency, errors)
- get_dependency_health() → dict de dependencias y su estado
- calculate_service_score() → 0-100
- get_bottlenecks() → lista de cuellos de botella detectados

Crear: **autonomous/health_monitor/anomaly_detector.py**

Clase AnomalyDetector usando isolation forest o z-score:
- Entrena con métricas históricas (últimos 7 días)
- Detecta desviaciones anormales en tiempo real
- Clasifica anomalías: leve, moderada, severa
- Genera alertas predictivas

Métodos:
- train(historical_metrics) → entrena modelo
- detect(current_metrics) → AnomalyReport(detected, severity, metrics_affected)
- predict_failure(current_trend) → tiempo estimado hasta fallo

Crear: **autonomous/health_monitor/health_aggregator.py**

Clase HealthAggregator que centraliza todo:
- Combina SystemMetrics + ServiceHealth + AnomalyDetector
- Calcula salud global del sistema (0-100)
- Genera reporte completo cada minuto
- Persiste en TimeSeries DB (InfluxDB o SQLite con timestamps)

Métodos:
- get_global_health() → GlobalHealth(score, components, anomalies, recommendations)
- start_monitoring() → loop asyncio que ejecuta cada 10s
- save_to_db(health_report) → persiste para trending
- get_historical_trend(days=7) → datos históricos

API Endpoint en PUSH (/api/health/comprehensive):
- GET /api/health/comprehensive → retorna GlobalHealth completo
- GET /api/health/metrics/system → SystemMetrics
- GET /api/health/metrics/services → ServiceHealth de todos
- GET /api/health/anomalies → anomalías detectadas últimas 24h
- GET /api/health/predictions → predicciones de fallos

═══════════════════════════════════════════════════════════════════════════
SUBSISTEMA 2: SELF-HEALING ENGINE - Auto-Corrección Inteligente
═══════════════════════════════════════════════════════════════════════════

Crear: **autonomous/self_healing/error_classifier.py**

Clase ErrorClassifier que categoriza errores:
- TRANSIENT, CONFIGURATION, RESOURCE, DEPENDENCY, FATAL

Métodos:
- classify_error(exception, context) → ErrorType, severity, recoverable
- get_recovery_strategy(error_type) → RecoveryStrategy
- learn_from_error(error, resolution) → actualiza knowledge base

Crear: **autonomous/self_healing/recovery_strategies.py**

Estrategias: RETRY, RESTART_SERVICE, RESTART_ALL, ROLLBACK, FALLBACK, DEGRADE, ISOLATE, ALERT_HUMAN.
Métodos por estrategia: retry_with_backoff, restart_service, rollback_to_version, activate_fallback, enter_degraded_mode, isolate_component.

Crear: **autonomous/self_healing/circuit_breaker.py**

Estados: CLOSED, OPEN, HALF_OPEN. Parámetros: failure_threshold, timeout, success_threshold.
Métodos: call, record_success, record_failure, get_state, reset. Decorador @circuit_breaker.

Crear: **autonomous/self_healing/failure_memory.py**

SQLite: error_signature, timestamp, context, recovery_used, success, notes.
Métodos: record_failure, get_similar_failures, suggest_recovery, get_recurring_errors, generate_preventive_actions.

Crear: **autonomous/self_healing/healing_orchestrator.py**

Flujo: detecta → clasifica → consulta FailureMemory → selecciona estrategia → ejecuta con CircuitBreaker → registra → notifica Telemetry.
Métodos: handle_error, start_healing_service, get_healing_stats.

API PUSH: POST /api/healing/trigger, GET /api/healing/history, GET /api/healing/strategies, POST /api/healing/override.

═══════════════════════════════════════════════════════════════════════════
SUBSISTEMA 3: EVOLUTION ENGINE 2.0 - Auto-Actualización Avanzada
═══════════════════════════════════════════════════════════════════════════

Crear: **autonomous/evolution/regression_tester.py** — run_full_test_suite, run_api_tests, run_performance_benchmark, compare_with_baseline.
Crear: **autonomous/evolution/backup_manager.py** — create_snapshot, list_snapshots, restore_snapshot, cleanup_old_snapshots, verify_snapshot.
Crear: **autonomous/evolution/staged_rollout.py** — start_rollout, advance_phase, rollback_phase, get_rollout_status (CANARY→BETA→STABLE→FULL).
Crear: **autonomous/evolution/metrics_comparator.py** — capture_baseline, capture_post_update, compare_metrics, should_rollback, generate_comparison_chart.
Crear: **autonomous/evolution/evolution_orchestrator_v2.py** — flujo: Scan→Backup→Regression Test→Sandbox→Baseline→Staged Rollout→Metrics Comparison→Auto-Rollback→Changelog→Notification.

API PUSH: POST /api/evolution/scan, POST /api/evolution/update, GET /api/evolution/status, POST /api/evolution/rollback, GET /api/evolution/history.

═══════════════════════════════════════════════════════════════════════════
SUBSISTEMA 4: TELEMETRY HUB - Observabilidad Total
═══════════════════════════════════════════════════════════════════════════

Crear: **autonomous/telemetry/metrics_aggregator.py** — collect_metric, get_metrics, aggregate, export_to_timeseries_db.
Crear: **autonomous/telemetry/logs_collector.py** — structured JSON logging, search_logs, aggregate_errors, export_logs.
Crear: **autonomous/telemetry/tracing_system.py** — start_trace, add_span, get_trace, analyze_slowest_traces.
Crear: **autonomous/telemetry/dashboard_engine.py** — dashboards: Health, Services, Neural Router, Tools, Self-Healing, Evolution.
Crear: **autonomous/telemetry/alert_manager.py** — reglas (threshold, rate of change, anomaly, compound), canales (bitácora, webhook, email), evaluate_rules, trigger_alert.

API PUSH: GET /api/telemetry/metrics, /logs, /traces, /dashboards, /dashboard/:name, /alerts, POST /api/telemetry/alerts/ack/:id.

═══════════════════════════════════════════════════════════════════════════
SUBSISTEMA 5: RESILIENCE LAYER - Redundancia y Resiliencia
═══════════════════════════════════════════════════════════════════════════

Crear: **autonomous/resilience/priority_queue.py** — CRITICAL/HIGH/MEDIUM/LOW, enqueue, dequeue, get_queue_stats, reorder_on_emergency.
Crear: **autonomous/resilience/resource_throttler.py** — límites (concurrent requests, CPU, RAM, GPU, rate), should_throttle, wait_for_resources, adjust_limits_dynamically.
Crear: **autonomous/resilience/survival_mode.py** — enter_survival_mode, exit, is_in_survival, get_disabled_features (mantiene solo core).
Crear: **autonomous/resilience/disaster_recovery.py** — create_full_backup, create_incremental_backup, restore_from_disaster, verify_backup_integrity, test_disaster_recovery.

API PUSH: GET /api/resilience/queue, /throttling, POST /api/resilience/survival/enter|exit, POST /api/resilience/backup, GET /api/resilience/backups.

═══════════════════════════════════════════════════════════════════════════
SUBSISTEMA 6: LEARNING ENGINE - Auto-Aprendizaje
═══════════════════════════════════════════════════════════════════════════

Crear: **autonomous/learning/pattern_analyzer.py** — analyze_usage_patterns, detect_anomalous_behavior, predict_next_action, generate_insights.
Crear: **autonomous/learning/performance_optimizer.py** — optimize_parameter, run_optimization_cycle, get_optimization_history, suggest_optimizations.
Crear: **autonomous/learning/route_optimizer.py** — analyze_model_performance, optimize_routing_rules, suggest_model_for_query, get_routing_stats (para Neural Router).
Crear: **autonomous/learning/feedback_loop.py** — record_feedback, analyze_feedback, identify_improvement_areas, generate_action_items.
Crear: **autonomous/learning/knowledge_graph.py** — add_entity, add_relation, query_graph, find_related, visualize_subgraph (memoria semántica).
Crear: **autonomous/learning/learning_orchestrator.py** — run_learning_cycle, start_continuous_learning, get_learning_insights, export_learned_knowledge.

API PUSH: GET /api/learning/patterns, /optimizations, /insights, POST /api/learning/feedback, GET /api/learning/knowledge-graph.

═══════════════════════════════════════════════════════════════════════════
INTEGRACIÓN CON SISTEMA EXISTENTE
═══════════════════════════════════════════════════════════════════════════

- **atlas_adapter/atlas_http_api.py**: imports de orquestadores, startup: inicializar orquestadores, background tasks, cargar config.
- **modules/nexus_heartbeat.py**: enviar métricas a HealthAggregator, usar CircuitBreaker en heartbeat, trigger healing si falla.
- **modules/humanoid/governance/state.py**: emergency_stop → SurvivalMode; growth → auto-optimizations; governed → approval para optimizations.

═══════════════════════════════════════════════════════════════════════════
DEPENDENCIAS
═══════════════════════════════════════════════════════════════════════════

psutil, GPUtil, scikit-learn, influxdb-client (opcional), py-cpuinfo, prometheus-client (opcional).

═══════════════════════════════════════════════════════════════════════════
CONFIGURACIÓN
═══════════════════════════════════════════════════════════════════════════

Crear **config/autonomous.yaml** con secciones: health_monitor, self_healing, evolution, telemetry, resilience, learning (thresholds, intervals, enabled flags).

═══════════════════════════════════════════════════════════════════════════
TESTING, DASHBOARD, REPORTE FINAL
═══════════════════════════════════════════════════════════════════════════

- Tests: tests/autonomous/test_*.py por subsistema.
- Dashboard: Health Score global, Service Status, Alerts, Healing Attempts, Optimizations, Learning Insights; páginas /dashboard/health, /telemetry, /learning.
- Reporte final: archivos creados, funcionalidades (checklist), integración, pruebas, métricas baseline, próximos pasos.

IMPLEMENTA TODO EL SISTEMA AUTONOMOUS COMPLETO. Código limpio, documentado, error handling robusto. NO atajos.
