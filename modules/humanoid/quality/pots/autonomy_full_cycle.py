"""
POT: Autonomy Full Cycle (Ciclo de Autonomía Completo)
======================================================
Procedimiento maestro que ejecuta un ciclo completo de autonomía ATLAS.

Integra todos los subsistemas autónomos:
1. Health Monitor - Auto-inspección
2. Self Healing - Auto-corrección
3. Learning - Auto-aprendizaje
4. Evolution - Auto-actualización
5. Telemetry - Registro de métricas
6. Sync - Sincronización con remoto

Este POT es el "latido" de la autonomía de ATLAS.

Triggers:
- Scheduler (cada X minutos)
- Comando manual "autonomy cycle"
- Después de detectar degradación

Severidad: MEDIUM (modifica sistema si hay healing)
"""
from modules.humanoid.quality.models import POT, POTStep, POTCategory, POTSeverity, StepType


def get_pot() -> POT:
    return POT(
        id="autonomy_full_cycle",
        name="Ciclo de Autonomía Completo",
        description="""
Procedimiento maestro que ejecuta un ciclo completo de autonomía ATLAS.

El ciclo incluye:
1. AUTO-INSPECCIÓN: Verificar salud del sistema (CPU, RAM, servicios)
2. AUTO-CORRECCIÓN: Si hay problemas, intentar healing automático
3. AUTO-APRENDIZAJE: Analizar patrones y generar insights
4. AUTO-EVOLUCIÓN: Verificar si hay actualizaciones pendientes
5. TELEMETRÍA: Registrar métricas del ciclo
6. SINCRONIZACIÓN: Commit y push si hay cambios
7. NOTIFICACIÓN: Informar estado a canales

Este es el "latido autónomo" de ATLAS - el procedimiento que mantiene
al sistema vivo, aprendiendo y mejorando continuamente.
        """.strip(),
        category=POTCategory.DIAGNOSTIC,
        severity=POTSeverity.MEDIUM,
        version="1.0.0",
        author="ATLAS QA Senior",
        
        trigger_check_ids=["autonomy_*", "cycle_*", "heartbeat_*"],
        trigger_keywords=["autonomy", "autonomous", "cycle", "heartbeat", "latido", "auto"],
        
        prerequisites=[
            "Servicios básicos funcionando (Push, Robot)",
            "Módulo autonomous disponible",
            "Configuración autonomous.yaml presente",
        ],
        required_services=["push"],
        required_permissions=["health_read", "healing_execute", "learning_read", "git_write"],
        
        objectives=[
            "Ejecutar auto-inspección completa",
            "Detectar y corregir anomalías",
            "Actualizar knowledge graph con patrones",
            "Verificar estado de evolución",
            "Sincronizar con repositorio si hay cambios",
            "Notificar resumen a canales",
        ],
        success_criteria="Ciclo completado con health score >= 70 y sin errores críticos",
        estimated_duration_minutes=5,
        
        tutorial_overview="""
## Guía del Ciclo de Autonomía Completo

### Arquitectura de Autonomía ATLAS
```
┌─────────────────────────────────────────────────────────────────────┐
│                    ATLAS AUTONOMY FULL CYCLE                         │
├─────────────────────────────────────────────────────────────────────┤
│                                                                      │
│  ┌──────────────┐     ┌──────────────┐     ┌──────────────┐        │
│  │   HEALTH     │────▶│   HEALING    │────▶│  LEARNING    │        │
│  │   MONITOR    │     │  (if needed) │     │   CYCLE      │        │
│  │              │     │              │     │              │        │
│  │ • CPU/RAM    │     │ • Classify   │     │ • Patterns   │        │
│  │ • Services   │     │ • Strategy   │     │ • Insights   │        │
│  │ • Anomalies  │     │ • Execute    │     │ • Optimize   │        │
│  └──────────────┘     └──────────────┘     └──────────────┘        │
│         │                    │                    │                 │
│         └────────────────────┼────────────────────┘                 │
│                              ▼                                      │
│  ┌──────────────┐     ┌──────────────┐     ┌──────────────┐        │
│  │  EVOLUTION   │────▶│  TELEMETRY   │────▶│    SYNC      │        │
│  │   CHECK      │     │   RECORD     │     │  GIT/NOTIFY  │        │
│  │              │     │              │     │              │        │
│  │ • Updates?   │     │ • Metrics    │     │ • Commit     │        │
│  │ • Rollout    │     │ • Dashboard  │     │ • Push       │        │
│  │ • Rollback   │     │ • Alerts     │     │ • Telegram   │        │
│  └──────────────┘     └──────────────┘     └──────────────┘        │
│                                                                      │
└─────────────────────────────────────────────────────────────────────┘
```

### Subsistemas Autónomos

| Subsistema | Función | Endpoints |
|------------|---------|-----------|
| Health Monitor | Auto-inspección | `/api/health/comprehensive` |
| Self Healing | Auto-corrección | `/api/healing/trigger` |
| Learning | Auto-aprendizaje | `/api/learning/insights` |
| Evolution | Auto-actualización | `/api/evolution/status` |
| Telemetry | Observabilidad | `/api/telemetry/metrics` |
| Resilience | Supervivencia | `/api/resilience/survival` |

### Flujo de Decisiones
```
health_score >= 80  →  Solo learning + telemetry
health_score 50-79  →  Healing suave + learning
health_score < 50   →  Healing agresivo + alerta
survival_mode       →  Solo operaciones críticas
```

### Frecuencia Recomendada
- **Normal**: Cada 5-10 minutos
- **Desarrollo**: Cada 30 minutos
- **Producción crítica**: Cada 2-3 minutos
        """.strip(),
        
        best_practices=[
            "Ejecutar regularmente via scheduler",
            "Monitorear health_score trends",
            "Revisar learning insights periódicamente",
            "No ejecutar durante deploys activos",
            "Permitir healing automático solo en modo growth",
        ],
        
        warnings=[
            "Healing puede reiniciar servicios",
            "Evolution puede aplicar actualizaciones",
            "Verificar modo governance antes de ejecutar",
        ],
        
        related_pots=[
            "diagnostic_full",
            "services_repair",
            "maintenance_daily",
            "session_startup",
            "git_push",
        ],
        tags=["autonomy", "cycle", "health", "healing", "learning", "evolution", "master"],
        has_rollback=False,  # El ciclo es mayormente read-only, healing tiene su propio rollback
        
        steps=[
            # ================================================================
            # FASE 1: AUTO-INSPECCIÓN (Health Monitor)
            # ================================================================
            POTStep(
                id="check_survival_mode",
                name="Verificar modo supervivencia",
                description="Si estamos en survival mode, saltar pasos no críticos",
                step_type=StepType.HTTP,
                http_method="GET",
                http_url="http://127.0.0.1:8791/api/resilience/survival",
                timeout_seconds=10,
                capture_output=True,
                continue_on_failure=True,
                tutorial_notes="survival_mode activo = solo operaciones críticas",
            ),
            
            POTStep(
                id="health_comprehensive",
                name="Obtener salud comprehensiva",
                description="Health score global incluyendo CPU, RAM, servicios, anomalías",
                step_type=StepType.HTTP,
                http_method="GET",
                http_url="http://127.0.0.1:8791/api/health/comprehensive",
                timeout_seconds=30,
                capture_output=True,
                continue_on_failure=True,
                tutorial_notes="""
Respuesta incluye:
- score: 0-100 (salud global)
- system_score: CPU/RAM/Disk
- services_score: Push/Robot/NEXUS
- anomalies: lista de anomalías detectadas
- recommendations: acciones sugeridas
                """,
            ),
            
            POTStep(
                id="health_system_metrics",
                name="Métricas de sistema",
                description="CPU, RAM, disco actuales",
                step_type=StepType.HTTP,
                http_method="GET",
                http_url="http://127.0.0.1:8791/api/health/metrics/system",
                timeout_seconds=15,
                capture_output=True,
                continue_on_failure=True,
            ),
            
            POTStep(
                id="health_services",
                name="Estado de servicios",
                description="Push, Robot, NEXUS online/offline",
                step_type=StepType.HTTP,
                http_method="GET",
                http_url="http://127.0.0.1:8791/api/health/metrics/services",
                timeout_seconds=15,
                capture_output=True,
                continue_on_failure=True,
            ),
            
            POTStep(
                id="check_anomalies",
                name="Detectar anomalías",
                description="Anomalías en las últimas 24h",
                step_type=StepType.HTTP,
                http_method="GET",
                http_url="http://127.0.0.1:8791/api/health/anomalies",
                timeout_seconds=15,
                capture_output=True,
                continue_on_failure=True,
            ),
            
            POTStep(
                id="log_health_status",
                name="Log estado de salud",
                description="Registrar resultado de auto-inspección",
                step_type=StepType.LOG,
                tutorial_notes="El health_score se usa para decidir si hacer healing",
            ),
            
            # ================================================================
            # FASE 2: AUTO-CORRECCIÓN (Self Healing)
            # ================================================================
            POTStep(
                id="check_healing_needed",
                name="Evaluar necesidad de healing",
                description="Si health_score < 70 o hay anomalías, intentar healing",
                step_type=StepType.CHECK,
                check_expression="context.get('health_comprehensive_ok', True)",
                capture_output=True,
                tutorial_notes="Si el health check falló, necesitamos healing",
            ),
            
            POTStep(
                id="healing_stats",
                name="Obtener estadísticas de healing",
                description="Ver estado del circuit breaker y historial",
                step_type=StepType.HTTP,
                http_method="GET",
                http_url="http://127.0.0.1:8791/api/healing/stats",
                timeout_seconds=15,
                capture_output=True,
                continue_on_failure=True,
            ),
            
            POTStep(
                id="check_ans_incidents",
                name="Verificar incidentes ANS",
                description="Ver si hay incidentes abiertos que requieran healing",
                step_type=StepType.HTTP,
                http_method="GET",
                http_url="http://127.0.0.1:8791/ans/incidents?status=open",
                timeout_seconds=15,
                capture_output=True,
                continue_on_failure=True,
            ),
            
            POTStep(
                id="trigger_ans_cycle",
                name="Ejecutar ciclo ANS",
                description="Disparar ciclo de checks y heals del sistema nervioso",
                step_type=StepType.HTTP,
                http_method="POST",
                http_url="http://127.0.0.1:8791/ans/cycle",
                timeout_seconds=60,
                capture_output=True,
                continue_on_failure=True,
                tutorial_notes="ANS ejecuta sus propios checks y heals internos",
            ),
            
            # ================================================================
            # FASE 3: AUTO-APRENDIZAJE (Learning)
            # ================================================================
            POTStep(
                id="learning_patterns",
                name="Analizar patrones de uso",
                description="Detectar patrones en comportamiento del sistema",
                step_type=StepType.HTTP,
                http_method="GET",
                http_url="http://127.0.0.1:8791/api/learning/patterns",
                timeout_seconds=30,
                capture_output=True,
                continue_on_failure=True,
            ),
            
            POTStep(
                id="learning_insights",
                name="Obtener insights de aprendizaje",
                description="Insights generados por el motor de aprendizaje",
                step_type=StepType.HTTP,
                http_method="GET",
                http_url="http://127.0.0.1:8791/api/learning/insights",
                timeout_seconds=30,
                capture_output=True,
                continue_on_failure=True,
                tutorial_notes="Insights incluyen optimizaciones sugeridas",
            ),
            
            # ================================================================
            # FASE 4: AUTO-EVOLUCIÓN (Evolution Check)
            # ================================================================
            POTStep(
                id="evolution_status",
                name="Estado de evolución",
                description="Verificar si hay actualizaciones en progreso o pendientes",
                step_type=StepType.HTTP,
                http_method="GET",
                http_url="http://127.0.0.1:8791/api/evolution/status",
                timeout_seconds=15,
                capture_output=True,
                continue_on_failure=True,
            ),
            
            POTStep(
                id="check_git_status",
                name="Verificar estado Git",
                description="Ver si hay cambios pendientes de sync",
                step_type=StepType.COMMAND,
                command="git status --porcelain",
                timeout_seconds=15,
                capture_output=True,
                continue_on_failure=True,
            ),
            
            POTStep(
                id="check_git_behind",
                name="Verificar commits remotos",
                description="Ver si estamos detrás del remoto",
                step_type=StepType.COMMAND,
                command="git fetch origin && git status -sb",
                timeout_seconds=60,
                capture_output=True,
                continue_on_failure=True,
            ),
            
            # ================================================================
            # FASE 5: TELEMETRÍA (Record Metrics)
            # ================================================================
            POTStep(
                id="telemetry_record",
                name="Registrar métricas del ciclo",
                description="Guardar métricas de este ciclo de autonomía",
                step_type=StepType.HTTP,
                http_method="GET",
                http_url="http://127.0.0.1:8791/api/telemetry/metrics?source=autonomy&time_range=300",
                timeout_seconds=15,
                capture_output=True,
                continue_on_failure=True,
            ),
            
            POTStep(
                id="check_resilience_queue",
                name="Verificar cola de resiliencia",
                description="Ver estado de la cola de operaciones pendientes",
                step_type=StepType.HTTP,
                http_method="GET",
                http_url="http://127.0.0.1:8791/api/resilience/queue",
                timeout_seconds=10,
                capture_output=True,
                continue_on_failure=True,
            ),
            
            POTStep(
                id="check_throttling",
                name="Verificar throttling",
                description="Ver si hay limitación de recursos activa",
                step_type=StepType.HTTP,
                http_method="GET",
                http_url="http://127.0.0.1:8791/api/resilience/throttling",
                timeout_seconds=10,
                capture_output=True,
                continue_on_failure=True,
            ),
            
            # ================================================================
            # FASE 6: REGISTRO EN CEREBRO
            # ================================================================
            POTStep(
                id="log_to_bitacora",
                name="Registrar ciclo en bitácora",
                description="Log del ciclo de autonomía en ANS",
                step_type=StepType.HTTP,
                http_method="POST",
                http_url="http://127.0.0.1:8791/ans/evolution-log",
                http_body={
                    "message": "[AUTONOMY] Ciclo de autonomía completo ejecutado",
                    "ok": True,
                    "source": "quality_pot.autonomy_full_cycle"
                },
                continue_on_failure=True,
            ),
            
            POTStep(
                id="refresh_dashboard",
                name="Refrescar dashboard",
                description="Actualizar información visual del dashboard",
                step_type=StepType.HTTP,
                http_method="GET",
                http_url="http://127.0.0.1:8791/status",
                timeout_seconds=15,
                continue_on_failure=True,
            ),
            
            # ================================================================
            # FASE 7: NOTIFICACIÓN
            # ================================================================
            POTStep(
                id="notify_cycle_complete",
                name="Notificar ciclo completado",
                description="Enviar resumen a canal OPS",
                step_type=StepType.NOTIFY,
                notify_channel="ops",
                notify_message="🔄 Ciclo de autonomía completado",
                continue_on_failure=True,
            ),
        ],
    )
