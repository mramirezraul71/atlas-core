"""
POT: Incident Response (Respuesta a Incidentes)
================================================
Procedimiento estándar para responder a incidentes activos.

Triggers:
- Incidente clasificado como HIGH o CRITICAL
- Activación manual por operador

Severidad: HIGH (es respuesta a una situación activa)
"""
from modules.humanoid.quality.models import (POT, POTCategory, POTSeverity,
                                             POTStep, StepType)


def get_pot() -> POT:
    return POT(
        id="incident_response",
        name="Respuesta a Incidentes",
        description="""
Procedimiento estándar de respuesta a incidentes activos.
Sigue el ciclo: Detectar → Contener → Investigar → Reparar → Verificar → Documentar.
        """.strip(),
        category=POTCategory.INCIDENT,
        severity=POTSeverity.HIGH,
        version="1.0.0",
        author="ATLAS QA Senior",
        trigger_check_ids=["incident_active", "alert_*"],
        trigger_keywords=["incident", "alert", "emergency", "response", "outage"],
        prerequisites=[
            "Incidente identificado y clasificado",
            "Acceso a logs del sistema",
            "Permisos de reinicio de servicios",
        ],
        required_services=[],  # Puede que los servicios estén caídos
        required_permissions=["service_restart", "log_access", "notify"],
        objectives=[
            "Contener el impacto del incidente",
            "Identificar causa raíz",
            "Aplicar fix o workaround",
            "Verificar resolución",
            "Documentar para prevención futura",
        ],
        success_criteria="Sistema restaurado a operación normal, incidente cerrado",
        estimated_duration_minutes=15,
        tutorial_overview="""
## Guía de Respuesta a Incidentes

### Ciclo de Respuesta (CDIRV)
```
┌─────────┐    ┌──────────┐    ┌─────────────┐
│ Detectar│───▶│ Contener │───▶│ Investigar  │
└─────────┘    └──────────┘    └──────┬──────┘
                                      │
┌─────────────┐    ┌──────────┐       │
│ Documentar  │◀───│ Verificar │◀─────┘
└─────────────┘    └──────────┘
                        ▲
                        │
                   ┌────┴────┐
                   │ Reparar │
                   └─────────┘
```

### Tiempos Objetivo (SLA)
| Severidad | Detección | Contención | Resolución |
|-----------|-----------|------------|------------|
| CRITICAL  | 1 min     | 5 min      | 30 min     |
| HIGH      | 5 min     | 15 min     | 1 hora     |
| MEDIUM    | 15 min    | 30 min     | 4 horas    |
| LOW       | 1 hora    | 4 horas    | 24 horas   |

### Contención vs Reparación
- **Contención**: Detener la hemorragia (reinicio rápido, disable feature)
- **Reparación**: Arreglar la causa raíz (fix de código, config)
        """.strip(),
        best_practices=[
            "Contener primero, investigar después",
            "Comunicar estado cada 15 minutos durante incidente",
            "No hacer cambios no relacionados durante el incidente",
            "Documentar TODOS los cambios realizados",
            "Crear ticket de seguimiento si la causa raíz no es obvia",
        ],
        warnings=[
            "No apagar servicios sin plan de recuperación",
            "Preservar logs antes de reiniciar (pueden perderse)",
            "Coordinar con otros operadores si hay múltiples personas",
        ],
        related_pots=["incident_triage", "incident_postmortem", "services_repair"],
        tags=["incident", "response", "emergency", "sla"],
        has_rollback=True,
        steps=[
            # Fase 1: Detección (ya hecha por ANS, pero confirmamos)
            POTStep(
                id="confirm_incident",
                name="Confirmar incidente",
                description="Verificar que el incidente está activo",
                step_type=StepType.HTTP,
                http_method="GET",
                http_url="http://127.0.0.1:8791/ans/incidents?status=open&limit=1",
                timeout_seconds=10,
                continue_on_failure=True,
                tutorial_notes="Confirmamos que hay un incidente activo que atender.",
            ),
            # Fase 2: Contención
            POTStep(
                id="notify_incident_start",
                name="Notificar inicio de respuesta",
                description="Avisar que se está atendiendo el incidente",
                step_type=StepType.NOTIFY,
                notify_channel="telegram",
                notify_message="🚨 Incidente detectado. Iniciando respuesta automática...",
                continue_on_failure=True,
            ),
            POTStep(
                id="capture_initial_state",
                name="Capturar estado inicial",
                description="Snapshot del sistema antes de intervenir",
                step_type=StepType.SNAPSHOT,
                capture_screenshot=True,
                tutorial_notes="""
IMPORTANTE: Capturar estado ANTES de hacer cambios.
Esto es crucial para:
1. Diagnóstico posterior
2. Comparación pre/post
3. Evidencia de auditoría
                """,
            ),
            POTStep(
                id="quick_health_check",
                name="Diagnóstico rápido de servicios",
                description="Verificar qué servicios responden",
                step_type=StepType.SCRIPT,
                script_path="scripts/check_nexus_ports.py",
                timeout_seconds=30,
                capture_output=True,
            ),
            # Fase 3: Investigación rápida
            POTStep(
                id="check_recent_logs",
                name="Revisar logs recientes",
                description="Buscar errores en los últimos 5 minutos",
                step_type=StepType.COMMAND,
                command='powershell -Command "Get-ChildItem logs -Filter *.log -Recurse | ForEach-Object { Get-Content $_.FullName -Tail 50 | Select-String -Pattern \\"ERROR|CRITICAL|Exception\\" } | Select-Object -First 20"',
                timeout_seconds=30,
                capture_output=True,
                continue_on_failure=True,
                tutorial_notes="Buscamos patrones de error para identificar causa raíz.",
            ),
            # Fase 4: Reparación
            POTStep(
                id="apply_standard_fix",
                name="Aplicar fix estándar",
                description="Ejecutar reparación según el tipo de incidente",
                step_type=StepType.LOG,
                tutorial_notes="""
Aquí el motor de POTs selecciona el POT de reparación apropiado:
- camera_* → POT camera_repair
- service_* → POT services_repair
- api_* → POT api_repair
El POT específico se ejecuta como sub-procedimiento.
                """,
            ),
            POTStep(
                id="restart_affected_service",
                name="Reiniciar servicio afectado",
                description="Reinicio limpio del servicio problemático",
                step_type=StepType.COMMAND,
                command="powershell -ExecutionPolicy Bypass -File scripts/restart_service_clean.ps1 -Service robot",
                timeout_seconds=90,
                tutorial_notes="Por defecto reiniciamos Robot. Ajustar según diagnóstico.",
            ),
            POTStep(
                id="wait_stabilization",
                name="Esperar estabilización",
                description="Dar tiempo al sistema para recuperarse",
                step_type=StepType.WAIT,
                wait_seconds=15,
            ),
            # Fase 5: Verificación
            POTStep(
                id="verify_resolution",
                name="Verificar resolución",
                description="Confirmar que el problema está resuelto",
                step_type=StepType.HTTP,
                http_method="GET",
                http_url="http://127.0.0.1:8791/health",
                timeout_seconds=15,
                check_expression="response.get('score', 0) >= 80",
                retries=3,
                retry_delay_seconds=5,
            ),
            POTStep(
                id="run_ans_verify",
                name="Ejecutar verificación ANS",
                description="Confirmar que el check ya no falla",
                step_type=StepType.HTTP,
                http_method="POST",
                http_url="http://127.0.0.1:8791/ans/run-now",
                http_body={"mode": "auto"},
                timeout_seconds=60,
            ),
            # Fase 6: Documentación
            POTStep(
                id="resolve_incident_record",
                name="Marcar incidente como resuelto",
                description="Actualizar estado en el sistema",
                step_type=StepType.HTTP,
                http_method="POST",
                http_url="http://127.0.0.1:8791/ans/resolve-incidents",
                timeout_seconds=10,
                continue_on_failure=True,
            ),
            POTStep(
                id="log_resolution",
                name="Registrar resolución",
                description="Documentar en bitácora",
                step_type=StepType.HTTP,
                http_method="POST",
                http_url="http://127.0.0.1:8791/ans/evolution-log",
                http_body={
                    "message": "[INCIDENTE] Incidente resuelto por POT incident_response",
                    "ok": True,
                    "source": "quality_pot",
                },
            ),
            POTStep(
                id="notify_resolution",
                name="Notificar resolución",
                description="Avisar que el incidente fue resuelto",
                step_type=StepType.NOTIFY,
                notify_channel="telegram",
                notify_message="✅ Incidente resuelto. Sistema operando normalmente.",
            ),
        ],
        rollback_steps=[
            POTStep(
                id="rollback_revert_changes",
                name="Revertir cambios",
                description="Deshacer cambios si la reparación empeoró las cosas",
                step_type=StepType.LOG,
                tutorial_notes="En caso de rollback, restaurar último estado conocido bueno.",
            ),
            POTStep(
                id="rollback_notify",
                name="Notificar rollback",
                description="Alertar que se necesita intervención manual",
                step_type=StepType.NOTIFY,
                notify_channel="telegram",
                notify_message="⚠️ ROLLBACK ejecutado. Incidente requiere intervención manual.",
            ),
        ],
    )
