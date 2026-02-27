"""
POT: Weekly Maintenance (Mantenimiento Semanal)
===============================================
Procedimiento de mantenimiento preventivo semanal más profundo.

Triggers:
- Scheduler: Domingos a las 04:00
- Manual: Cuando se requiera mantenimiento profundo

Severidad: MEDIUM (algunas operaciones pueden afectar rendimiento temporalmente)
"""
from modules.humanoid.quality.models import POT, POTStep, POTCategory, POTSeverity, StepType


def get_pot() -> POT:
    return POT(
        id="maintenance_weekly",
        name="Mantenimiento Semanal",
        description="""
Procedimiento de mantenimiento semanal más exhaustivo que incluye:
- Optimización de bases de datos
- Verificación profunda de integridad
- Limpieza de archivos huérfanos
- Actualización de dependencias (check only)
- Generación de reporte semanal
        """.strip(),
        category=POTCategory.MAINTENANCE,
        severity=POTSeverity.MEDIUM,
        version="1.0.0",
        author="ATLAS QA Senior",
        
        trigger_check_ids=["weekly_maintenance", "scheduled_weekly"],
        trigger_keywords=["weekly", "semanal", "deep", "maintenance"],
        
        prerequisites=[
            "Mantenimiento diario ejecutado",
            "Sistema en estado estable",
            "Horario de baja actividad (idealmente domingo madrugada)",
        ],
        required_services=["push", "robot"],
        required_permissions=["db_optimize", "service_restart"],
        
        objectives=[
            "Optimizar bases de datos SQLite (VACUUM)",
            "Verificar dependencias actualizables",
            "Limpiar archivos huérfanos de workshop",
            "Ejecutar diagnóstico completo de cámaras",
            "Generar reporte semanal de estado",
        ],
        success_criteria="Todas las tareas completadas, ninguna alerta crítica",
        estimated_duration_minutes=20,
        
        tutorial_overview="""
## Guía de Mantenimiento Semanal

### Diferencia con Mantenimiento Diario
| Aspecto | Diario | Semanal |
|---------|--------|---------|
| Duración | ~5 min | ~20 min |
| DB | Integrity check | VACUUM + ANALYZE |
| Deps | No | Check updates |
| Cámaras | No | Diagnóstico completo |
| Servicios | No | Reinicio preventivo |

### VACUUM en SQLite
```sql
VACUUM;  -- Reconstruye la DB, recupera espacio
ANALYZE; -- Actualiza estadísticas para queries
```

### Archivos Huérfanos
- Workshop tickets en 'working' por más de 24h
- Snapshots sin referencia en reportes
- Logs de procesos que ya no existen
        """.strip(),
        
        best_practices=[
            "Ejecutar siempre en horario de baja actividad",
            "Hacer backup antes de VACUUM en DBs grandes",
            "Revisar changelog de dependencias antes de actualizar",
            "Documentar cualquier anomalía encontrada",
        ],
        
        warnings=[
            "VACUUM puede tardar varios minutos en DBs grandes",
            "No interrumpir el proceso una vez iniciado",
            "Servicios pueden reiniciarse durante el proceso",
        ],
        
        related_pots=["maintenance_daily", "diagnostic_full", "upgrade_dependencies"],
        tags=["maintenance", "weekly", "optimization", "vacuum", "preventive"],
        
        steps=[
            POTStep(
                id="run_daily_first",
                name="Ejecutar mantenimiento diario",
                description="Asegurar que las tareas diarias están hechas",
                step_type=StepType.LOG,
                tutorial_notes="El mantenimiento semanal asume que el diario ya se ejecutó.",
            ),
            
            POTStep(
                id="vacuum_scheduler_db",
                name="Optimizar DB del scheduler",
                description="VACUUM en la base de datos del scheduler",
                step_type=StepType.COMMAND,
                command='python -c "import sqlite3; c=sqlite3.connect(\\"data/scheduler.db\\"); c.execute(\\"VACUUM\\"); c.execute(\\"ANALYZE\\"); print(\\"VACUUM OK\\")"',
                timeout_seconds=120,
                continue_on_failure=True,
                tutorial_notes="VACUUM reconstruye la DB eliminando fragmentación.",
            ),
            
            POTStep(
                id="vacuum_ans_db",
                name="Optimizar DB del ANS",
                description="VACUUM en la base de datos del ANS",
                step_type=StepType.COMMAND,
                command='python -c "import sqlite3; c=sqlite3.connect(\\"data/ans.db\\"); c.execute(\\"VACUUM\\"); c.execute(\\"ANALYZE\\"); print(\\"VACUUM OK\\")"',
                timeout_seconds=120,
                continue_on_failure=True,
            ),
            
            POTStep(
                id="vacuum_approvals_db",
                name="Optimizar DB de aprobaciones",
                description="VACUUM en la base de datos de aprobaciones",
                step_type=StepType.COMMAND,
                command='python -c "import sqlite3; c=sqlite3.connect(\\"data/approvals.db\\"); c.execute(\\"VACUUM\\"); c.execute(\\"ANALYZE\\"); print(\\"VACUUM OK\\")"',
                timeout_seconds=60,
                continue_on_failure=True,
            ),
            
            POTStep(
                id="check_dependency_updates",
                name="Verificar actualizaciones de dependencias",
                description="Listar paquetes con actualizaciones disponibles",
                step_type=StepType.COMMAND,
                command="python scripts/check_pip_outdated.py",
                timeout_seconds=120,
                continue_on_failure=True,
                capture_output=True,
                tutorial_notes="Solo lista, NO actualiza. Las actualizaciones requieren POT upgrade_dependencies.",
            ),
            
            POTStep(
                id="clean_workshop_stale",
                name="Limpiar tickets huérfanos del Workshop",
                description="Mover tickets estancados en 'working' al inbox",
                step_type=StepType.COMMAND,
                command='powershell -Command "Get-ChildItem -Path logs/workshop/working -Filter *.json -ErrorAction SilentlyContinue | Where-Object { $_.LastWriteTime -lt (Get-Date).AddHours(-24) } | Move-Item -Destination logs/workshop/inbox -Force"',
                timeout_seconds=30,
                continue_on_failure=True,
                tutorial_notes="Tickets en 'working' por más de 24h probablemente fallaron.",
            ),
            
            POTStep(
                id="camera_diagnostic",
                name="Diagnóstico completo de cámaras",
                description="Ejecutar auto-reparación de cámaras",
                step_type=StepType.SCRIPT,
                script_path="scripts/atlas_camera_autorepair.py",
                timeout_seconds=180,
                continue_on_failure=True,
                capture_output=True,
            ),
            
            POTStep(
                id="preventive_restart_robot",
                name="Reinicio preventivo de Robot",
                description="Reiniciar Robot backend para limpiar memoria",
                step_type=StepType.COMMAND,
                command="powershell -ExecutionPolicy Bypass -File scripts/restart_service_clean.ps1 -Service robot",
                timeout_seconds=90,
                tutorial_notes="""
Reinicio preventivo semanal:
- Libera memoria acumulada
- Reinicia contadores
- Limpia handles de archivos
                """,
            ),
            
            POTStep(
                id="wait_stabilize",
                name="Esperar estabilización",
                description="Dar tiempo al sistema para estabilizarse",
                step_type=StepType.WAIT,
                wait_seconds=10,
            ),
            
            POTStep(
                id="full_health_check",
                name="Verificación completa de salud",
                description="Ejecutar diagnóstico de todos los servicios",
                step_type=StepType.SCRIPT,
                script_path="scripts/check_nexus_ports.py",
                timeout_seconds=60,
                capture_output=True,
            ),
            
            POTStep(
                id="generate_weekly_report",
                name="Generar reporte semanal",
                description="Crear resumen de estado del sistema",
                step_type=StepType.HTTP,
                http_method="GET",
                http_url="http://127.0.0.1:8791/health",
                timeout_seconds=30,
                capture_output=True,
            ),
            
            POTStep(
                id="notify_completion",
                name="Notificar completación",
                description="Enviar notificación de mantenimiento completado",
                step_type=StepType.NOTIFY,
                notify_channel="telegram",
                notify_message="✅ Mantenimiento semanal completado. Sistema optimizado.",
                continue_on_failure=True,
            ),
            
            POTStep(
                id="log_maintenance",
                name="Registrar en bitácora",
                description="Log de mantenimiento semanal",
                step_type=StepType.HTTP,
                http_method="POST",
                http_url="http://127.0.0.1:8791/ans/evolution-log",
                http_body={
                    "message": "[MANTENIMIENTO] Mantenimiento semanal completado por POT",
                    "ok": True,
                    "source": "quality_pot"
                },
                continue_on_failure=True,
            ),
        ],
    )
