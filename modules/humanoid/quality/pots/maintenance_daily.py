"""
POT: Daily Maintenance (Mantenimiento Diario)
=============================================
Procedimiento de mantenimiento preventivo diario.

Triggers:
- Scheduler: Diario a las 03:00 (madrugada)
- Manual: Cuando se requiera limpieza general

Severidad: LOW (operaciones de rutina)
"""
from modules.humanoid.quality.models import POT, POTStep, POTCategory, POTSeverity, StepType


def get_pot() -> POT:
    return POT(
        id="maintenance_daily",
        name="Mantenimiento Diario",
        description="""
Procedimiento de mantenimiento preventivo que debe ejecutarse diariamente.
Incluye limpieza de logs, verificación de espacio en disco, rotación de
archivos temporales y verificación de salud general.
        """.strip(),
        category=POTCategory.MAINTENANCE,
        severity=POTSeverity.LOW,
        version="1.0.0",
        author="ATLAS QA Senior",
        
        trigger_check_ids=["scheduled_maintenance", "daily_check"],
        trigger_keywords=["maintenance", "daily", "cleanup", "routine"],
        
        prerequisites=[
            "Sistema en estado estable (no en medio de operación crítica)",
            "Preferiblemente ejecutar en horario de baja actividad",
        ],
        required_services=["push"],
        
        objectives=[
            "Limpiar archivos temporales y logs antiguos",
            "Verificar espacio en disco",
            "Rotar logs si exceden tamaño",
            "Verificar integridad de base de datos",
            "Generar reporte de estado",
        ],
        success_criteria="Todas las tareas de mantenimiento completadas sin error",
        estimated_duration_minutes=10,
        
        tutorial_overview="""
## Guía de Mantenimiento Diario

### Tareas de Mantenimiento
1. **Limpieza de Temporales**: Eliminar archivos .tmp, .log antiguos
2. **Rotación de Logs**: Si algún log > 50MB, rotar
3. **Espacio en Disco**: Alertar si < 10% libre
4. **DB Integrity**: Verificar SQLite databases
5. **Health Report**: Generar snapshot de estado

### Horario Recomendado
- **Producción**: 03:00-04:00 (madrugada)
- **Desarrollo**: Cualquier momento de baja actividad

### Archivos a Limpiar
```
logs/*.log (> 7 días)
__pycache__/ (completo)
.pytest_cache/ (completo)
snapshots/temp/ (> 3 días)
```
        """.strip(),
        
        best_practices=[
            "No ejecutar durante operaciones críticas",
            "Verificar backup antes de eliminar logs importantes",
            "Mantener al menos 7 días de logs para debugging",
            "Monitorear espacio en disco post-limpieza",
        ],
        
        warnings=[
            "No eliminar logs del día actual",
            "Preservar logs de errores críticos",
        ],
        
        related_pots=["maintenance_weekly", "disk_cleanup", "log_rotation"],
        tags=["maintenance", "daily", "cleanup", "logs", "preventive"],
        
        steps=[
            POTStep(
                id="check_system_idle",
                name="Verificar sistema estable",
                description="Confirmar que no hay operaciones críticas en curso",
                step_type=StepType.HTTP,
                http_method="GET",
                http_url="http://127.0.0.1:8791/health",
                timeout_seconds=10,
                check_expression="response.get('score', 0) >= 70",
                tutorial_notes="Solo proceder si el health score es >= 70",
            ),
            
            POTStep(
                id="clean_pycache",
                name="Limpiar __pycache__",
                description="Eliminar directorios de cache de Python",
                step_type=StepType.COMMAND,
                command='powershell -Command "Get-ChildItem -Path . -Directory -Recurse -Filter __pycache__ | Remove-Item -Recurse -Force -ErrorAction SilentlyContinue"',
                timeout_seconds=60,
                continue_on_failure=True,
                tutorial_notes="__pycache__ se regenera automáticamente. Seguro eliminar.",
            ),
            
            POTStep(
                id="clean_pytest_cache",
                name="Limpiar .pytest_cache",
                description="Eliminar cache de pytest",
                step_type=StepType.COMMAND,
                command='powershell -Command "Get-ChildItem -Path . -Directory -Recurse -Filter .pytest_cache | Remove-Item -Recurse -Force -ErrorAction SilentlyContinue"',
                timeout_seconds=30,
                continue_on_failure=True,
            ),
            
            POTStep(
                id="clean_old_logs",
                name="Limpiar logs antiguos",
                description="Eliminar logs de más de 7 días",
                step_type=StepType.COMMAND,
                command='powershell -Command "Get-ChildItem -Path logs -Filter *.log -Recurse -ErrorAction SilentlyContinue | Where-Object { $_.LastWriteTime -lt (Get-Date).AddDays(-7) } | Remove-Item -Force -ErrorAction SilentlyContinue"',
                timeout_seconds=60,
                continue_on_failure=True,
                tutorial_notes="Mantener 7 días de logs para debugging de problemas recientes.",
            ),
            
            POTStep(
                id="clean_temp_snapshots",
                name="Limpiar snapshots temporales",
                description="Eliminar snapshots temp de más de 3 días",
                step_type=StepType.COMMAND,
                command='powershell -Command "Get-ChildItem -Path snapshots -Recurse -ErrorAction SilentlyContinue | Where-Object { -not $_.PSIsContainer -and $_.LastWriteTime -lt (Get-Date).AddDays(-3) } | Remove-Item -Force -ErrorAction SilentlyContinue"',
                timeout_seconds=60,
                continue_on_failure=True,
            ),
            
            POTStep(
                id="check_disk_space",
                name="Verificar espacio en disco",
                description="Alertar si hay menos de 10% libre",
                step_type=StepType.COMMAND,
                command='powershell -Command "$disk = Get-PSDrive C; $pctFree = [math]::Round(($disk.Free / ($disk.Used + $disk.Free)) * 100, 2); Write-Output \\"DISK_FREE_PCT: $pctFree\\"; if ($pctFree -lt 10) { Write-Output \\"WARNING: Low disk space!\\" }"',
                timeout_seconds=15,
                capture_output=True,
                tutorial_notes="Si el disco tiene menos del 10% libre, considerar limpieza urgente.",
            ),
            
            POTStep(
                id="verify_sqlite_dbs",
                name="Verificar integridad SQLite",
                description="Ejecutar integrity_check en bases de datos",
                step_type=StepType.COMMAND,
                command="python scripts/check_sqlite_integrity.py",
                timeout_seconds=60,
                continue_on_failure=True,
                capture_output=True,
            ),
            
            POTStep(
                id="generate_health_snapshot",
                name="Generar snapshot de salud",
                description="Capturar estado actual del sistema",
                step_type=StepType.HTTP,
                http_method="GET",
                http_url="http://127.0.0.1:8791/health",
                timeout_seconds=30,
                capture_output=True,
                tutorial_notes="Este snapshot se guarda para comparar evolución día a día.",
            ),
            
            POTStep(
                id="log_maintenance",
                name="Registrar mantenimiento",
                description="Log en bitácora de evolución",
                step_type=StepType.HTTP,
                http_method="POST",
                http_url="http://127.0.0.1:8791/ans/evolution-log",
                http_body={
                    "message": "[MANTENIMIENTO] Mantenimiento diario completado por POT",
                    "ok": True,
                    "source": "quality_pot"
                },
                continue_on_failure=True,
            ),
        ],
    )
