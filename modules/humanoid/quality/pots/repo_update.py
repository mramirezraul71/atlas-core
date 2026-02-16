"""
POT: Repository Update (Actualización Completa del Repositorio)
================================================================
Procedimiento completo para actualizar el repositorio y dependencias.

Triggers:
- Actualización programada
- Después de pull con cambios
- Mantenimiento del sistema

Severidad: MEDIUM
"""
from modules.humanoid.quality.models import POT, POTStep, POTCategory, POTSeverity, StepType


def get_pot() -> POT:
    return POT(
        id="repo_update",
        name="Actualización Completa del Repositorio",
        description="""
Procedimiento integral para actualizar el repositorio, incluyendo:
- Pull de cambios remotos
- Actualización de submodules
- Instalación de dependencias nuevas
- Verificación de integridad
- Reinicio de servicios si necesario
        """.strip(),
        category=POTCategory.UPGRADE,
        severity=POTSeverity.MEDIUM,
        version="1.0.0",
        author="ATLAS QA Senior",
        
        trigger_check_ids=["repo_outdated", "update_available", "deps_*"],
        trigger_keywords=["update", "upgrade", "actualizar", "dependencies", "deps"],
        
        prerequisites=[
            "Repositorio Git configurado",
            "Acceso de red",
            "pip/package manager disponible",
        ],
        required_services=["push"],
        required_permissions=["git_write", "pip_install", "service_restart"],
        
        objectives=[
            "Sincronizar con repositorio remoto",
            "Actualizar submodules si existen",
            "Instalar/actualizar dependencias Python",
            "Verificar que el sistema funciona",
            "Notificar completación",
        ],
        success_criteria="Repositorio actualizado, dependencias instaladas, servicios funcionando",
        estimated_duration_minutes=10,
        
        tutorial_overview="""
## Guía de Actualización del Repositorio

### Flujo Completo de Actualización
```
┌──────────┐     ┌──────────┐     ┌──────────┐     ┌──────────┐
│   Pull   │────▶│Submodules│────▶│   Pip    │────▶│  Verify  │
│          │     │  Update  │     │ Install  │     │ & Notify │
└──────────┘     └──────────┘     └──────────┘     └──────────┘
```

### Submodules
Si el repo tiene submodules (carpeta nexus/ por ejemplo):
```bash
git submodule update --init --recursive
```

### Dependencias
```bash
pip install -r requirements.txt --upgrade
```

### Verificación Post-Update
1. Comprobar syntax de Python
2. Verificar servicios responden
3. Ejecutar health check
        """.strip(),
        
        best_practices=[
            "Hacer backup antes de actualización mayor",
            "Verificar changelog antes de actualizar",
            "Ejecutar tests después de actualizar",
            "Reiniciar servicios solo si necesario",
        ],
        
        warnings=[
            "Actualizaciones pueden romper compatibilidad",
            "Verificar requirements.txt antes de pip install",
            "Tener plan de rollback si algo falla",
        ],
        
        related_pots=["git_pull", "git_push", "dependency_update", "services_repair"],
        tags=["update", "upgrade", "repo", "dependencies", "deployment"],
        has_rollback=True,
        
        steps=[
            POTStep(
                id="execute_pull",
                name="Pull de cambios remotos",
                description="Obtener últimos cambios del repositorio",
                step_type=StepType.SCRIPT,
                script_path="scripts/repo_monitor.py",
                timeout_seconds=120,
                capture_output=True,
                continue_on_failure=True,
            ),
            
            POTStep(
                id="update_submodules",
                name="Actualizar submodules",
                description="Sincronizar submodules si existen",
                step_type=StepType.COMMAND,
                command="git submodule update --init --recursive",
                timeout_seconds=120,
                continue_on_failure=True,
                tutorial_notes="Solo aplica si hay submodules (nexus/, etc)",
            ),
            
            POTStep(
                id="install_deps",
                name="Instalar dependencias",
                description="Actualizar paquetes Python",
                step_type=StepType.COMMAND,
                command="pip install -r requirements.txt --quiet",
                timeout_seconds=300,
                continue_on_failure=True,
            ),
            
            POTStep(
                id="compile_check",
                name="Verificar sintaxis Python",
                description="Compilar todos los módulos para detectar errores",
                step_type=StepType.COMMAND,
                command="python -m compileall modules/ -q",
                timeout_seconds=60,
                continue_on_failure=True,
            ),
            
            POTStep(
                id="restart_services",
                name="Reiniciar servicios",
                description="Reiniciar para cargar cambios",
                step_type=StepType.COMMAND,
                command="powershell -ExecutionPolicy Bypass -File scripts/restart_service_clean.ps1 -Service robot",
                timeout_seconds=90,
                continue_on_failure=True,
            ),
            
            POTStep(
                id="wait_stabilize",
                name="Esperar estabilización",
                description="Dar tiempo a los servicios para iniciar",
                step_type=StepType.WAIT,
                wait_seconds=10,
            ),
            
            POTStep(
                id="health_check",
                name="Verificar salud del sistema",
                description="Confirmar que todo funciona",
                step_type=StepType.HTTP,
                http_method="GET",
                http_url="http://127.0.0.1:8791/health",
                timeout_seconds=20,
                capture_output=True,
            ),
            
            POTStep(
                id="log_to_bitacora",
                name="Registrar en bitácora",
                description="Log de actualización",
                step_type=StepType.HTTP,
                http_method="POST",
                http_url="http://127.0.0.1:8791/ans/evolution-log",
                http_body={
                    "message": "[UPDATE] Repositorio actualizado por POT repo_update",
                    "ok": True,
                    "source": "quality_pot"
                },
            ),
            
            POTStep(
                id="notify_update",
                name="Notificar actualización",
                description="Enviar notificación a canales",
                step_type=StepType.NOTIFY,
                notify_channel="telegram",
                notify_message="📦 Repositorio actualizado exitosamente",
                continue_on_failure=True,
            ),
        ],
        
        rollback_steps=[
            POTStep(
                id="rollback_reset",
                name="Revertir a commit anterior",
                description="Si algo falló, volver al estado previo",
                step_type=StepType.COMMAND,
                command="git reset --hard HEAD~1",
                timeout_seconds=30,
            ),
            
            POTStep(
                id="rollback_restart",
                name="Reiniciar servicios",
                description="Reiniciar con código anterior",
                step_type=StepType.COMMAND,
                command="powershell -ExecutionPolicy Bypass -File scripts/restart_service_clean.ps1 -Service robot",
                timeout_seconds=90,
            ),
            
            POTStep(
                id="rollback_notify",
                name="Notificar rollback",
                description="Alertar del rollback",
                step_type=StepType.NOTIFY,
                notify_channel="telegram",
                notify_message="⚠️ ROLLBACK ejecutado en repo_update",
            ),
        ],
    )
