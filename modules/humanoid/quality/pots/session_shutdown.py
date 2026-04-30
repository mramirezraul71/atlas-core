"""
POT: Session Shutdown (Cierre de Sesión ATLAS)
==============================================
Procedimiento de cierre de sesión de trabajo.

Triggers:
- Al finalizar trabajo
- Comando "buenas noches"
- Shutdown del sistema

Severidad: LOW
"""
from modules.humanoid.quality.models import (POT, POTCategory, POTSeverity,
                                             POTStep, StepType)


def get_pot() -> POT:
    return POT(
        id="session_shutdown",
        name="Cierre de Sesión ATLAS",
        description="""
Procedimiento de cierre de sesión que asegura:
- Commit de cambios pendientes
- Push al repositorio
- Resumen de sesión
- Cierre ordenado
        """.strip(),
        category=POTCategory.MAINTENANCE,
        severity=POTSeverity.LOW,
        version="1.0.0",
        author="ATLAS QA Senior",
        trigger_check_ids=["shutdown_*", "session_end"],
        trigger_keywords=[
            "shutdown",
            "cierre",
            "buenas noches",
            "end session",
            "goodnight",
            "finalizar",
        ],
        prerequisites=[
            "Sesión activa",
        ],
        required_services=["push"],
        required_permissions=["git_write", "git_push"],
        objectives=[
            "Guardar cambios pendientes",
            "Sincronizar con remoto",
            "Generar resumen de sesión",
            "Notificar cierre",
        ],
        success_criteria="Todos los cambios guardados y sincronizados",
        estimated_duration_minutes=3,
        tutorial_overview="""
## Guía de Cierre de Sesión

### Secuencia de Shutdown
```
┌──────────┐     ┌──────────┐     ┌──────────┐     ┌──────────┐
│  Check   │────▶│  Commit  │────▶│   Push   │────▶│ Summary  │
│ Changes  │     │   All    │     │          │     │ & Notify │
└──────────┘     └──────────┘     └──────────┘     └──────────┘
```

### ¿Por qué Shutdown Ordenado?
1. **No perder trabajo**: Commit de todo lo pendiente
2. **Sincronización**: Push para backup remoto
3. **Documentación**: Log de lo trabajado
4. **Tranquilidad**: Saber que todo está guardado
        """.strip(),
        best_practices=[
            "Ejecutar antes de cerrar la PC",
            "Revisar qué se va a commitear",
            "Verificar que el push fue exitoso",
        ],
        related_pots=["git_commit", "git_push", "session_startup"],
        tags=["shutdown", "session", "sync", "backup"],
        steps=[
            POTStep(
                id="check_local_changes",
                name="Verificar cambios locales",
                description="Ver si hay algo pendiente de commit",
                step_type=StepType.COMMAND,
                command="git status --porcelain",
                timeout_seconds=10,
                capture_output=True,
            ),
            POTStep(
                id="stage_all",
                name="Stage todos los cambios",
                description="Agregar cambios al staging",
                step_type=StepType.COMMAND,
                command="git add -A",
                timeout_seconds=15,
            ),
            POTStep(
                id="commit_session",
                name="Commit de sesión",
                description="Guardar cambios del día",
                step_type=StepType.COMMAND,
                command='git commit -m "chore: session commit by ATLAS" --allow-empty',
                timeout_seconds=30,
                continue_on_failure=True,
            ),
            POTStep(
                id="push_changes",
                name="Push al remoto",
                description="Sincronizar con GitHub",
                step_type=StepType.COMMAND,
                command="git push origin HEAD",
                timeout_seconds=180,
                capture_output=True,
                continue_on_failure=True,
            ),
            POTStep(
                id="get_session_stats",
                name="Obtener estadísticas de sesión",
                description="Commits del día",
                step_type=StepType.COMMAND,
                command='git log --oneline --since="midnight"',
                timeout_seconds=10,
                capture_output=True,
            ),
            POTStep(
                id="log_shutdown",
                name="Registrar cierre de sesión",
                description="Log en bitácora ANS",
                step_type=StepType.HTTP,
                http_method="POST",
                http_url="http://127.0.0.1:8791/ans/evolution-log",
                http_body={
                    "message": "[SESSION] Cierre de sesión - Trabajo guardado y sincronizado",
                    "ok": True,
                    "source": "quality_pot",
                },
            ),
            POTStep(
                id="notify_shutdown",
                name="Notificar cierre",
                description="Avisar fin de sesión",
                step_type=StepType.NOTIFY,
                notify_channel="telegram",
                notify_message="🌙 Sesión ATLAS finalizada. Cambios guardados y sincronizados. Buenas noches, Comandante.",
                continue_on_failure=True,
            ),
        ],
    )
