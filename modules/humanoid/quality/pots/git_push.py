"""
POT: Git Push (Push al Remoto)
==============================
Procedimiento para enviar commits al repositorio remoto.

Triggers:
- Después de commits locales
- Cuando hay commits pendientes de sincronizar

Severidad: HIGH (modifica repositorio remoto)
"""
from modules.humanoid.quality.models import POT, POTStep, POTCategory, POTSeverity, StepType


def get_pot() -> POT:
    return POT(
        id="git_push",
        name="Push al Repositorio Remoto",
        description="""
Procedimiento para enviar commits locales al repositorio remoto (GitHub/GitLab).
Incluye verificación de estado, pull previo si necesario, y push.
        """.strip(),
        category=POTCategory.DEPLOYMENT,
        severity=POTSeverity.HIGH,
        version="1.0.0",
        author="ATLAS QA Senior",
        
        trigger_check_ids=["git_ahead", "push_pending", "sync_*"],
        trigger_keywords=["push", "sync", "upload", "github", "remote"],
        
        prerequisites=[
            "Commits locales pendientes de push",
            "Acceso al repositorio remoto configurado",
            "Credenciales Git configuradas",
        ],
        required_services=[],
        required_permissions=["git_push", "network_access"],
        
        objectives=[
            "Verificar commits pendientes de push",
            "Hacer pull previo para evitar conflictos",
            "Push de commits al remoto",
            "Verificar sincronización exitosa",
            "Notificar a canales",
        ],
        success_criteria="Branch local sincronizado con remoto",
        estimated_duration_minutes=3,
        
        tutorial_overview="""
## Guía de Push al Remoto

### Flujo Seguro de Push
```
┌──────────┐     ┌──────────┐     ┌──────────┐     ┌──────────┐
│  Status  │────▶│  Fetch   │────▶│  Pull    │────▶│   Push   │
└──────────┘     └──────────┘     └──────────┘     └──────────┘
```

### Verificar Estado
```bash
git status -sb
# ## main...origin/main [ahead 3]
# ^^^ indica 3 commits pendientes de push
```

### Resolver Conflictos
Si pull tiene conflictos:
1. Resolver manualmente
2. git add <archivos>
3. git commit
4. Reintentar push
        """.strip(),
        
        best_practices=[
            "SIEMPRE hacer pull antes de push",
            "Verificar que CI/CD no está corriendo",
            "No hacer push --force a main/master",
            "Revisar commits antes de push",
        ],
        
        warnings=[
            "NUNCA push --force sin autorización explícita",
            "Verificar que no hay secrets en commits",
            "Si hay conflictos, resolver antes de push",
        ],
        
        related_pots=["git_commit", "git_pull", "repo_update"],
        tags=["git", "push", "sync", "remote", "deployment"],
        has_rollback=False,  # Push no tiene rollback simple
        
        steps=[
            POTStep(
                id="check_remote",
                name="Verificar conexión remota",
                description="Confirmar acceso al repositorio remoto",
                step_type=StepType.COMMAND,
                command="git remote -v",
                timeout_seconds=10,
                capture_output=True,
            ),
            
            POTStep(
                id="check_ahead_behind",
                name="Verificar commits pendientes",
                description="Ver cuántos commits adelante/atrás estamos",
                step_type=StepType.COMMAND,
                command="git status -sb",
                timeout_seconds=10,
                capture_output=True,
                tutorial_notes="[ahead N] = N commits para push, [behind N] = N para pull",
            ),
            
            POTStep(
                id="fetch_remote",
                name="Fetch del remoto",
                description="Obtener información actualizada del remoto",
                step_type=StepType.COMMAND,
                command="git fetch origin",
                timeout_seconds=60,
                continue_on_failure=True,
            ),
            
            POTStep(
                id="pull_if_behind",
                name="Pull si hay cambios remotos",
                description="Traer cambios del remoto si estamos atrás",
                step_type=StepType.COMMAND,
                command="git pull --rebase origin HEAD",
                timeout_seconds=120,
                continue_on_failure=True,
                tutorial_notes="--rebase mantiene historial limpio",
            ),
            
            POTStep(
                id="execute_push",
                name="Ejecutar push",
                description="Enviar commits al remoto",
                step_type=StepType.COMMAND,
                command="git push origin HEAD",
                timeout_seconds=180,
                capture_output=True,
            ),
            
            POTStep(
                id="verify_sync",
                name="Verificar sincronización",
                description="Confirmar que estamos sincronizados",
                step_type=StepType.COMMAND,
                command="git status -sb",
                timeout_seconds=10,
                capture_output=True,
                check_expression="'ahead' not in output and 'behind' not in output",
            ),
            
            POTStep(
                id="log_to_bitacora",
                name="Registrar en bitácora",
                description="Log del push en ANS",
                step_type=StepType.HTTP,
                http_method="POST",
                http_url="http://127.0.0.1:8791/ans/evolution-log",
                http_body={
                    "message": "[GIT] Push al remoto completado por POT git_push",
                    "ok": True,
                    "source": "quality_pot"
                },
                continue_on_failure=True,
            ),
            
            POTStep(
                id="notify_push",
                name="Notificar push",
                description="Enviar notificación a Telegram",
                step_type=StepType.NOTIFY,
                notify_channel="telegram",
                notify_message="🚀 Push al repositorio remoto completado",
                continue_on_failure=True,
            ),
        ],
    )
