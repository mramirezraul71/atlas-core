"""
POT: Git Pull (Actualización desde Remoto)
==========================================
Procedimiento para obtener cambios del repositorio remoto.

Triggers:
- Al inicio del día/sesión
- Antes de comenzar cambios
- Cuando hay cambios en el remoto

Severidad: MEDIUM
"""
from modules.humanoid.quality.models import POT, POTStep, POTCategory, POTSeverity, StepType


def get_pot() -> POT:
    return POT(
        id="git_pull",
        name="Pull desde Repositorio Remoto",
        description="""
Procedimiento para obtener y aplicar cambios del repositorio remoto.
Incluye stash de cambios locales si necesario, pull, y restauración.
        """.strip(),
        category=POTCategory.DEPLOYMENT,
        severity=POTSeverity.MEDIUM,
        version="1.0.0",
        author="ATLAS QA Senior",
        
        trigger_check_ids=["git_behind", "pull_pending", "remote_*"],
        trigger_keywords=["pull", "update", "actualizar", "fetch", "sync"],
        
        prerequisites=[
            "Repositorio Git configurado con remote",
            "Acceso de red al remoto",
        ],
        required_services=[],
        required_permissions=["git_read", "network_access"],
        
        objectives=[
            "Guardar cambios locales pendientes (stash)",
            "Obtener cambios del remoto",
            "Aplicar cambios (merge/rebase)",
            "Restaurar cambios locales si hubo stash",
            "Verificar estado limpio",
        ],
        success_criteria="Branch local actualizado con últimos cambios del remoto",
        estimated_duration_minutes=2,
        
        tutorial_overview="""
## Guía de Pull desde Remoto

### Flujo Seguro de Pull
```
┌──────────┐     ┌──────────┐     ┌──────────┐     ┌──────────┐
│  Stash   │────▶│  Fetch   │────▶│  Pull    │────▶│  Pop     │
│ (if any) │     │          │     │          │     │  Stash   │
└──────────┘     └──────────┘     └──────────┘     └──────────┘
```

### Manejo de Conflictos
Si hay conflictos después del pull:
1. Identificar archivos: `git status`
2. Editar y resolver: buscar `<<<<<<<`
3. Marcar resuelto: `git add <archivo>`
4. Continuar: `git rebase --continue` o `git merge --continue`
        """.strip(),
        
        best_practices=[
            "Pull al inicio de cada sesión de trabajo",
            "Stash cambios locales antes de pull",
            "Preferir --rebase para historial limpio",
            "Resolver conflictos inmediatamente",
        ],
        
        warnings=[
            "Si hay conflictos, no hacer más cambios hasta resolver",
            "Verificar que los tests pasen después del pull",
        ],
        
        related_pots=["git_commit", "git_push", "repo_update"],
        tags=["git", "pull", "sync", "update", "deployment"],
        
        steps=[
            POTStep(
                id="check_local_changes",
                name="Verificar cambios locales",
                description="Ver si hay cambios sin commit",
                step_type=StepType.COMMAND,
                command="git status --porcelain",
                timeout_seconds=10,
                capture_output=True,
            ),
            
            POTStep(
                id="stash_if_needed",
                name="Stash de cambios locales",
                description="Guardar cambios temporalmente si existen",
                step_type=StepType.COMMAND,
                command="git stash push -m 'POT_git_pull_autostash'",
                timeout_seconds=30,
                continue_on_failure=True,
                condition="context.get('check_local_changes_output', '').strip() != ''",
                tutorial_notes="Solo hace stash si hay cambios pendientes",
            ),
            
            POTStep(
                id="fetch_remote",
                name="Fetch del remoto",
                description="Obtener referencias actualizadas",
                step_type=StepType.COMMAND,
                command="git fetch origin",
                timeout_seconds=60,
            ),
            
            POTStep(
                id="check_behind",
                name="Verificar commits pendientes",
                description="Ver cuántos commits estamos atrás",
                step_type=StepType.COMMAND,
                command="git rev-list --count HEAD..origin/HEAD",
                timeout_seconds=10,
                capture_output=True,
                continue_on_failure=True,
            ),
            
            POTStep(
                id="execute_pull",
                name="Ejecutar pull",
                description="Traer y aplicar cambios del remoto",
                step_type=StepType.COMMAND,
                command="git pull --rebase origin HEAD",
                timeout_seconds=180,
                capture_output=True,
                tutorial_notes="--rebase re-aplica commits locales sobre los remotos",
            ),
            
            POTStep(
                id="pop_stash",
                name="Restaurar stash",
                description="Recuperar cambios locales guardados",
                step_type=StepType.COMMAND,
                command="git stash pop",
                timeout_seconds=30,
                continue_on_failure=True,
                condition="'POT_git_pull_autostash' in context.get('stash_if_needed_output', '')",
            ),
            
            POTStep(
                id="verify_status",
                name="Verificar estado final",
                description="Confirmar que el pull fue exitoso",
                step_type=StepType.COMMAND,
                command="git status -sb",
                timeout_seconds=10,
                capture_output=True,
            ),
            
            POTStep(
                id="log_to_bitacora",
                name="Registrar en bitácora",
                description="Log del pull en ANS",
                step_type=StepType.HTTP,
                http_method="POST",
                http_url="http://127.0.0.1:8791/ans/evolution-log",
                http_body={
                    "message": "[GIT] Pull desde remoto completado por POT git_pull",
                    "ok": True,
                    "source": "quality_pot"
                },
                continue_on_failure=True,
            ),
        ],
    )
