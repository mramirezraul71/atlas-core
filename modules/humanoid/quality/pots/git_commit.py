"""
POT: Git Commit (Commit de Cambios)
===================================
Procedimiento estándar para realizar commits de código.

Triggers:
- Cuando hay cambios pendientes para guardar
- Después de completar una tarea de desarrollo
- Antes de hacer push al remoto

Severidad: MEDIUM (modifica historial de Git)
"""
from modules.humanoid.quality.models import POT, POTStep, POTCategory, POTSeverity, StepType


def get_pot() -> POT:
    return POT(
        id="git_commit",
        name="Commit de Cambios Git",
        description="""
Procedimiento estándar para realizar commits de código al repositorio.
Incluye verificación de cambios, staging, generación de mensaje y commit.
        """.strip(),
        category=POTCategory.DEPLOYMENT,
        severity=POTSeverity.MEDIUM,
        version="1.0.0",
        author="ATLAS QA Senior",
        
        trigger_check_ids=["git_pending", "repo_changes", "commit_*"],
        trigger_keywords=["commit", "git", "save", "guardar", "cambios"],
        
        prerequisites=[
            "Repositorio Git inicializado",
            "Cambios pendientes en working directory",
            "No hay conflictos de merge activos",
        ],
        required_services=[],
        required_permissions=["git_write"],
        
        objectives=[
            "Verificar estado del repositorio",
            "Revisar cambios pendientes",
            "Staging de archivos relevantes",
            "Generar mensaje de commit descriptivo",
            "Ejecutar commit",
            "Notificar a canales de comunicación",
        ],
        success_criteria="Commit creado exitosamente con hash válido",
        estimated_duration_minutes=2,
        
        tutorial_overview="""
## Guía de Commit de Cambios

### Flujo de Commit
```
┌──────────┐     ┌──────────┐     ┌──────────┐     ┌──────────┐
│  Status  │────▶│   Diff   │────▶│   Add    │────▶│  Commit  │
└──────────┘     └──────────┘     └──────────┘     └──────────┘
```

### Convención de Mensajes
```
<tipo>(<scope>): <descripción corta>

[cuerpo opcional]

[footer opcional]
```

Tipos válidos:
- `feat`: Nueva funcionalidad
- `fix`: Corrección de bug
- `docs`: Documentación
- `style`: Formato (no afecta código)
- `refactor`: Refactorización
- `test`: Tests
- `chore`: Mantenimiento

### Archivos a Ignorar
- `.env`, `credenciales.*`
- `__pycache__/`, `.pytest_cache/`
- `*.log`, `*.tmp`
- `node_modules/`, `venv/`
        """.strip(),
        
        best_practices=[
            "Commits pequeños y frecuentes",
            "Un commit = un cambio lógico",
            "Mensaje descriptivo en presente",
            "No commitear archivos sensibles",
            "Verificar diff antes de commit",
        ],
        
        warnings=[
            "NUNCA commitear credenciales o secrets",
            "Verificar que no hay archivos grandes binarios",
            "No usar --force en commits normales",
        ],
        
        related_pots=["git_push", "git_pull", "repo_update"],
        tags=["git", "commit", "vcs", "deployment"],
        
        steps=[
            POTStep(
                id="check_git_status",
                name="Verificar estado Git",
                description="Ver archivos modificados, staged y untracked",
                step_type=StepType.COMMAND,
                command="git status --porcelain",
                timeout_seconds=10,
                capture_output=True,
                tutorial_notes="""
git status --porcelain devuelve formato parseable:
- M = modified
- A = added
- D = deleted
- ?? = untracked
                """,
            ),
            
            POTStep(
                id="check_no_conflicts",
                name="Verificar sin conflictos",
                description="Asegurar que no hay conflictos de merge",
                step_type=StepType.COMMAND,
                command="git diff --check",
                timeout_seconds=10,
                continue_on_failure=True,
                capture_output=True,
            ),
            
            POTStep(
                id="show_diff_summary",
                name="Mostrar resumen de cambios",
                description="Ver estadísticas de lo que cambiará",
                step_type=StepType.COMMAND,
                command="git diff --stat",
                timeout_seconds=15,
                capture_output=True,
                tutorial_notes="Muestra archivos cambiados con +/- de líneas.",
            ),
            
            POTStep(
                id="stage_changes",
                name="Staging de cambios",
                description="Agregar cambios al staging area",
                step_type=StepType.COMMAND,
                command="git add -A",
                timeout_seconds=15,
                tutorial_notes="""
git add -A agrega:
- Archivos nuevos (untracked)
- Archivos modificados
- Archivos eliminados
Excluye lo que está en .gitignore
                """,
            ),
            
            POTStep(
                id="verify_staged",
                name="Verificar staged",
                description="Confirmar qué está en staging",
                step_type=StepType.COMMAND,
                command="git diff --cached --stat",
                timeout_seconds=10,
                capture_output=True,
            ),
            
            POTStep(
                id="execute_commit",
                name="Ejecutar commit",
                description="Crear el commit con mensaje",
                step_type=StepType.COMMAND,
                command='git commit -m "chore: automated commit by ATLAS POT"',
                timeout_seconds=30,
                capture_output=True,
                tutorial_notes="""
El mensaje puede personalizarse vía contexto:
context['commit_message'] = "feat: nueva funcionalidad"
                """,
            ),
            
            POTStep(
                id="get_commit_hash",
                name="Obtener hash del commit",
                description="Capturar el hash del commit creado",
                step_type=StepType.COMMAND,
                command="git rev-parse --short HEAD",
                timeout_seconds=5,
                capture_output=True,
            ),
            
            POTStep(
                id="log_to_bitacora",
                name="Registrar en bitácora",
                description="Log del commit en ANS",
                step_type=StepType.HTTP,
                http_method="POST",
                http_url="http://127.0.0.1:8791/ans/evolution-log",
                http_body={
                    "message": "[GIT] Commit realizado por POT git_commit",
                    "ok": True,
                    "source": "quality_pot"
                },
                continue_on_failure=True,
            ),
            
            POTStep(
                id="notify_commit",
                name="Notificar commit",
                description="Enviar notificación a canales",
                step_type=StepType.NOTIFY,
                notify_channel="ops",
                notify_message="📝 Commit realizado exitosamente",
                continue_on_failure=True,
            ),
        ],
    )
