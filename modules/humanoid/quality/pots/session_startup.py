"""
POT: Session Startup (Inicio de Sesión ATLAS)
==============================================
Procedimiento de inicio de sesión de trabajo.

Triggers:
- Al iniciar ATLAS
- Al abrir dashboard
- Comando "buenos días"

Severidad: LOW
"""
from __future__ import annotations

import os

from modules.humanoid.quality.models import POT, POTStep, POTCategory, POTSeverity, StepType


def _push_base_url() -> str:
    """Base URL del PUSH (8791 por defecto). Prioridad: ATLAS_PUSH_BASE_URL, ATLAS_DASHBOARD_URL, host+SERVICE_PORT."""
    base = (os.getenv("ATLAS_PUSH_BASE_URL") or os.getenv("ATLAS_DASHBOARD_URL") or "").strip().rstrip("/")
    if base:
        return base
    port = int(os.getenv("SERVICE_PORT", "8791") or 8791)
    host = (os.getenv("ATLAS_PUSH_HOST") or "127.0.0.1").strip()
    return f"http://{host}:{port}"


def _robot_base_url() -> str:
    """Base URL del backend Robot (8002 por defecto)."""
    base = (
        os.getenv("ATLAS_ROBOT_BASE_URL")
        or os.getenv("ROBOT_BASE_URL")
        or os.getenv("NEXUS_ROBOT_API_URL")
        or ""
    ).strip().rstrip("/")
    if base:
        return base
    port = int(os.getenv("ROBOT_PORT", "8002") or 8002)
    host = (os.getenv("ROBOT_HOST") or "127.0.0.1").strip()
    return f"http://{host}:{port}"


def get_pot() -> POT:
    push = _push_base_url()
    robot = _robot_base_url()
    return POT(
        id="session_startup",
        name="Inicio de Sesión ATLAS",
        description="""
Procedimiento de inicio de sesión que prepara el sistema para trabajo:
- Sincronizar con repositorio remoto
- Verificar servicios
- Cargar configuraciones
- Notificar disponibilidad
        """.strip(),
        category=POTCategory.DIAGNOSTIC,
        severity=POTSeverity.LOW,
        version="1.0.0",
        author="ATLAS QA Senior",
        
        trigger_check_ids=["startup_*", "session_start"],
        trigger_keywords=["startup", "inicio", "buenos dias", "start session", "morning"],
        
        prerequisites=[
            "Sistema arrancado",
            "Servicios básicos disponibles",
        ],
        required_services=[],
        required_permissions=["git_read", "service_status"],
        
        objectives=[
            "Sincronizar repositorio",
            "Verificar salud del sistema",
            "Preparar workspace",
            "Notificar inicio de sesión",
        ],
        success_criteria="Sistema listo para trabajo con todos los servicios verificados",
        estimated_duration_minutes=3,
        
        tutorial_overview="""
## Guía de Inicio de Sesión

### Secuencia de Startup
```
┌──────────┐     ┌──────────┐     ┌──────────┐     ┌──────────┐
│  Git     │────▶│  Health  │────▶│ Services │────▶│  Ready!  │
│  Sync    │     │  Check   │     │  Verify  │     │          │
└──────────┘     └──────────┘     └──────────┘     └──────────┘
```

### ¿Por qué Startup?
1. **Sincronización**: Obtener últimos cambios del equipo
2. **Verificación**: Confirmar que todo funciona
3. **Preparación**: Cargar contexto necesario
4. **Notificación**: Avisar disponibilidad

### Comandos Útiles Post-Startup
- `git status -sb` - Ver estado rápido
- Health/status del PUSH: contra la base configurada (env `ATLAS_PUSH_BASE_URL` / host + `SERVICE_PORT`), por ejemplo `GET .../health` y `GET .../status`

Variables de entorno relevantes: `ATLAS_PUSH_BASE_URL`, `ATLAS_DASHBOARD_URL`, `SERVICE_PORT`, `ATLAS_PUSH_HOST`; Robot: `ATLAS_ROBOT_BASE_URL`, `ROBOT_BASE_URL`, `NEXUS_ROBOT_API_URL`, `ROBOT_PORT`, `ROBOT_HOST`.
        """.strip(),
        
        best_practices=[
            "Ejecutar al inicio de cada sesión de trabajo",
            "Revisar el resumen de cambios remotos",
            "Verificar que no hay incidentes pendientes",
        ],
        
        related_pots=["git_pull", "diagnostic_full", "session_shutdown"],
        tags=["startup", "session", "sync", "initialization"],
        
        steps=[
            POTStep(
                id="fetch_remote",
                name="Obtener cambios remotos",
                description="Sincronizar información del repositorio",
                step_type=StepType.COMMAND,
                command="git fetch origin",
                timeout_seconds=60,
                continue_on_failure=True,
            ),
            
            POTStep(
                id="check_behind",
                name="Verificar commits pendientes",
                description="Ver si hay cambios para pull",
                step_type=StepType.COMMAND,
                command="git status -sb",
                timeout_seconds=10,
                capture_output=True,
            ),
            
            POTStep(
                id="pull_if_needed",
                name="Pull si hay cambios",
                description="Traer cambios del remoto (modo seguro: solo fast-forward, sin rebase)",
                step_type=StepType.COMMAND,
                command="git pull --ff-only",
                timeout_seconds=120,
                continue_on_failure=True,
            ),
            
            POTStep(
                id="check_health",
                name="Verificar salud del sistema",
                description="Health check general",
                step_type=StepType.HTTP,
                http_method="GET",
                http_url=f"{push}/health",
                timeout_seconds=15,
                capture_output=True,
                continue_on_failure=True,
            ),
            
            POTStep(
                id="check_robot",
                name="Verificar Robot backend",
                description="Confirmar Robot disponible",
                step_type=StepType.HTTP,
                http_method="GET",
                http_url=f"{robot}/api/health",
                timeout_seconds=15,
                continue_on_failure=True,
            ),
            
            POTStep(
                id="check_incidents",
                name="Revisar incidentes pendientes",
                description="Ver si hay algo que atender",
                step_type=StepType.HTTP,
                http_method="GET",
                http_url=f"{push}/ans/incidents?status=open",
                timeout_seconds=10,
                capture_output=True,
                continue_on_failure=True,
            ),
            
            POTStep(
                id="check_approvals",
                name="Revisar aprobaciones pendientes",
                description="Ver si hay approvals esperando",
                step_type=StepType.HTTP,
                http_method="GET",
                http_url=f"{push}/approvals/pending",
                timeout_seconds=10,
                capture_output=True,
                continue_on_failure=True,
            ),
            
            POTStep(
                id="log_startup",
                name="Registrar inicio de sesión",
                description="Log en bitácora ANS",
                step_type=StepType.HTTP,
                http_method="POST",
                http_url=f"{push}/ans/evolution-log",
                http_body={
                    "message": "[SESSION] Inicio de sesión - Sistema preparado",
                    "ok": True,
                    "source": "quality_pot"
                },
            ),
            
            POTStep(
                id="notify_ready",
                name="Notificar disponibilidad",
                description="Avisar que ATLAS está listo",
                step_type=StepType.NOTIFY,
                notify_channel="ops",
                notify_message="🟢 ATLAS listo para trabajar. Sesión iniciada.",
                continue_on_failure=True,
            ),
        ],
    )
