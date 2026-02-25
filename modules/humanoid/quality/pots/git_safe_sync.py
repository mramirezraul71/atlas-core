"""
POT: Git Safe Sync (Autoreparación Git)
=======================================
Procedimiento de emergencia para mantener el repo en estado operativo sin romper historial.

Objetivo:
- Detectar y corregir estados Git peligrosos (index.lock zombie, rebase/merge en progreso)
- Sincronizar con remoto de forma segura (solo fast-forward, sin rebase)

NOTA DE SEGURIDAD:
- Por defecto NO aborta rebase/merge automáticamente. Puede habilitarse por env vars.
"""

from modules.humanoid.quality.models import (POT, POTCategory, POTSeverity,
                                             POTStep, StepType)


def get_pot() -> POT:
    return POT(
        id="git_safe_sync",
        name="Auto-reparación Git + Sync seguro",
        description="""
Procedimiento para estabilizar el repositorio cuando la autonomía detecta estados peligrosos:
- Limpia `index.lock` stale (si aplica)
- Detecta rebase/merge en progreso
- Ejecuta `git fetch origin` y `git pull --ff-only`

Este POT existe para evitar bucles de rebase/conflicto durante arranque o autosync.
        """.strip(),
        category=POTCategory.RECOVERY,
        severity=POTSeverity.HIGH,
        version="1.0.0",
        author="ATLAS QA Senior",
        trigger_check_ids=[
            "git_unsafe",
            "git_lock",
            "rebase_in_progress",
            "merge_in_progress",
            "detached_head",
        ],
        trigger_keywords=[
            "git unsafe",
            "index.lock",
            "rebase",
            "merge",
            "detached",
            "safe sync",
            "ff-only",
        ],
        prerequisites=[
            "Git instalado",
            "Repositorio con remote origin configurado",
        ],
        required_services=[],
        required_permissions=["git_write", "network_access"],
        objectives=[
            "Eliminar estados Git tóxicos para la autonomía",
            "Sincronizar sin rebase automático",
            "Dejar evidencia en Bitácora",
        ],
        success_criteria="Repo estable y actualizado sin rebase automático",
        estimated_duration_minutes=2,
        best_practices=[
            "Mantener autosync en modo seguro (ff-only)",
            "Exigir aprobación para abortar rebase/merge si el owner no lo autorizó",
        ],
        warnings=[
            "Abortar rebase/merge puede descartar progreso no consolidado; por defecto está deshabilitado",
        ],
        tags=["git", "recovery", "sync", "ff-only", "autonomy"],
        steps=[
            POTStep(
                id="remediate_and_sync",
                name="Remediar estado Git y sync seguro",
                description="Ejecuta script de auto-reparación Git (lock/rebase/ff-only).",
                step_type=StepType.SCRIPT,
                script_path="modules/humanoid/quality/scripts/git_safe_sync.py",
                timeout_seconds=180,
                capture_output=True,
            ),
            POTStep(
                id="log_to_bitacora",
                name="Registrar en bitácora",
                description="Registrar ejecución del POT en ANS (para auditoría).",
                step_type=StepType.HTTP,
                http_method="POST",
                http_url="http://127.0.0.1:8791/ans/evolution-log",
                http_body={
                    "message": "[GIT] Safe sync ejecutado por POT git_safe_sync (ff-only + autoreparación)",
                    "ok": True,
                    "source": "quality_pot",
                },
                continue_on_failure=True,
            ),
            POTStep(
                id="notify_ops",
                name="Notificar a OPS",
                description="Notifica que el repo fue estabilizado (sin spam).",
                step_type=StepType.NOTIFY,
                notify_channel="ops",
                notify_message="🧰 Git estabilizado: safe-sync ejecutado (ff-only).",
                continue_on_failure=True,
            ),
        ],
    )
