from __future__ import annotations

from typing import Dict

from ..executor import git_safe_sync
from ..models import POT, POTSeverity, POTStep


def get_pot() -> POT:
    rules = [
        "REGLA 0: PROHIBIDO rebase automático. Solo `git pull --ff-only` (fast-forward).",
        "REGLA 1: Por defecto es CHECK ONLY. Para permitir cambios: QUALITY_GIT_ALLOW_PULL/COMMIT/PUSH=true explícitos.",
        "REGLA 2: No se commitea nada fuera de include_paths (default: atlas_adapter,modules,tools,config).",
        "REGLA 3: Se excluyen SIEMPRE: logs/, snapshots/, venvs/, temp_*, __pycache__/ y ATLAS_VAULT/.",
        "REGLA 4: Si el árbol no está limpio, se bloquean pull/push (seguridad).",
        "REGLA 5: Si el repo está en detached HEAD o hay rebase en progreso, SYNC queda BLOQUEADO.",
    ]

    def _run(ctx: Dict) -> str:
        mode = (ctx.get("mode") or "check").strip().lower()
        if mode not in ("check", "sync"):
            mode = "check"
        return git_safe_sync(mode=mode)

    return POT(
        id="git_safe_sync",
        name="Git Safe Sync (SIN REBASE)",
        description="Ciclo Git seguro para ATLAS_PUSH: fetch+evidencia; opcionalmente commit/pull/push bajo flags explícitos.",
        severity=POTSeverity.HIGH,
        rules=rules,
        tags=["git", "repo", "sync", "safe"],
        steps=[
            POTStep(id="git_safe_sync", name="Ejecutar ciclo Git seguro", run=_run, fatal=True),
        ],
    )

