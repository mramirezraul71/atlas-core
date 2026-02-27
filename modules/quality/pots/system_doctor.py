from __future__ import annotations

from typing import Dict

from ..models import POT, POTSeverity, POTStep


def get_pot() -> POT:
    def _run(ctx: Dict) -> str:
        # Reusa el diagnóstico determinista del command_router.
        from modules.command_router import handle

        return handle("/doctor")

    return POT(
        id="system_doctor",
        name="System Doctor (Paths/Vault/Logs/Snapshots)",
        description="Verifica que Vault/Notas/Logs/Snapshots existan y sean escribibles. No modifica servicios.",
        severity=POTSeverity.MED,
        rules=[
            "Check-only: valida directorios y permisos de escritura.",
            "Si falla, NO intenta reparar automáticamente; devuelve causa para corrección manual.",
        ],
        tags=["health", "doctor", "vault", "paths"],
        steps=[POTStep(id="doctor", name="Ejecutar /doctor", run=_run, fatal=True)],
    )

