from __future__ import annotations

from typing import Dict

from ..models import POT, POTSeverity, POTStep


def get_pot() -> POT:
    def _run(ctx: Dict) -> str:
        return "ATLAS Doctor fue eliminado del sistema."

    return POT(
        id="system_doctor_removed",
        name="System Doctor Removed",
        description="Compatibilidad: el modulo Doctor fue retirado y ya no ejecuta diagnostico autonomo.",
        severity=POTSeverity.MED,
        rules=[
            "Compatibilidad solamente: no inicia procesos ni inspecciones.",
            "Si se invoca, responde que el modulo fue eliminado.",
        ],
        tags=["health", "removed", "compat"],
        steps=[POTStep(id="doctor_removed", name="Informar eliminacion", run=_run, fatal=False)],
    )
