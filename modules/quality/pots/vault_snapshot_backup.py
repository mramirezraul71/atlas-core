from __future__ import annotations

from typing import Dict

from ..models import POT, POTSeverity, POTStep


def get_pot() -> POT:
    def _run(ctx: Dict) -> str:
        # Crea un snapshot determinista (seguro, sin dependencias externas).
        from modules.command_router import handle

        label = (ctx.get("label") or "backup").strip() or "backup"
        return handle(f"/snapshot {label}")

    return POT(
        id="vault_snapshot_backup",
        name="Vault Snapshot Backup",
        description="Crea snapshot en Vault (C:\\ATLAS\\snapshots por defecto).",
        severity=POTSeverity.MED,
        rules=[
            "Seguro: solo crea carpeta snapshot + meta.json.",
            "No borra nada, no toca Git, no toca red.",
        ],
        tags=["backup", "snapshot", "vault"],
        steps=[POTStep(id="snapshot", name="Crear snapshot", run=_run, fatal=True)],
    )

