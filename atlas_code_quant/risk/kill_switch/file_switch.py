"""Atlas Code Quant — File-based kill switch (F19).

Kill switch operado por presencia de un fichero en disco
(``ATLAS_KILLSWITCH_FILE``). Diseño deliberadamente simple:

* Si el fichero existe y contiene la marca ``"KILL"`` (o cualquier
  contenido no vacío), el kill switch se considera **activado**.
* No se borra automáticamente: el desactivado es manual/humano.
* Defensivo: errores de E/S se traducen a ``activated=False`` con
  ``error`` para evitar bloqueos espurios por permisos en local.

Sólo se usa para el pipeline paper. NO autoriza ni desactiva live.
"""

from __future__ import annotations

import logging
import os
from dataclasses import dataclass
from pathlib import Path
from typing import Callable

logger = logging.getLogger("atlas.code_quant.risk.kill_switch")


__all__ = [
    "KillSwitchStatus",
    "FileKillSwitch",
    "load_kill_switch_path_from_env",
]


@dataclass(frozen=True)
class KillSwitchStatus:
    activated: bool
    path: str
    reason: str
    raw_marker: str | None = None
    error: str | None = None


def load_kill_switch_path_from_env(default: str = "/tmp/atlas_killswitch") -> str:
    return os.environ.get("ATLAS_KILLSWITCH_FILE") or default


class FileKillSwitch:
    """Kill switch basado en fichero (F19).

    El path se lee al construir; los métodos relean el fichero
    cada vez que se llaman para reflejar cambios externos.
    """

    def __init__(
        self,
        *,
        path: str | os.PathLike[str] | None = None,
        path_provider: Callable[[], str] | None = None,
    ) -> None:
        if path is not None:
            self._provider: Callable[[], str] = lambda p=str(path): p
        elif path_provider is not None:
            self._provider = path_provider
        else:
            self._provider = load_kill_switch_path_from_env

    def status(self) -> KillSwitchStatus:
        try:
            path_str = self._provider()
        except Exception as exc:  # noqa: BLE001
            return KillSwitchStatus(
                activated=False,
                path="",
                reason=f"path_provider_raised:{exc}",
                error=str(exc),
            )
        p = Path(path_str)
        if not p.exists():
            return KillSwitchStatus(
                activated=False,
                path=str(p),
                reason="killswitch_file_absent",
            )
        try:
            content = p.read_text("utf-8", errors="replace").strip()
        except Exception as exc:  # noqa: BLE001
            return KillSwitchStatus(
                activated=False,
                path=str(p),
                reason="killswitch_file_unreadable",
                error=str(exc),
            )
        if not content:
            # fichero vacío: no se considera activado para evitar
            # falsos positivos.
            return KillSwitchStatus(
                activated=False,
                path=str(p),
                reason="killswitch_file_empty",
                raw_marker="",
            )
        return KillSwitchStatus(
            activated=True,
            path=str(p),
            reason="killswitch_file_present",
            raw_marker=content[:64],  # truncado por seguridad
        )

    def is_activated(self) -> bool:
        return self.status().activated
