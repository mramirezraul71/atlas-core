"""`IntentRouter` — envoltorio tipado sobre `command_router.handle`.

Diseño (ver docs/atlas_push/PLAN_STEP_B.md):

- El router no duplica la ejecución: delega en
  ``modules.command_router.handle`` para obtener el ``output``.
- La clasificación (``kind``) se hace sobre el mismo texto, con la
  misma prioridad de ramas que ``handle``, pero **sin ejecutar** nada.
  Así evitamos efectos secundarios y garantizamos que ``kind`` y
  ``output`` provienen del mismo texto y son consistentes.
- ``IntentResult`` es inmutable; se compone vía ``replace`` si se
  necesita (no en B).

Regla de oro: este módulo no importa nada del brain core mayor de
ATLAS. La única dependencia externa a ``atlas_push`` es
``modules.command_router``, permitido por ARCHITECTURE §7.
"""

from __future__ import annotations

import re
from typing import Callable

# Dependencia permitida (ver PLAN_STEP_B §2).
from modules import command_router as _cr

from atlas_push.intents.types import IntentResult, Kind


# Patrones lenguaje natural — espejo de los usados en
# ``modules/command_router.py`` (funciones ``handle``).
# Se compilan una vez por módulo para minimizar coste por llamada.
_RE_NATURAL_NOTE_CREATE = re.compile(
    r"crea\s+una\s+nota\s+(llamada|con\s+el\s+t[ií]tulo)\s+(.+)$",
    re.IGNORECASE,
)
_RE_NATURAL_NOTE_APPEND = re.compile(
    r"agrega\s+a\s+la\s+nota\s+(.+?)\s+que\s+(.+)$",
    re.IGNORECASE,
)
_RE_NATURAL_SNAPSHOT = re.compile(
    r"crea\s+un\s+snapshot\s+llamado\s+(.+)$",
    re.IGNORECASE,
)


def _classify(text: str) -> Kind:
    """Clasifica ``text`` en uno de los ``Kind`` sin ejecutar nada.

    El orden replica el de ``command_router.handle`` para que cada
    texto mapee al mismo ``kind`` que la rama que se habría ejecutado
    allí.

    Nota sobre ``empty``: ``command_router.handle`` comprueba
    ``if not text`` (truthy), no ``text.strip()``. Por tanto un texto
    que sea sólo espacios/tabs NO es tratado como ``empty`` por el
    handle original; cae al fallback (``inbox``). Preservamos esa
    semántica exacta aquí: sólo ``None`` o ``""`` (vacío estricto)
    son ``empty``.
    """
    if text is None or text == "":
        return "empty"

    t = text.strip()
    low = t.lower()

    # Texto no vacío pero solo whitespace: ``handle`` lo trata como
    # fallback a inbox (no como empty). Replicamos.
    if not t:
        return "inbox.fallback"

    # Comandos estilo Telegram.
    if low.startswith("/status"):
        return "status"
    if low.startswith("/doctor"):
        return "doctor"
    if low.startswith("/modules"):
        return "modules"
    if low.startswith("/snapshot"):
        return "snapshot"

    if low.startswith("/note"):
        rest = t[5:].strip().lower()
        if rest.startswith("create"):
            return "note.create"
        if rest.startswith("append"):
            return "note.append"
        if rest.startswith("view"):
            return "note.view"
        # /note sin subcomando válido: en ``handle`` cae al help. No es
        # una acción propiamente dicha; lo marcamos como ``note.view``
        # NO sería correcto. El ``handle`` devuelve un texto de uso.
        # Lo tratamos como ``inbox.fallback`` porque su ``output`` no es
        # una acción de nota, y para el caller externo es equivalente a
        # "no pasó nada". Mantenemos consistencia con los 12 kinds
        # aprobados: no inventamos uno nuevo.
        return "inbox.fallback"

    # Lenguaje natural (mismo orden que ``handle``).
    if _RE_NATURAL_NOTE_CREATE.search(low):
        return "natural.note.create"
    if _RE_NATURAL_NOTE_APPEND.search(t):
        return "natural.note.append"
    if _RE_NATURAL_SNAPSHOT.search(t):
        return "natural.snapshot"
    if "módulos" in low and ("activos" in low or "tienes" in low):
        return "natural.modules"

    # Fallback final: va a Inbox.
    return "inbox.fallback"


class IntentRouter:
    """Envoltorio tipado sobre ``command_router.handle``.

    Uso:

        router = IntentRouter()
        result = router.handle("/status")
        assert result.kind == "status"
        assert result.output.startswith("ATLAS: OK")

    El router es sin estado; puede instanciarse una vez por proceso y
    reutilizarse (así se hace en ``atlas_adapter.atlas_http_api``).
    """

    # Inyección opcional del ``handle`` subyacente, útil para tests.
    # En producción es ``command_router.handle``.
    _handle: Callable[[str], str]

    def __init__(
        self,
        *,
        handle_fn: Callable[[str], str] | None = None,
    ) -> None:
        self._handle = handle_fn if handle_fn is not None else _cr.handle

    def handle(self, text: str) -> IntentResult:
        """Clasifica y resuelve un intent textual.

        Devuelve siempre un ``IntentResult``. Nunca lanza por
        excepciones del ``handle`` subyacente: las envuelve en un
        ``IntentResult`` con ``ok=False`` y ``output=str(exc)``, para
        no romper el endpoint HTTP upstream.
        """
        raw = text if text is not None else ""
        kind = _classify(raw)

        if kind == "empty":
            # Paridad con ``command_router.handle("")`` que devuelve
            # "ATLAS: vacío.".
            return IntentResult(
                kind="empty",
                ok=False,
                output="ATLAS: vacío.",
                raw_input=raw,
                meta={},
            )

        try:
            out = self._handle(raw)
        except Exception as exc:  # pragma: no cover — ruta defensiva
            return IntentResult(
                kind=kind,
                ok=False,
                output=str(exc),
                raw_input=raw,
                meta={},
            )

        return IntentResult(
            kind=kind,
            ok=True,
            output=out,
            raw_input=raw,
            meta={},
        )
