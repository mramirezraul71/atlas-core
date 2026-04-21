"""Tipos del subsistema de intents.

Define `IntentResult` (dataclass inmutable) y el alias `Kind`. Ambos
forman el contrato de `IntentRouter.handle(text) -> IntentResult`.

Regla de diseño: `IntentResult` no conoce `command_router` ni ninguna
estructura del brain core mayor. Es un tipo de dominio puro.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Literal


# Taxonomía aprobada (ver docs/atlas_push/PLAN_STEP_B.md §4).
Kind = Literal[
    "status",
    "doctor",
    "modules",
    "snapshot",
    "note.create",
    "note.append",
    "note.view",
    "natural.note.create",
    "natural.note.append",
    "natural.snapshot",
    "natural.modules",
    "inbox.fallback",
    "empty",
]


@dataclass(frozen=True)
class IntentResult:
    """Resultado de resolver un intent textual.

    Instancias inmutables: cualquier intento de mutación lanza
    ``dataclasses.FrozenInstanceError``.

    Campos:
        kind:      Clasificación del intent (ver ``Kind``).
        ok:        ``True`` salvo ``kind == "empty"`` o excepción al
                   ejecutar. **Nota**: en el Paso B, el endpoint HTTP
                   ``/intent`` ignora este campo y sigue devolviendo
                   ``ok: True`` por paridad estricta con el contrato
                   anterior. Este campo queda disponible para
                   consumidores futuros.
        output:    Texto devuelto por ``command_router.handle(text)``,
                   byte‑idéntico.
        raw_input: Texto original recibido por el router.
        meta:      Reservado para metadatos futuros. Siempre ``{}`` en B.
    """

    kind: Kind
    ok: bool
    output: str
    raw_input: str
    meta: dict[str, Any] = field(default_factory=dict)
