"""Reconciler — F6 (esqueleto).

Compara las posiciones internas (paper o tradier) contra una fuente de verdad
externa y devuelve un diff auditable. En el esqueleto, la fuente externa se
inyecta por parámetro para mantener pureza de tests.
"""
from __future__ import annotations

from dataclasses import dataclass, field


@dataclass(slots=True)
class ReconcileItem:
    symbol: str
    internal_qty: int
    external_qty: int

    @property
    def delta(self) -> int:
        return self.external_qty - self.internal_qty


@dataclass(slots=True)
class ReconcileReport:
    items: list[ReconcileItem] = field(default_factory=list)

    @property
    def has_drift(self) -> bool:
        return any(i.delta != 0 for i in self.items)


def reconcile(
    internal: dict[str, int],
    external: dict[str, int],
) -> ReconcileReport:
    """Devuelve un reporte con todos los símbolos vistos en cualquier lado."""
    seen = set(internal) | set(external)
    items = [
        ReconcileItem(
            symbol=sym,
            internal_qty=int(internal.get(sym, 0)),
            external_qty=int(external.get(sym, 0)),
        )
        for sym in sorted(seen)
    ]
    return ReconcileReport(items=items)
