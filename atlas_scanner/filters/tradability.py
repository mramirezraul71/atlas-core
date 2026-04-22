from __future__ import annotations

from typing import Tuple

from atlas_scanner.models import SymbolSnapshot


def apply_tradability_filter(
    symbols: Tuple[SymbolSnapshot, ...],
) -> Tuple[Tuple[SymbolSnapshot, ...], Tuple[str, ...]]:
    """
    S0 stub: comportamiento neutro.
    Devuelve todos los simbolos como kept y ningun simbolo rechazado.
    """
    return (symbols, ())

