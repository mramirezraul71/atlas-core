from __future__ import annotations

from typing import Any, Iterable, List, Optional, Sequence, TypeVar

T = TypeVar("T")


def normalize_text(s: str) -> str:
    """PY001: Normaliza texto para comparaciones y logs.

    Reglas esperadas (ver tests):
    - strip de espacios extremos
    - colapsa whitespace interno a un solo espacio
    - lowercase
    """
    raise NotImplementedError("Implementar en PY001")


def chunk_list(items: Sequence[T], size: int) -> List[List[T]]:
    """PY001: Partir una secuencia en chunks de tamaño fijo."""
    raise NotImplementedError("Implementar en PY001")


def safe_get(d: dict, key: str, default: Optional[Any] = None) -> Any:
    """PY001: Obtener clave anidada por 'a.b.c' con default."""
    raise NotImplementedError("Implementar en PY001")

