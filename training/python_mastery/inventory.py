from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Tuple


@dataclass(frozen=True)
class InventoryResult:
    total_files: int
    total_size_bytes: int
    top_extensions: List[Tuple[str, int]]  # (ext, count)


def scan_inventory(root: Path, min_size_kb: int = 0) -> InventoryResult:
    """PY003: escanear archivos bajo root.

    Reglas (ver tests):
    - contar archivos regulares
    - sumar tamaños
    - top extensiones por cantidad (ext vacío -> "<noext>")
    - ignorar errores de permisos sin romper
    """
    raise NotImplementedError("Implementar en PY003")

