"""Auto-Refactor: detecta funciones largas, código duplicado, módulos densos, deps sin usar. Refactor + smoke + rollback."""
from __future__ import annotations

from .engine import refactor_target, scan_targets

__all__ = ["scan_targets", "refactor_target"]
