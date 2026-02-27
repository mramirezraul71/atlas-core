"""Auto-Refactor: detecta funciones largas, código duplicado, módulos densos, deps sin usar. Refactor + smoke + rollback."""
from __future__ import annotations

from .engine import scan_targets, refactor_target

__all__ = ["scan_targets", "refactor_target"]
