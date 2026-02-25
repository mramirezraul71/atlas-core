"""Macros: record/replay storage in SQLite."""
from __future__ import annotations

from .store import delete_macro, get_macro, list_macros, save_macro

__all__ = ["list_macros", "get_macro", "save_macro", "delete_macro"]
