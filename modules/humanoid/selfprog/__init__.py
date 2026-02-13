"""Self-Programming: crear módulo, endpoint, skill, refactorizar, instalar dep, modificar routing."""
from __future__ import annotations

from .engine import (
    create_module,
    create_endpoint,
    add_skill,
    refactor_function,
    install_dependency,
    run_selfprog_flow,
)

__all__ = [
    "create_module",
    "create_endpoint",
    "add_skill",
    "refactor_function",
    "install_dependency",
    "run_selfprog_flow",
]
