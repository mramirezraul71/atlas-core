"""App scaffolder: generate project structures and RUNBOOK."""
from __future__ import annotations

from .generator import generate
from .runbook import generate_runbook
from .templates import get_template

__all__ = ["generate", "generate_runbook", "get_template"]
