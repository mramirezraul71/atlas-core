"""Reporte Cerebro: cada acción autónoma genera qué pasó, qué hice, por qué, resultado, impacto, rollback. Max 10 bullets."""
from __future__ import annotations

from .engine import emit_report, format_report

__all__ = ["emit_report", "format_report"]
