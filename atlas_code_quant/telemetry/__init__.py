"""Módulo de telemetría (F1 scaffold)."""

from .logger import get_quant_logger
from .metrics import InMemoryCounter

__all__ = ["get_quant_logger", "InMemoryCounter"]
