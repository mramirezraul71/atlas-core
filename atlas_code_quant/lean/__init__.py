"""Scaffold de integración LEAN (F1).

No contiene integración real con QuantConnect.
"""

from .config import LeanConfig
from .launcher import LeanLauncher, LeanRunResult

__all__ = ["LeanConfig", "LeanLauncher", "LeanRunResult"]
