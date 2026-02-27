"""Nervio: capa que conecta cerebro (Cursor/brain) con Nexus (ojos externos) y manos (mouse local).
El nervio llega hasta Nexus para visión y a la máquina local para mouse/teclado.
"""
from __future__ import annotations

from .eyes import eyes_capture, nerve_eyes_status
from .hands import hands_execute, hands_locate
from .feet import feet_execute, feet_status

__all__ = [
    "eyes_capture",
    "nerve_eyes_status",
    "hands_execute",
    "hands_locate",
    "feet_execute",
    "feet_status",
]
