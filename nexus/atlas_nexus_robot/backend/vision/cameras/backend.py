"""
Selección de backend OpenCV para captura de cámaras.

En Windows, CAP_MSMF suele ser inestable con ciertos drivers (errores tipo -1072875772).
Preferimos CAP_DSHOW cuando está disponible y dejamos CAP_MSMF/CAP_ANY como fallback.

Config:
- NEXUS_CAMERA_BACKEND: DSHOW | MSMF | ANY (forzar)
"""

from __future__ import annotations

import os
import sys
from typing import List, Optional

import cv2


def _backend_from_name(name: str) -> Optional[int]:
    n = (name or "").strip().upper()
    if not n:
        return None
    if n == "DSHOW" and hasattr(cv2, "CAP_DSHOW"):
        return cv2.CAP_DSHOW
    if n == "MSMF" and hasattr(cv2, "CAP_MSMF"):
        return cv2.CAP_MSMF
    if n == "ANY":
        return cv2.CAP_ANY
    return None


def preferred_backends() -> List[int]:
    """
    Lista ordenada de backends a probar.
    """
    forced = _backend_from_name(os.getenv("NEXUS_CAMERA_BACKEND", ""))
    if forced is not None:
        return [forced]

    if sys.platform == "win32":
        out: List[int] = []
        if hasattr(cv2, "CAP_DSHOW"):
            out.append(cv2.CAP_DSHOW)
        if hasattr(cv2, "CAP_MSMF"):
            out.append(cv2.CAP_MSMF)
        out.append(cv2.CAP_ANY)
        return out

    return [cv2.CAP_ANY]

