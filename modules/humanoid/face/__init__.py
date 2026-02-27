"""Reconocimiento facial: detección y verificación."""
from __future__ import annotations

from .detector import face_detect, face_check_image

__all__ = ["face_detect", "face_check_image"]
