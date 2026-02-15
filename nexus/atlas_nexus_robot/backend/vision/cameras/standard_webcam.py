"""
Driver para webcam estándar UVC (cv2.VideoCapture).
"""

import cv2
import numpy as np
from typing import Optional, Tuple, Dict, Any

from .base import CameraBase
from .backend import preferred_backends


class StandardWebcam(CameraBase):
    """Webcam genérica vía OpenCV."""

    def __init__(
        self,
        index: int = 0,
        backend: Optional[int] = None,
        width: int = 640,
        height: int = 480,
        model_name: str = "Webcam estándar",
    ):
        self.index = index
        # En Windows, preferimos DSHOW por estabilidad; fallback a MSMF/ANY.
        self.backend = backend if backend is not None else preferred_backends()[0]
        self.width = width
        self.height = height
        self.model_name = model_name
        self._cap: Optional[cv2.VideoCapture] = None

    def open(self) -> bool:
        if self._cap is not None:
            return self._cap.isOpened()
        # Intento robusto: prueba backend preferido y fallback.
        tried = []
        for b in ([self.backend] + [x for x in preferred_backends() if x != self.backend]):
            tried.append(int(b))
            cap = cv2.VideoCapture(self.index, b)
            if cap.isOpened():
                self._cap = cap
                break
            try:
                cap.release()
            except Exception:
                pass
        if self._cap.isOpened() and (self.width or self.height):
            self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        return self._cap.isOpened() if self._cap else False

    def read(self) -> Tuple[bool, Optional[np.ndarray]]:
        if not self._cap or not self._cap.isOpened():
            return False, None
        ret, frame = self._cap.read()
        return ret, frame

    def release(self) -> None:
        if self._cap:
            self._cap.release()
            self._cap = None

    def get_properties(self) -> Dict[str, Any]:
        w, h = self.width, self.height
        if self._cap and self._cap.isOpened():
            w = int(self._cap.get(cv2.CAP_PROP_FRAME_WIDTH) or w)
            h = int(self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT) or h)
        return {
            "model": self.model_name,
            "driver": "uvc_standard",
            "resolution": [w, h],
            "capabilities": ["video"],
            "index": self.index,
        }

    @property
    def is_opened(self) -> bool:
        return self._cap is not None and self._cap.isOpened()
