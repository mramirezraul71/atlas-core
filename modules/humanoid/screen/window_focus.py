from __future__ import annotations

import os
import sys
from typing import Optional


def get_active_window_title() -> str:
    """
    Windows-only (ctypes): obtiene el título de la ventana en primer plano.
    Best-effort: si falla o no es Windows, retorna "".
    """
    if os.name != "nt":
        return ""
    try:
        import ctypes
        from ctypes import wintypes

        user32 = ctypes.WinDLL("user32", use_last_error=True)
        GetForegroundWindow = user32.GetForegroundWindow
        GetWindowTextLengthW = user32.GetWindowTextLengthW
        GetWindowTextW = user32.GetWindowTextW

        hwnd = GetForegroundWindow()
        if not hwnd:
            return ""
        length = GetWindowTextLengthW(hwnd)
        if length <= 0:
            # Algunas ventanas devuelven 0; intentamos buffer mínimo
            length = 256
        buf = ctypes.create_unicode_buffer(length + 1)
        GetWindowTextW(hwnd, buf, length + 1)
        return (buf.value or "").strip()
    except Exception:
        return ""


def active_window_matches(expected_substring: str) -> bool:
    exp = (expected_substring or "").strip().lower()
    if not exp:
        return True
    title = get_active_window_title().lower()
    return exp in title if title else False

