"""Screenshot: full screen or window (Windows). Prefer mss, fallback pyautogui."""
from __future__ import annotations

import base64
import io
import os
from pathlib import Path
from typing import Optional, Tuple

from .status import _screen_deps_ok


def capture_screen(region: Optional[Tuple[int, int, int, int]] = None, format: str = "png") -> Tuple[Optional[bytes], str]:
    """
    Capture screen. region = (x, y, w, h) or None for full screen.
    Returns (png_bytes, error_message). error_message empty on success.
    """
    if not _screen_deps_ok():
        return None, "screen module disabled: missing deps (mss/pyautogui)"

    try:
        if region:
            return _capture_region(region, format)
        return _capture_full(format)
    except Exception as e:
        return None, str(e)


def _capture_full(format: str) -> Tuple[Optional[bytes], str]:
    try:
        import mss
        with mss.mss() as sct:
            mon = sct.monitors[0]
            shot = sct.grab(mon)
            from PIL import Image
            img = Image.frombytes("RGB", shot.size, shot.bgra, "raw", "BGRX")
            buf = io.BytesIO()
            img.save(buf, format=format.upper())
            return buf.getvalue(), ""
    except ImportError:
        pass
    try:
        import pyautogui
        pil = pyautogui.screenshot()
        buf = io.BytesIO()
        pil.save(buf, format=format.upper())
        return buf.getvalue(), ""
    except Exception as e:
        return None, str(e)


def _capture_region(region: Tuple[int, int, int, int], format: str) -> Tuple[Optional[bytes], str]:
    x, y, w, h = region
    try:
        import mss
        with mss.mss() as sct:
            mon = {"left": x, "top": y, "width": w, "height": h}
            shot = sct.grab(mon)
            from PIL import Image
            img = Image.frombytes("RGB", shot.size, shot.bgra, "raw", "BGRX")
            buf = io.BytesIO()
            img.save(buf, format=format.upper())
            return buf.getvalue(), ""
    except ImportError:
        pass
    try:
        import pyautogui
        pil = pyautogui.screenshot(region=(x, y, w, h))
        buf = io.BytesIO()
        pil.save(buf, format=format.upper())
        return buf.getvalue(), ""
    except Exception as e:
        return None, str(e)


def save_capture_to_file(png_bytes: bytes, dir_path: str, prefix: str = "screen") -> str:
    """Save capture to dir_path/prefix_<timestamp>.png. Returns path."""
    from datetime import datetime, timezone
    Path(dir_path).mkdir(parents=True, exist_ok=True)
    ts = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")
    path = Path(dir_path) / f"{prefix}_{ts}.png"
    path.write_bytes(png_bytes)
    return str(path)


def capture_to_base64(region: Optional[Tuple[int, int, int, int]] = None) -> Tuple[Optional[str], str]:
    """Capture and return base64 string for API/vision. Returns (b64, error)."""
    data, err = capture_screen(region=region)
    if err or not data:
        return None, err or "no data"
    return base64.b64encode(data).decode("ascii"), ""
