"""Click, type, hotkeys, scroll. Policy-gated; evidence recorded by caller."""
from __future__ import annotations

import time
from typing import Any, Dict, Optional, Tuple

from .status import _screen_deps_ok


def do_click(x: int, y: int) -> Dict[str, Any]:
    """Click at (x, y). Returns {ok, error}."""
    if not _screen_deps_ok():
        return {"ok": False, "error": "screen_deps_missing"}
    try:
        import pyautogui
        pyautogui.click(x, y)
        return {"ok": True, "error": None}
    except Exception as e:
        return {"ok": False, "error": str(e)}


def do_type(text: str, interval: Optional[float] = None) -> Dict[str, Any]:
    """Type text. interval = delay between keys (optional)."""
    if not _screen_deps_ok():
        return {"ok": False, "error": "screen_deps_missing"}
    try:
        import pyautogui
        if interval is not None:
            pyautogui.write(text, interval=interval)
        else:
            pyautogui.write(text)
        return {"ok": True, "error": None}
    except Exception as e:
        return {"ok": False, "error": str(e)}


def do_hotkey(*keys: str) -> Dict[str, Any]:
    """Hotkey e.g. do_hotkey('ctrl', 'c')."""
    if not _screen_deps_ok():
        return {"ok": False, "error": "screen_deps_missing"}
    try:
        import pyautogui
        pyautogui.hotkey(*keys)
        return {"ok": True, "error": None}
    except Exception as e:
        return {"ok": False, "error": str(e)}


def do_scroll(clicks: int, x: Optional[int] = None, y: Optional[int] = None) -> Dict[str, Any]:
    """Scroll. clicks > 0 up, < 0 down. Optional (x,y) for position."""
    if not _screen_deps_ok():
        return {"ok": False, "error": "screen_deps_missing"}
    try:
        import pyautogui
        if x is not None and y is not None:
            pyautogui.scroll(clicks, x=x, y=y)
        else:
            pyautogui.scroll(clicks)
        return {"ok": True, "error": None}
    except Exception as e:
        return {"ok": False, "error": str(e)}


def execute_action(action: str, payload: Dict[str, Any]) -> Dict[str, Any]:
    """
    Execute one of: click, type, hotkey, scroll.
    payload: click -> {x, y}; type -> {text, interval?}; hotkey -> {keys: []}; scroll -> {clicks, x?, y?}.
    Returns {ok, error, evidence_ref?}.
    """
    if action == "click":
        x = int(payload.get("x", 0))
        y = int(payload.get("y", 0))
        return do_click(x, y)
    if action == "type":
        return do_type(payload.get("text", ""), payload.get("interval"))
    if action == "hotkey":
        keys = payload.get("keys") or []
        if not keys:
            return {"ok": False, "error": "keys required"}
        return do_hotkey(*keys)
    if action == "scroll":
        clicks = int(payload.get("clicks", 0))
        return do_scroll(clicks, payload.get("x"), payload.get("y"))
    return {"ok": False, "error": f"unknown action: {action}"}
