"""Screen module status: enabled/disabled from deps."""
from __future__ import annotations

from typing import Any, Dict

_screen_deps_cache: Dict[str, bool] = {}


def _screen_deps_ok() -> bool:
    """True if at least mss or pyautogui available for capture."""
    if "capture" in _screen_deps_cache:
        return _screen_deps_cache["capture"]
    try:
        import mss
        _screen_deps_cache["capture"] = True
        return True
    except ImportError:
        pass
    try:
        import pyautogui
        _screen_deps_cache["capture"] = True
        return True
    except ImportError:
        pass
    _screen_deps_cache["capture"] = False
    return False


def get_screen_status() -> Dict[str, Any]:
    """Status for GET /screen/status. Always responds; enabled=false when deps missing."""
    from modules.humanoid.deps_checker import check_screen
    dep = check_screen()
    return {
        "ok": True,
        "enabled": dep.get("available", False),
        "module": "screen",
        "missing_deps": dep.get("missing_deps", []),
        "suggested": dep.get("suggested", ""),
    }
