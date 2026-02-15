"""Click, type, hotkeys, scroll. Policy-gated; evidence recorded by caller."""
from __future__ import annotations

import time
from typing import Any, Dict, Optional, Tuple

from .status import _screen_deps_ok


_rate_window_s = 1.0
_recent_actions: list[float] = []


def _rate_limit_ok(weight: int = 1) -> bool:
    """
    Rate limit simple (memoria local del proceso) para evitar bucles de input.
    Config: HANDS_MAX_ACTIONS_PER_SEC (default 6).
    """
    try:
        import os

        mx = int((os.getenv("HANDS_MAX_ACTIONS_PER_SEC") or "6").strip() or "6")
        if mx < 1:
            mx = 1
        if mx > 30:
            mx = 30
    except Exception:
        mx = 6
    now = time.time()
    # purge
    cutoff = now - _rate_window_s
    while _recent_actions and _recent_actions[0] < cutoff:
        _recent_actions.pop(0)
    # add weight
    if len(_recent_actions) + max(1, int(weight)) > mx:
        return False
    for _ in range(max(1, int(weight))):
        _recent_actions.append(now)
    return True


def _governance_allows_input() -> Tuple[bool, Optional[str]]:
    """
    Bloqueo duro en EMERGENCY_STOP. En modo governed no bloqueamos por defecto
    (no hay flujo de approvals para input interactivo), pero queda preparado.
    """
    try:
        from modules.humanoid.governance.state import get_emergency_stop

        if get_emergency_stop():
            return False, "emergency_stop"
    except Exception:
        # si governance falla, no bloquear por accidente; solo aplicamos rate-limit
        pass
    return True, None


def _guard(action_kind: str, payload: Optional[Dict[str, Any]] = None, weight: int = 1) -> Optional[str]:
    ok, reason = _governance_allows_input()
    if not ok:
        return reason or "blocked"
    if not _rate_limit_ok(weight=weight):
        return "rate_limited"
    # validaciones ligeras por tipo
    if action_kind == "click" and payload:
        try:
            x = int(payload.get("x", 0))
            y = int(payload.get("y", 0))
            import pyautogui

            w, h = pyautogui.size()
            if x < 0 or y < 0 or x >= w or y >= h:
                return "out_of_bounds"
        except Exception:
            # no bloquear si no podemos validar bounds
            pass
    if action_kind == "type" and payload:
        t = str(payload.get("text", "") or "")
        if len(t) > 2000:
            return "text_too_long"
    return None


def do_click(x: int, y: int) -> Dict[str, Any]:
    """Click at (x, y). Returns {ok, error}."""
    if not _screen_deps_ok():
        return {"ok": False, "error": "screen_deps_missing"}
    err = _guard("click", {"x": x, "y": y}, weight=1)
    if err:
        return {"ok": False, "error": err}
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
    # typing es más "pesado"
    err = _guard("type", {"text": text}, weight=2)
    if err:
        return {"ok": False, "error": err}
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
    err = _guard("hotkey", {"keys": list(keys)}, weight=1)
    if err:
        return {"ok": False, "error": err}
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
    err = _guard("scroll", {"clicks": clicks, "x": x, "y": y}, weight=1)
    if err:
        return {"ok": False, "error": err}
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
        r = do_click(x, y)
        try:
            from modules.humanoid.comms.ops_bus import emit as ops_emit
            ops_emit("hands", f"Click en ({x},{y}) => {'OK' if r.get('ok') else 'FAIL'}", level="med" if not r.get("ok") else "info", data={"action": "click", "x": x, "y": y, "ok": r.get("ok")})
        except Exception:
            pass
        return r
    if action == "type":
        text = payload.get("text", "") or ""
        r = do_type(text, payload.get("interval"))
        try:
            from modules.humanoid.comms.ops_bus import emit as ops_emit
            ops_emit("hands", f"Type len={len(str(text))} => {'OK' if r.get('ok') else 'FAIL'}", level="med" if not r.get("ok") else "info", data={"action": "type", "len": len(str(text)), "ok": r.get("ok")})
        except Exception:
            pass
        return r
    if action == "hotkey":
        keys = payload.get("keys") or []
        if not keys:
            return {"ok": False, "error": "keys required"}
        r = do_hotkey(*keys)
        try:
            from modules.humanoid.comms.ops_bus import emit as ops_emit
            ops_emit("hands", f"Hotkey {'+'.join(keys)} => {'OK' if r.get('ok') else 'FAIL'}", level="med" if not r.get("ok") else "info", data={"action": "hotkey", "keys": keys, "ok": r.get("ok")})
        except Exception:
            pass
        return r
    if action == "scroll":
        clicks = int(payload.get("clicks", 0))
        r = do_scroll(clicks, payload.get("x"), payload.get("y"))
        try:
            from modules.humanoid.comms.ops_bus import emit as ops_emit
            ops_emit("hands", f"Scroll {clicks} => {'OK' if r.get('ok') else 'FAIL'}", level="med" if not r.get("ok") else "info", data={"action": "scroll", "clicks": clicks, "ok": r.get("ok")})
        except Exception:
            pass
        return r
    return {"ok": False, "error": f"unknown action: {action}"}
