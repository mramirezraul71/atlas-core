"""Click, type, hotkeys, scroll. Policy-gated; evidence recorded by caller."""
from __future__ import annotations

import time
from typing import Any, Dict, Optional, Tuple

from .status import _screen_deps_ok
from .window_focus import active_window_matches, active_window_process_matches, get_active_window_title, get_active_window_process_path


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
    # Preflight: ventana activa esperada (evita clicks fuera de contexto)
    try:
        expected = ""
        expected_proc = ""
        if payload:
            expected = str(payload.get("expected_window") or payload.get("expected_window_title") or "")
            expected_proc = str(payload.get("expected_process") or payload.get("expected_exe") or payload.get("expected_process_path") or "")
        if not expected:
            import os
            expected = (os.getenv("HANDS_EXPECT_WINDOW_TITLE") or "").strip()
        if not expected_proc:
            import os
            expected_proc = (os.getenv("HANDS_EXPECT_WINDOW_PROCESS") or "").strip()
        if expected and not active_window_matches(expected):
            return "wrong_active_window"
        if expected_proc and not active_window_process_matches(expected_proc):
            return "wrong_active_process"
    except Exception:
        pass
    # Preflight: confirmación visual para acciones destructivas (OCR)
    try:
        destructive = bool(payload.get("destructive")) if payload else False
        confirm_text = str(payload.get("confirm_text") or "") if payload else ""
        if destructive or confirm_text:
            from modules.humanoid.governance.gates import decide
            d = decide("screen_act_destructive", context={"target": confirm_text, "expected_window": expected})
            if d.needs_approval and not d.allow:
                try:
                    # Evidencia (antes): screenshot + ventana activa
                    evidence_path = ""
                    try:
                        from pathlib import Path
                        from .capture import capture_screen, save_capture_to_file
                        root = Path(__file__).resolve().parents[3]
                        png, err = capture_screen()
                        if png and not err:
                            evidence_path = save_capture_to_file(png, str(root / "snapshots" / "hands_eyes" / "preflight"), prefix="preflight")
                    except Exception:
                        evidence_path = ""
                    from modules.humanoid.approvals.service import create as create_approval
                    from modules.humanoid.governance.dynamic_risk import assess_action
                    a = assess_action("screen_act_destructive", {"confirm_text": confirm_text})
                    cr = create_approval(
                        "screen_act_destructive",
                        {
                            "confirm_text": confirm_text,
                            "expected_window": expected,
                            "expected_process": expected_proc,
                            "active_window": get_active_window_title(),
                            "active_process_path": get_active_window_process_path(),
                            "risk": a.risk,
                            "reason": a.reason,
                            "signature": a.signature,
                            "evidence_path": evidence_path,
                        },
                    )
                    # Enviar evidencia por OPS (Telegram/Audio) si existe
                    try:
                        if evidence_path:
                            from modules.humanoid.comms.ops_bus import emit as ops_emit
                            ops_emit("approval", "Confirmación requerida para acción destructiva (evidencia adjunta).", level="high", evidence_path=evidence_path)
                    except Exception:
                        pass
                    return f"approval_required:{cr.get('approval_id') or ''}".rstrip(":")
                except Exception:
                    return "approval_required"
            if confirm_text:
                from .locator import locate
                m = locate(confirm_text)
                if not m.get("ok") or not (m.get("matches") or []):
                    return "confirm_text_not_found"
    except Exception:
        pass
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
    if action_kind == "move" and payload:
        try:
            x = int(payload.get("x", 0))
            y = int(payload.get("y", 0))
            import pyautogui

            w, h = pyautogui.size()
            if x < 0 or y < 0 or x >= w or y >= h:
                return "out_of_bounds"
        except Exception:
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


def do_move(x: int, y: int, duration: float = 0.2) -> Dict[str, Any]:
    """Move mouse to (x,y). Returns {ok, error}."""
    if not _screen_deps_ok():
        return {"ok": False, "error": "screen_deps_missing"}
    err = _guard("move", {"x": x, "y": y}, weight=1)
    if err:
        return {"ok": False, "error": err}
    try:
        import pyautogui
        pyautogui.moveTo(int(x), int(y), duration=float(duration or 0.0))
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
        # Permite preflight: expected_window_title / confirm_text / destructive
        err = _guard("click", {"x": x, "y": y, **(payload or {})}, weight=1)
        if err:
            # Enriquecer error con contexto humano mínimo
            try:
                title = get_active_window_title()
            except Exception:
                title = ""
            try:
                proc = get_active_window_process_path()
            except Exception:
                proc = ""
            out = {"ok": False, "error": err, "active_window": title[:120] if title else ""}
            if proc:
                out["active_process_path"] = proc[:220]
            # Si la guard devolvió approval_required:<id>, devolver id separado
            if err.startswith("approval_required:"):
                out["approval_id"] = err.split(":", 1)[1]
                out["error"] = "approval_required"
            return out
        # Evidencia before/after (opcional; default off para no spamear)
        evidence = bool(payload.get("record_evidence") or payload.get("evidence"))
        before_path = ""
        after_path = ""
        if evidence:
            try:
                from pathlib import Path
                from .capture import capture_screen, save_capture_to_file
                root = Path(__file__).resolve().parents[3]
                png, e = capture_screen()
                if png and not e:
                    before_path = save_capture_to_file(png, str(root / "snapshots" / "hands_eyes" / "actions"), prefix="before")
            except Exception:
                before_path = ""
        r = do_click(x, y)
        if evidence:
            try:
                time.sleep(0.25)
                from pathlib import Path
                from .capture import capture_screen, save_capture_to_file
                root = Path(__file__).resolve().parents[3]
                png, e = capture_screen()
                if png and not e:
                    after_path = save_capture_to_file(png, str(root / "snapshots" / "hands_eyes" / "actions"), prefix="after")
            except Exception:
                after_path = ""
            if before_path or after_path:
                r["evidence"] = {"before": before_path or None, "after": after_path or None}
        try:
            from modules.humanoid.comms.ops_bus import emit as ops_emit
            ops_emit("hands", f"Click en ({x},{y}) => {'OK' if r.get('ok') else 'FAIL'}", level="med" if not r.get("ok") else "info", data={"action": "click", "x": x, "y": y, "ok": r.get("ok")})
        except Exception:
            pass
        return r
    if action == "move":
        x = int(payload.get("x", 0))
        y = int(payload.get("y", 0))
        dur = float(payload.get("duration", 0.2) or 0.2)
        r = do_move(x, y, duration=dur)
        try:
            from modules.humanoid.comms.ops_bus import emit as ops_emit
            ops_emit("hands", f"Move a ({x},{y}) => {'OK' if r.get('ok') else 'FAIL'}", level="med" if not r.get("ok") else "info", data={"action": "move", "x": x, "y": y, "ok": r.get("ok")})
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
