"""PC Walker (Windows): navegación por la "casa" del PC.

Diseño (robusto):
- Prioriza navegación nativa de Windows (explorer.exe, ms-settings:, os.startfile)
  para evitar bloqueos de automatización por UI.
- Usa "manos" (pyautogui) + OCR solo cuando es necesario (clic por texto, hotkeys).
- Emite OPS + evidencia (screenshots) para trazabilidad.
"""
from __future__ import annotations

import os
import subprocess
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple


def _repo_root() -> Path:
    # Repo root = .../modules/humanoid/pc_walker.py -> parents[2] == repo
    return Path(__file__).resolve().parents[2]


def _snap_dir() -> Path:
    return _repo_root() / "snapshots" / "pc_walk"


def _ops(subsystem: str, message: str, level: str = "info", data: Optional[dict] = None, evidence_path: str = "") -> None:
    try:
        from modules.humanoid.comms.ops_bus import emit as ops_emit

        ops_emit(subsystem, message, level=level, data=data or {}, evidence_path=evidence_path or "")
    except Exception:
        pass


def _shell_open(target: str) -> Dict[str, Any]:
    """Abre una ruta/app/URI usando Windows sin UI automation."""
    t = (target or "").strip()
    if not t:
        return {"ok": False, "error": "missing_target"}
    try:
        low = t.lower()
        # Settings URI
        if low.startswith("ms-settings:"):
            os.startfile(t)  # type: ignore[attr-defined]
            return {"ok": True, "error": None}
        # URL
        if low.startswith("http://") or low.startswith("https://"):
            subprocess.Popen(["cmd", "/c", "start", "", t], shell=False)
            return {"ok": True, "error": None}
        # Path (local/UNC) -> Explorer (non-blocking)
        is_path_like = (
            (len(t) >= 3 and t[1] == ":" and (t[2] == "\\" or t[2] == "/"))
            or t.startswith("\\\\")
        )
        if is_path_like:
            subprocess.Popen(["explorer.exe", t], shell=False)
            return {"ok": True, "error": None}
        # Fallback: cmd start
        subprocess.Popen(["cmd", "/c", "start", "", t], shell=False)
        return {"ok": True, "error": None}
    except Exception as e:
        return {"ok": False, "error": str(e)}


def _capture_evidence(prefix: str = "pc") -> Tuple[Optional[str], Optional[str]]:
    """Capture full screen and save into snapshots/pc_walk. Returns (path, error)."""
    try:
        from modules.humanoid.screen.capture import capture_screen, save_capture_to_file

        png, err = capture_screen(region=None)
        if err or not png:
            return None, err or "no_capture"
        out_dir = _snap_dir()
        out_dir.mkdir(parents=True, exist_ok=True)
        path = save_capture_to_file(png, str(out_dir), prefix=prefix)
        return path, None
    except Exception as e:
        return None, str(e)


def _screen_act(action: str, payload: Dict[str, Any]) -> Dict[str, Any]:
    """Execute screen action with governance/rate-limit via screen.actions."""
    try:
        from modules.humanoid.screen.actions import execute_action

        return execute_action(action, payload or {})
    except Exception as e:
        return {"ok": False, "error": str(e)}


def _locate_text(query: str, region: Optional[Tuple[int, int, int, int]] = None) -> Dict[str, Any]:
    try:
        from modules.humanoid.screen.locator import locate

        return locate(query, region=region)
    except Exception as e:
        return {"ok": False, "matches": [], "error": str(e)}


def _center_of_bbox(bbox: List[int]) -> Tuple[int, int]:
    try:
        x, y, w, h = int(bbox[0]), int(bbox[1]), int(bbox[2]), int(bbox[3])
        return x + max(1, w) // 2, y + max(1, h) // 2
    except Exception:
        return 0, 0


_DESTRUCTIVE_HOTKEYS = {
    ("alt", "f4"),
    ("winleft", "l"),  # lock
    ("ctrl", "w"),
    ("ctrl", "shift", "esc"),  # task manager (strong)
    ("alt", "space"),  # window system menu (can close/move)
}


def _is_destructive_step(step: Dict[str, Any]) -> bool:
    kind = (step.get("kind") or "").strip().lower()
    if kind in ("shutdown", "restart", "delete", "format", "kill"):
        return True
    if kind == "hotkey":
        keys = step.get("keys") or step.get("payload", {}).get("keys") or []
        keys = [str(k).strip().lower() for k in (keys or []) if str(k).strip()]
        tup = tuple(keys)
        if tup in _DESTRUCTIVE_HOTKEYS:
            return True
    return False


def status() -> Dict[str, Any]:
    """Estado del Walker (deps + rutas)."""
    try:
        from modules.humanoid.screen.status import get_screen_status

        st = get_screen_status()
    except Exception:
        st = {"enabled": False, "error": "screen_status_unavailable"}
    return {
        "ok": True,
        "enabled": bool(st.get("enabled")),
        "screen": st,
        "snap_dir": str(_snap_dir()),
    }


@dataclass
class WalkResult:
    ok: bool
    steps: List[Dict[str, Any]]
    ms: int
    error: Optional[str] = None
    last_evidence: str = ""


def run_workflow(
    steps: List[Dict[str, Any]],
    allow_destructive: bool = False,
    sleep_ms: int = 250,
    step_timeout_s: float = 4.0,
) -> Dict[str, Any]:
    """Ejecuta una secuencia de pasos Windows (caminar por UI).

    Step schema (dict):
    - kind: hotkey|type|click_xy|click_text|wait|screenshot|ocr|open_app|run_dialog|open_path|alt_tab
    """
    t0 = time.perf_counter()
    done: List[Dict[str, Any]] = []
    last_ev = ""
    try:
        for i, st in enumerate(steps or []):
            kind = (st.get("kind") or "").strip().lower()
            if not kind:
                done.append({"i": i, "ok": False, "kind": "?", "error": "missing_kind"})
                continue

            if _is_destructive_step(st) and not allow_destructive:
                msg = f"Paso {i+1}: bloqueado por seguridad (destructive)."
                _ops("pc_walk", msg, level="med", data={"i": i, "kind": kind})
                done.append({"i": i, "ok": False, "kind": kind, "error": "destructive_blocked"})
                return WalkResult(ok=False, steps=done, ms=int((time.perf_counter() - t0) * 1000), error="destructive_blocked", last_evidence=last_ev).__dict__

            # default pacing
            if sleep_ms and sleep_ms > 0:
                time.sleep(max(0.0, float(sleep_ms) / 1000.0))

            if kind == "wait":
                ms = int(st.get("ms") or st.get("timeout_ms") or 500)
                time.sleep(max(0.0, float(ms) / 1000.0))
                done.append({"i": i, "ok": True, "kind": "wait", "ms": ms})
                continue

            if kind in ("hotkey",):
                keys = st.get("keys") or []
                keys = [str(k).strip().lower() for k in (keys or []) if str(k).strip()]
                r = _screen_act("hotkey", {"keys": keys})
                done.append({"i": i, "ok": bool(r.get("ok")), "kind": "hotkey", "keys": keys, "error": r.get("error") or err})
                _ops("pc_walk", f"Hotkey {'+'.join(keys)} => {'OK' if r.get('ok') else 'FAIL'}", level="info" if r.get("ok") else "med", data={"i": i, "keys": keys, "timeout": err == "timeout"})
                continue

            if kind in ("type",):
                text = str(st.get("text") or "")
                interval = st.get("interval")
                payload: Dict[str, Any] = {"text": text}
                if interval is not None:
                    payload["interval"] = interval
                r = _screen_act("type", payload)
                done.append({"i": i, "ok": bool(r.get("ok")), "kind": "type", "len": len(text), "error": r.get("error") or err})
                _ops("pc_walk", f"Type len={len(text)} => {'OK' if r.get('ok') else 'FAIL'}", level="info" if r.get("ok") else "med", data={"i": i, "len": len(text), "timeout": err == "timeout"})
                continue

            if kind in ("click_xy", "click"):
                x = int(st.get("x") or 0)
                y = int(st.get("y") or 0)
                r = _screen_act("click", {"x": x, "y": y})
                done.append({"i": i, "ok": bool(r.get("ok")), "kind": "click_xy", "x": x, "y": y, "error": r.get("error") or err})
                _ops("pc_walk", f"Click ({x},{y}) => {'OK' if r.get('ok') else 'FAIL'}", level="info" if r.get("ok") else "med", data={"i": i, "x": x, "y": y, "timeout": err == "timeout"})
                continue

            if kind == "click_text":
                q = str(st.get("query") or st.get("text") or "").strip()
                loc = _locate_text(q)
                matches = loc.get("matches") or []
                if not loc.get("ok") or not matches:
                    done.append({"i": i, "ok": False, "kind": "click_text", "query": q, "error": loc.get("error") or "not_found"})
                    _ops("pc_walk", f"Locate '{q}' => NOT FOUND", level="med", data={"i": i, "query": q})
                    continue
                bbox = matches[0].get("bbox") or [0, 0, 0, 0]
                cx, cy = _center_of_bbox(bbox)
                r = _screen_act("click", {"x": cx, "y": cy})
                done.append({"i": i, "ok": bool(r.get("ok")), "kind": "click_text", "query": q, "x": cx, "y": cy, "error": r.get("error") or err})
                _ops("pc_walk", f"ClickText '{q}' ({cx},{cy}) => {'OK' if r.get('ok') else 'FAIL'}", level="info" if r.get("ok") else "med", data={"i": i, "query": q, "x": cx, "y": cy, "timeout": err == "timeout"})
                continue

            if kind == "screenshot":
                name = str(st.get("name") or f"pc_{int(time.time())}")
                path, cerr = _capture_evidence(prefix=name[:24])
                if path:
                    last_ev = path
                    done.append({"i": i, "ok": True, "kind": "screenshot", "path": path})
                    _ops("pc_walk", f"Screenshot guardado: {path}", level="info", data={"i": i}, evidence_path=path)
                else:
                    done.append({"i": i, "ok": False, "kind": "screenshot", "error": (cerr or err or "capture_failed")})
                    _ops("pc_walk", f"Screenshot FAIL: {cerr or err}", level="med", data={"i": i, "timeout": err == "timeout"})
                continue

            if kind == "ocr":
                # OCR del screenshot actual
                path, cerr = _capture_evidence(prefix="ocr")
                if not path:
                    done.append({"i": i, "ok": False, "kind": "ocr", "error": (cerr or err or "capture_failed")})
                    continue
                try:
                    from modules.humanoid.screen.ocr import run_ocr
                    text, oerr = run_ocr(image_path=path)
                    ok = not bool(oerr)
                    done.append({"i": i, "ok": ok, "kind": "ocr", "chars": len(text), "error": oerr, "path": path})
                    _ops("pc_walk", f"OCR chars={len(text)} err={oerr or 'none'}", level="info" if ok else "med", data={"i": i, "chars": len(text)}, evidence_path=path)
                except Exception as e:
                    done.append({"i": i, "ok": False, "kind": "ocr", "error": str(e), "path": path})
                continue

            if kind == "alt_tab":
                count = int(st.get("count") or 1)
                # Alt+Tab n veces (moverse por ventanas)
                for _ in range(max(1, min(10, count))):
                    ok, _, err = _worker_call("screen_act", {"action": "hotkey", "payload": {"keys": ["alt", "tab"]}}, step_timeout_s)
                    if not ok and err == "timeout":
                        _disabled_until_ts = time.time() + 60
                        break
                    time.sleep(0.25)
                done.append({"i": i, "ok": True, "kind": "alt_tab", "count": count})
                _ops("pc_walk", f"Alt+Tab x{count}", level="info", data={"i": i, "count": count})
                continue

            if kind == "open_app":
                name = str(st.get("name") or st.get("app") or "").strip()
                if not name:
                    done.append({"i": i, "ok": False, "kind": "open_app", "error": "missing_name"})
                    continue
                # Preferir apertura nativa:
                target = "explorer.exe" if "explor" in name.lower() else name
                r = _shell_open(target)
                done.append({"i": i, "ok": bool(r.get("ok")), "kind": "open_app", "name": name, "error": r.get("error")})
                _ops("pc_walk", f"Abrir app (shell): {name} => {'OK' if r.get('ok') else 'FAIL'}", level="info" if r.get("ok") else "med", data={"i": i, "name": name})
                continue

            if kind == "run_dialog":
                # Win+R, type, Enter
                cmd = str(st.get("command") or st.get("text") or "").strip()
                if not cmd:
                    done.append({"i": i, "ok": False, "kind": "run_dialog", "error": "missing_command"})
                    continue
                ok, _, err = _worker_call("screen_act", {"action": "hotkey", "payload": {"keys": ["winleft", "r"]}}, step_timeout_s)
                if not ok and err == "timeout":
                    _disabled_until_ts = time.time() + 60
                    done.append({"i": i, "ok": False, "kind": "run_dialog", "command": cmd, "error": "timeout"})
                    return WalkResult(ok=False, steps=done, ms=int((time.perf_counter() - t0) * 1000), error="timeout", last_evidence=last_ev).__dict__
                time.sleep(0.35)
                _worker_call("screen_act", {"action": "type", "payload": {"text": cmd}}, step_timeout_s)
                time.sleep(0.1)
                _worker_call("screen_act", {"action": "hotkey", "payload": {"keys": ["enter"]}}, step_timeout_s)
                done.append({"i": i, "ok": True, "kind": "run_dialog", "command": cmd})
                _ops("pc_walk", f"Win+R: {cmd}", level="info", data={"i": i, "command": cmd})
                continue

            if kind == "open_path":
                # Explorador a ruta via Win+R
                path = str(st.get("path") or st.get("text") or "").strip()
                if not path:
                    done.append({"i": i, "ok": False, "kind": "open_path", "error": "missing_path"})
                    continue
                r = _shell_open(path)
                done.append({"i": i, "ok": bool(r.get("ok")), "kind": "open_path", "path": path, "error": r.get("error")})
                _ops("pc_walk", f"Abrir ruta (shell): {path} => {'OK' if r.get('ok') else 'FAIL'}", level="info" if r.get("ok") else "med", data={"i": i, "path": path})
                continue

            done.append({"i": i, "ok": False, "kind": kind, "error": "unsupported_kind"})
            _ops("pc_walk", f"Paso no soportado: {kind}", level="med", data={"i": i, "kind": kind})

        ms = int((time.perf_counter() - t0) * 1000)
        ok_all = all(bool(x.get("ok")) for x in done) if done else True
        return WalkResult(ok=ok_all, steps=done, ms=ms, error=None, last_evidence=last_ev).__dict__
    except Exception as e:
        ms = int((time.perf_counter() - t0) * 1000)
        return WalkResult(ok=False, steps=done, ms=ms, error=str(e), last_evidence=last_ev).__dict__

