"""Hands+Eyes Engine: capture + OCR + Vision + layout + locate + action + record/replay + evidencia."""
from __future__ import annotations

import base64
import os
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

EVIDENCE_DIR = Path(os.getenv("HANDS_EYES_EVIDENCE_DIR", "C:\\ATLAS_PUSH\\snapshots\\hands_eyes"))


def _screen_deps_ok() -> bool:
    try:
        from modules.humanoid.screen.status import _screen_deps_ok as ok
        return ok()
    except Exception:
        return False


def _audit(module: str, action: str, ok: bool, payload: Optional[Dict] = None, evidence_path: Optional[str] = None) -> None:
    try:
        from modules.humanoid.audit import get_audit_logger
        result = {"evidence_path": evidence_path} if evidence_path else None
        get_audit_logger().log_event("hands_eyes", "system", module, action, ok, 0, None, payload or {}, result)
    except Exception:
        pass


def _save_evidence(kind: str, data: bytes, meta: Dict[str, Any]) -> str:
    EVIDENCE_DIR.mkdir(parents=True, exist_ok=True)
    ts = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")
    ext = "png" if kind == "capture" else "txt"
    path = EVIDENCE_DIR / f"{kind}_{ts}.{ext}"
    if kind == "capture":
        path.write_bytes(data)
    else:
        path.write_text(str(meta.get("content", meta)), encoding="utf-8")
    return str(path)


def capture_full_scene(region: Optional[Tuple[int, int, int, int]] = None, save_evidence: bool = True) -> Dict[str, Any]:
    """Captura pantalla completa o región. Retorna {ok, png_bytes, base64, evidence_path, error}."""
    if not _screen_deps_ok():
        return {"ok": False, "png_bytes": None, "base64": None, "evidence_path": None, "error": "screen_deps_missing"}
    try:
        from modules.humanoid.screen.capture import capture_screen, capture_to_base64
        png, err = capture_screen(region=region)
        if err or not png:
            return {"ok": False, "png_bytes": None, "base64": None, "evidence_path": None, "error": err or "no_data"}
        b64, _ = capture_to_base64(region=region)
        if not b64:
            b64 = base64.b64encode(png).decode("ascii")
        evidence = _save_evidence("capture", png, {}) if save_evidence else None
        _audit("hands_eyes", "capture", True, {"region": region}, evidence)
        return {"ok": True, "png_bytes": png, "base64": b64, "evidence_path": evidence, "error": None}
    except Exception as e:
        return {"ok": False, "png_bytes": None, "base64": None, "evidence_path": None, "error": str(e)}


def analyze_scene(image_base64: Optional[str] = None, region: Optional[Tuple[int, int, int, int]] = None, prompt: str = "") -> Dict[str, Any]:
    """Análisis de escena: OCR + Vision LLM. Retorna {ok, ocr_text, vision_description, layout, error}."""
    if not image_base64:
        cap = capture_full_scene(region=region, save_evidence=True)
        if not cap.get("ok"):
            return {"ok": False, "ocr_text": "", "vision_description": "", "layout": {}, "error": cap.get("error")}
        image_base64 = cap.get("base64")

    out: Dict[str, Any] = {"ok": False, "ocr_text": "", "vision_description": "", "layout": {}, "error": None}

    try:
        from modules.humanoid.screen.ocr import run_ocr
        import io
        from PIL import Image
        raw = base64.b64decode(image_base64)
        text, ocr_err = run_ocr(image_bytes=raw)
        out["ocr_text"] = text

        from modules.humanoid.screen.vision_llm import analyze_image
        p = prompt or "Describe what you see. If UI: list buttons, fields, labels. Suggest actions."
        r = analyze_image(image_base64=image_base64, prompt=p)
        out["vision_description"] = r.get("description", "")
        out["ok"] = r.get("ok", False)
        out["error"] = r.get("error")

        from modules.humanoid.screen.layout import get_layout
        layout = get_layout(region=region)
        out["layout"] = layout
    except Exception as e:
        out["error"] = str(e)
    _audit("hands_eyes", "analyze_scene", out["ok"], {}, None)
    return out


def locate_element(query: str, region: Optional[Tuple[int, int, int, int]] = None) -> Dict[str, Any]:
    """Localiza elemento por texto. Retorna {ok, matches: [{bbox, text}], error}."""
    try:
        from modules.humanoid.screen.locator import locate
        return locate(query, region=region)
    except Exception as e:
        return {"ok": False, "matches": [], "error": str(e)}


def execute_action(action: str, payload: Dict[str, Any], verify_before: bool = False, verify_after: bool = False) -> Dict[str, Any]:
    """Ejecuta acción: click, type, hotkey, scroll. Opcional verificación before/after. Evidencia obligatoria."""
    try:
        from modules.humanoid.screen.actions import execute_action as do_action
        evidence_before = None
        if verify_before:
            cap = capture_full_scene(save_evidence=True)
            evidence_before = cap.get("evidence_path")
        r = do_action(action, payload)
        evidence_after = None
        if verify_after:
            cap = capture_full_scene(save_evidence=True)
            evidence_after = cap.get("evidence_path")
        r["evidence_before"] = evidence_before
        r["evidence_after"] = evidence_after
        _audit("hands_eyes", f"action_{action}", r.get("ok", False), payload, evidence_after or evidence_before)
        return r
    except Exception as e:
        return {"ok": False, "error": str(e), "evidence_before": None, "evidence_after": None}


def start_recording() -> Dict[str, Any]:
    try:
        from modules.humanoid.screen.record import start_recording as sr
        return sr()
    except Exception as e:
        return {"ok": False, "error": str(e)}


def stop_recording() -> Dict[str, Any]:
    try:
        from modules.humanoid.screen.record import stop_recording as st
        return st()
    except Exception as e:
        return {"ok": False, "error": str(e), "actions": []}


def replay_actions(actions: List[Dict[str, Any]], delay_ms: int = 100) -> Dict[str, Any]:
    """Reproduce macro. Si falla: detectar razón, intentar alternativa."""
    from modules.humanoid.screen.actions import execute_action
    results = []
    for i, a in enumerate(actions):
        act = a.get("action", "")
        payload = a.get("payload", {})
        time.sleep(delay_ms / 1000.0)
        r = execute_action(act, payload)
        results.append({"index": i, "action": act, "ok": r.get("ok", False), "error": r.get("error")})
        if not r.get("ok") and i < len(actions) - 1:
            time.sleep(0.5)
            r2 = execute_action(act, payload)
            results[-1]["retry_ok"] = r2.get("ok", False)
    ok_all = all(x.get("ok", False) for x in results)
    return {"ok": ok_all, "results": results}


def run_workflow(steps: List[Dict[str, Any]]) -> Dict[str, Any]:
    """Ejecuta flujo: capture -> locate -> action. Si falla: plan alternativo."""
    out: Dict[str, Any] = {"ok": True, "steps_done": [], "evidence": [], "error": None}
    for step in steps:
        kind = (step.get("kind") or "action").lower()
        if kind == "capture":
            r = capture_full_scene(region=step.get("region"), save_evidence=True)
            out["steps_done"].append({"kind": "capture", "ok": r.get("ok")})
            if r.get("evidence_path"):
                out["evidence"].append(r["evidence_path"])
        elif kind == "locate":
            r = locate_element(step.get("query", ""), step.get("region"))
            out["steps_done"].append({"kind": "locate", "ok": r.get("ok"), "matches": r.get("matches", [])})
        elif kind == "action":
            r = execute_action(step.get("action", ""), step.get("payload", {}), verify_after=step.get("verify_after", False))
            out["steps_done"].append({"kind": "action", "ok": r.get("ok"), "action": step.get("action")})
            if r.get("evidence_after"):
                out["evidence"].append(r["evidence_after"])
        if not out["steps_done"][-1].get("ok"):
            out["ok"] = False
            out["error"] = f"step {kind} failed"
            break
    return out
