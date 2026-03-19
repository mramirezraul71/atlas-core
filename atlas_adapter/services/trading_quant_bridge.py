"""Bridge helpers between PUSH vision primitives and Atlas Code-Quant."""
from __future__ import annotations

import json
import os
import urllib.error
import urllib.request
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Optional

from modules.nexus_core import get_default_screen_calib_path, grasp, look_at_screen


def _now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def _clean(value: str | None) -> str:
    return (value or "").strip().rstrip("/")


def get_quant_api_base() -> str:
    """Return the canonical Atlas Code-Quant base URL."""
    return (
        _clean(os.getenv("ATLAS_QUANT_API_URL"))
        or _clean(os.getenv("QUANT_API_URL"))
        or "http://127.0.0.1:8792"
    )


def get_quant_api_key() -> str:
    """Return the local API key used to call Atlas Code-Quant."""
    return (
        _clean(os.getenv("ATLAS_QUANT_API_KEY"))
        or _clean(os.getenv("QUANT_API_KEY"))
        or "atlas-quant-local"
    )


def get_quant_handoff_prompt_path(repo_root: Optional[Path] = None) -> Path:
    """Return the prompt path used for native quant handoff."""
    base = repo_root or Path(__file__).resolve().parents[2]
    return base / "prompts" / "quant_native_camera_integration_handoff.md"


def _json_post(
    url: str,
    payload: dict[str, Any],
    *,
    headers: Optional[dict[str, str]] = None,
    timeout_sec: int = 20,
) -> tuple[bool, int, dict[str, Any], str]:
    body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
    req = urllib.request.Request(
        url=url,
        data=body,
        method="POST",
        headers={
            "Content-Type": "application/json",
            "Accept": "application/json",
            **(headers or {}),
        },
    )
    try:
        with urllib.request.urlopen(req, timeout=max(1, int(timeout_sec))) as response:
            raw = response.read().decode("utf-8", errors="replace")
            try:
                data = json.loads(raw) if raw else {}
            except Exception:
                data = {"raw": raw}
            return True, int(response.status), data, ""
    except urllib.error.HTTPError as exc:
        raw = exc.read().decode("utf-8", errors="replace")
        try:
            data = json.loads(raw) if raw else {}
        except Exception:
            data = {"raw": raw}
        return False, int(exc.code), data, str(exc)
    except Exception as exc:
        return False, 0, {}, str(exc)


def capture_quant_vision_context(
    *,
    capture_target: str = "nexus:camera",
    screen_target: Optional[dict[str, Any]] = None,
    calib_path: Optional[str] = None,
    source: str = "camera",
) -> dict[str, Any]:
    """Capture optional gaze alignment plus a vision snapshot for quant workflows."""
    snapshot: dict[str, Any] = {
        "captured_at": _now_iso(),
        "capture_target": str(capture_target or "nexus:camera"),
        "screen_target": screen_target if isinstance(screen_target, dict) else None,
        "calib_path": str(calib_path or get_default_screen_calib_path()),
        "source": str(source or "camera"),
        "gaze": None,
        "capture": None,
        "capture_ok": False,
    }

    if isinstance(screen_target, dict):
        x = float(screen_target.get("x", 0.0) or 0.0)
        y = float(screen_target.get("y", 0.0) or 0.0)
        zoom_value = screen_target.get("zoom")
        zoom = float(zoom_value) if zoom_value is not None else None
        snapshot["gaze"] = look_at_screen(
            x,
            y,
            zoom=zoom,
            calib_path=calib_path,
            source=source,
        )

    capture = grasp(str(capture_target or "nexus:camera"))
    snapshot["capture"] = capture
    snapshot["capture_ok"] = bool(capture.get("ok"))
    snapshot["capture_path"] = capture.get("path")
    snapshot["resource_id"] = capture.get("resource_id")
    return snapshot


def call_quant_operation_cycle(
    *,
    order: dict[str, Any],
    action: str = "evaluate",
    capture_context: bool = False,
    timeout_sec: int = 20,
    api_base: Optional[str] = None,
    api_key: Optional[str] = None,
) -> dict[str, Any]:
    """Call the quant operation cycle endpoint with operator-safe defaults."""
    base = _clean(api_base) or get_quant_api_base()
    key = api_key if api_key is not None else get_quant_api_key()
    ok, status, data, error = _json_post(
        f"{base}/api/v2/quant/operation/test-cycle",
        {
            "order": order if isinstance(order, dict) else {},
            "action": str(action or "evaluate"),
            "capture_context": bool(capture_context),
        },
        headers={"x-api-key": key} if key else {},
        timeout_sec=timeout_sec,
    )
    return {
        "ok": ok and bool((data or {}).get("ok", ok)),
        "http_ok": ok,
        "status_code": status,
        "api_base": base,
        "api_key_present": bool(key),
        "payload": data,
        "error": error or ((data or {}).get("error") if isinstance(data, dict) else ""),
    }


def run_quant_vision_cycle(
    *,
    order: dict[str, Any],
    action: str = "evaluate",
    include_vision: bool = True,
    capture_target: str = "nexus:camera",
    screen_target: Optional[dict[str, Any]] = None,
    calib_path: Optional[str] = None,
    source: str = "camera",
    quant_capture_context: bool = False,
    timeout_sec: int = 20,
    api_base: Optional[str] = None,
    api_key: Optional[str] = None,
) -> dict[str, Any]:
    """Run a quant operation cycle with optional external vision capture from PUSH."""
    vision_snapshot = (
        capture_quant_vision_context(
            capture_target=capture_target,
            screen_target=screen_target,
            calib_path=calib_path,
            source=source,
        )
        if include_vision
        else None
    )
    quant = call_quant_operation_cycle(
        order=order,
        action=action,
        capture_context=quant_capture_context,
        timeout_sec=timeout_sec,
        api_base=api_base,
        api_key=api_key,
    )
    return {
        "ok": bool(quant.get("ok")),
        "generated_at": _now_iso(),
        "vision_snapshot": vision_snapshot,
        "quant": quant,
    }


def get_quant_bridge_status(repo_root: Optional[Path] = None) -> dict[str, Any]:
    """Return bridge readiness without exposing secrets."""
    return {
        "ok": True,
        "quant_api_base": get_quant_api_base(),
        "quant_api_key_present": bool(get_quant_api_key()),
        "screen_gaze_calibration_path": str(get_default_screen_calib_path()),
        "handoff_prompt_path": str(get_quant_handoff_prompt_path(repo_root=repo_root)),
        "bridge_endpoints": {
            "vision_cycle": "/api/trading/quant/vision-cycle",
            "bridge_status": "/api/trading/quant/vision-bridge/status",
            "look_at_screen": "/api/primitives/nexus/look-at-screen",
        },
        "notes": [
            "This bridge keeps quant integration outside atlas_code_quant.",
            "Native quant camera integration can be done later using the handoff prompt.",
        ],
    }
