"""Governed vision state for paper-first ATLAS operational tests."""
from __future__ import annotations

import json
import os
import sys
from copy import deepcopy
from datetime import datetime
from pathlib import Path
from typing import Any
from urllib import error as urlerror
from urllib import request as urlrequest

from config.settings import settings


_DEFAULT_STATE = {
    "provider": "direct_nexus",
    "operator_present": True,
    "screen_integrity_ok": True,
    "last_capture_path": None,
    "last_capture_at": None,
    "last_capture_note": "",
    "notes": "Modo de camara directa robot para pruebas simuladas.",
}


class SensorVisionService:
    def __init__(self, state_path: Path | None = None) -> None:
        base_dir = settings.data_dir.parent / "operation"
        base_dir.mkdir(parents=True, exist_ok=True)
        self.state_path = state_path or (base_dir / "sensor_vision_state.json")
        self.snapshots_dir = base_dir / "snapshots"
        self.snapshots_dir.mkdir(parents=True, exist_ok=True)
        self._ensure_state()

    def _ensure_state(self) -> None:
        if not self.state_path.exists():
            self._save(_DEFAULT_STATE)

    def _load(self) -> dict[str, Any]:
        self._ensure_state()
        try:
            data = json.loads(self.state_path.read_text(encoding="utf-8"))
            if isinstance(data, dict):
                merged = deepcopy(_DEFAULT_STATE)
                merged.update(data)
                return merged
        except Exception:
            pass
        return deepcopy(_DEFAULT_STATE)

    def _save(self, payload: dict[str, Any]) -> dict[str, Any]:
        merged = deepcopy(_DEFAULT_STATE)
        merged.update(payload or {})
        self.state_path.write_text(json.dumps(merged, ensure_ascii=True, indent=2), encoding="utf-8")
        return merged

    @staticmethod
    def _clean(value: str | None) -> str:
        return (value or "").strip().rstrip("/")

    def _push_base_url(self) -> str:
        return self._clean(os.getenv("ATLAS_PUSH_BASE_URL")) or "http://127.0.0.1:8791"

    def _robot_base_url(self) -> str:
        return self._clean(os.getenv("NEXUS_ROBOT_API_URL")) or "http://127.0.0.1:8002"

    def _push_timeout_sec(self) -> int:
        raw = (os.getenv("ATLAS_PUSH_TIMEOUT_SEC") or "").strip()
        try:
            return max(1, min(int(raw), 120))
        except Exception:
            return 12

    def _push_capture_timeout_sec(self) -> int:
        raw = (os.getenv("ATLAS_PUSH_CAPTURE_TIMEOUT_SEC") or "").strip()
        try:
            return max(5, min(int(raw), 180))
        except Exception:
            return 45

    def _push_calib_path(self) -> str | None:
        raw = (os.getenv("ATLAS_SCREEN_GAZE_CALIB_PATH") or "").strip()
        if raw:
            return raw
        default_path = Path(__file__).resolve().parents[2] / "config" / "screen_gaze_calibration.json"
        return str(default_path) if default_path.exists() else None

    def _request_json(
        self,
        method: str,
        path: str,
        *,
        payload: dict[str, Any] | None = None,
        timeout_sec: int | None = None,
    ) -> tuple[bool, int, dict[str, Any], str]:
        base = self._push_base_url()
        url = f"{base}{path}"
        data = None
        headers = {"Accept": "application/json"}
        if payload is not None:
            data = json.dumps(payload, ensure_ascii=False).encode("utf-8")
            headers["Content-Type"] = "application/json"
        req = urlrequest.Request(url=url, data=data, method=method.upper(), headers=headers)
        try:
            with urlrequest.urlopen(req, timeout=timeout_sec or self._push_timeout_sec()) as response:
                raw = response.read().decode("utf-8", errors="replace")
                parsed = json.loads(raw) if raw else {}
                return True, int(response.status), parsed if isinstance(parsed, dict) else {"raw": parsed}, ""
        except urlerror.HTTPError as exc:
            raw = exc.read().decode("utf-8", errors="replace")
            try:
                parsed = json.loads(raw) if raw else {}
            except Exception:
                parsed = {"raw": raw}
            return False, int(exc.code), parsed if isinstance(parsed, dict) else {"raw": parsed}, str(exc)
        except Exception as exc:
            return False, 0, {}, str(exc)

    @staticmethod
    def _default_screen_target(calib_path: str | None) -> dict[str, Any] | None:
        if not calib_path:
            return None
        try:
            payload = json.loads(Path(calib_path).read_text(encoding="utf-8"))
            width = float(payload.get("W", 0.0) or 0.0)
            height = float(payload.get("H", 0.0) or 0.0)
            if width <= 0 or height <= 0:
                return None
            return {
                "x": width / 2.0,
                "y": height / 2.0,
                "zoom": float(payload.get("zoom_center", 1.0) or 1.0),
            }
        except Exception:
            return None

    @staticmethod
    def _ensure_repo_root_on_path() -> None:
        repo_root = Path(__file__).resolve().parents[2]
        repo_root_str = str(repo_root)
        if repo_root_str not in sys.path:
            sys.path.insert(0, repo_root_str)

    def _direct_nexus_available(self) -> bool:
        try:
            req = urlrequest.Request(
                url=f"{self._robot_base_url()}/api/health",
                method="GET",
                headers={"Accept": "application/json"},
            )
            with urlrequest.urlopen(req, timeout=3) as response:
                raw = response.read().decode("utf-8", errors="replace")
                payload = json.loads(raw) if raw else {}
                if int(response.status) != 200:
                    return False
                if not isinstance(payload, dict):
                    return True
                if payload.get("ok") is True:
                    return True
                return str(payload.get("status") or "").strip().lower() in {"healthy", "ok", "ready"}
        except Exception:
            return False

    def _direct_nexus_capture(
        self,
        *,
        capture_target: str = "nexus:camera",
        screen_target: dict[str, Any] | None = None,
        calib_path: str | None = None,
        source: str = "camera",
        bridge_status: dict[str, Any] | None = None,
        provider_name: str = "direct_nexus",
    ) -> dict[str, Any]:
        record: dict[str, Any] = {
            "provider": str(provider_name or "direct_nexus"),
            "bridge_status": bridge_status or {"ok": False},
            "capture_backend": "direct_nexus" if str(provider_name or "") == "direct_nexus" else "direct_nexus_fallback",
            "capture_target": str(capture_target or "nexus:camera"),
            "screen_target": screen_target if isinstance(screen_target, dict) else None,
            "calib_path": str(calib_path) if calib_path else None,
            "source": str(source or "camera"),
            "gaze": None,
            "capture": None,
            "capture_ok": False,
            "capture_path": None,
            "resource_id": None,
        }
        try:
            self._ensure_repo_root_on_path()
            from modules.nexus_core.primitives import grasp, look_at_screen

            if isinstance(screen_target, dict):
                gaze_result = look_at_screen(
                    float(screen_target.get("x", 0.0) or 0.0),
                    float(screen_target.get("y", 0.0) or 0.0),
                    zoom=float(screen_target.get("zoom", 1.0) or 1.0),
                    calib_path=calib_path,
                    source=source,
                )
                record["gaze"] = gaze_result if isinstance(gaze_result, dict) else {"raw": gaze_result}

            capture_result = grasp(str(capture_target or "nexus:camera"))
            capture_dict = capture_result if isinstance(capture_result, dict) else {}
            record["capture"] = capture_dict
            record["capture_ok"] = bool(capture_dict.get("ok"))
            record["capture_path"] = capture_dict.get("path")
            record["resource_id"] = capture_dict.get("resource_id")
            if not record["capture_ok"]:
                record["capture_error"] = capture_dict.get("error") or "Captura directa Nexus fallida"
            return record
        except Exception as exc:
            record["capture_error"] = str(exc)
            return record

    def _atlas_push_bridge_status(self) -> dict[str, Any]:
        ok, status, payload, error = self._request_json("GET", "/api/trading/quant/vision-bridge/status")
        result = payload if isinstance(payload, dict) else {}
        return {
            **result,
            "ok": bool(result.get("ok", ok)),
            "http_ok": ok,
            "status_code": status,
            "error": error or (result.get("error") if isinstance(result, dict) else None),
            "push_base_url": self._push_base_url(),
        }

    def _atlas_push_bridge_capture(
        self,
        *,
        capture_target: str = "nexus:camera",
        screen_target: dict[str, Any] | None = None,
        calib_path: str | None = None,
        source: str = "camera",
    ) -> dict[str, Any]:
        bridge_status = self._atlas_push_bridge_status()
        effective_calib_path = calib_path or self._push_calib_path() or bridge_status.get("screen_gaze_calibration_path")
        record: dict[str, Any] = {
            "provider": "atlas_push_bridge",
            "bridge_status": bridge_status,
            "capture_backend": "push_bridge",
            "capture_target": str(capture_target or "nexus:camera"),
            "screen_target": screen_target if isinstance(screen_target, dict) else None,
            "calib_path": str(effective_calib_path) if effective_calib_path else None,
            "source": str(source or "camera"),
            "gaze": None,
            "capture": None,
            "capture_ok": False,
            "capture_path": None,
            "resource_id": None,
        }

        if not bridge_status.get("ok"):
            fallback = self._direct_nexus_capture(
                capture_target=capture_target,
                screen_target=screen_target,
                calib_path=str(effective_calib_path) if effective_calib_path else None,
                source=source,
                bridge_status=bridge_status,
                provider_name="atlas_push_bridge",
            )
            if fallback.get("capture_ok"):
                fallback["bridge_warning"] = bridge_status.get("error") or "PUSH bridge no disponible"
            else:
                fallback.setdefault("capture_error", bridge_status.get("error") or "PUSH bridge no disponible")
            return fallback

        if isinstance(screen_target, dict):
            gaze_payload = {
                "x": float(screen_target.get("x", 0.0) or 0.0),
                "y": float(screen_target.get("y", 0.0) or 0.0),
                "source": str(source or "camera"),
            }
            zoom_value = screen_target.get("zoom")
            if zoom_value is not None:
                gaze_payload["zoom"] = float(zoom_value)
            if effective_calib_path:
                gaze_payload["calib_path"] = str(effective_calib_path)
            gaze_ok, gaze_status, gaze_data, gaze_error = self._request_json(
                "POST",
                "/api/primitives/nexus/look-at-screen",
                payload=gaze_payload,
                timeout_sec=self._push_capture_timeout_sec(),
            )
            record["gaze"] = {
                **(gaze_data if isinstance(gaze_data, dict) else {}),
                "http_ok": gaze_ok,
                "status_code": gaze_status,
                "error": gaze_error or (gaze_data.get("error") if isinstance(gaze_data, dict) else None),
            }

        capture_ok, capture_status, capture_data, capture_error = self._request_json(
            "POST",
            "/api/primitives/nexus/grasp",
            payload={"target_id": str(capture_target or "nexus:camera")},
            timeout_sec=self._push_capture_timeout_sec(),
        )
        capture_dict = capture_data if isinstance(capture_data, dict) else {}
        record["capture"] = {
            **capture_dict,
            "http_ok": capture_ok,
            "status_code": capture_status,
            "error": capture_error or capture_dict.get("error"),
        }
        record["capture_ok"] = bool(capture_dict.get("ok"))
        record["capture_path"] = capture_dict.get("path")
        record["resource_id"] = capture_dict.get("resource_id")
        if not record["capture_ok"]:
            record["capture_error"] = capture_error or capture_dict.get("error") or "PUSH capture fallida"
            fallback = self._direct_nexus_capture(
                capture_target=capture_target,
                screen_target=screen_target,
                calib_path=str(effective_calib_path) if effective_calib_path else None,
                source=source,
                bridge_status=bridge_status,
            )
            if fallback.get("capture_ok"):
                fallback["bridge_warning"] = record["capture_error"]
            else:
                fallback["bridge_capture_error"] = record["capture_error"]
            if record.get("gaze") is not None and fallback.get("gaze") is None:
                fallback["gaze"] = record["gaze"]
            return fallback
        return record

    def update(
        self,
        *,
        provider: str | None = None,
        operator_present: bool | None = None,
        screen_integrity_ok: bool | None = None,
        notes: str | None = None,
    ) -> dict[str, Any]:
        state = self._load()
        if provider is not None:
            state["provider"] = str(provider)
        if operator_present is not None:
            state["operator_present"] = bool(operator_present)
        if screen_integrity_ok is not None:
            state["screen_integrity_ok"] = bool(screen_integrity_ok)
        if notes is not None:
            state["notes"] = str(notes)
        return self._save(state)

    def status(self) -> dict[str, Any]:
        state = self._load()
        provider = str(state.get("provider") or "direct_nexus")
        bridge_status = self._atlas_push_bridge_status() if provider == "atlas_push_bridge" else None
        provider_ready = (
            provider in {"off", "manual"}
            or (provider == "desktop_capture" and self._desktop_capture_available())
            or (provider == "direct_nexus" and self._direct_nexus_available())
            or (
                provider == "atlas_push_bridge"
                and (bool((bridge_status or {}).get("ok")) or self._direct_nexus_available())
            )
        )
        return {
            **state,
            "supported_modes": ["off", "manual", "desktop_capture", "direct_nexus", "atlas_push_bridge", "insta360_pending"],
            "desktop_capture_available": self._desktop_capture_available(),
            "insta360_available": False,
            "atlas_push_bridge_status": bridge_status,
            "atlas_push_bridge_available": bool((bridge_status or {}).get("ok")),
            "direct_nexus_fallback_available": self._direct_nexus_available(),
            "provider_ready": provider_ready,
        }

    def diagnose(self) -> dict[str, Any]:
        """Diagnóstico detallado del proveedor de visión — incluye causas de provider_ready=False."""
        state = self._load()
        provider = str(state.get("provider") or "direct_nexus")
        checks: list[dict[str, Any]] = []

        # NEXUS robot check (port 8002)
        nexus_url = self._robot_base_url()
        nexus_ok = self._direct_nexus_available()
        checks.append({
            "name": "nexus_robot",
            "url": f"{nexus_url}/api/health",
            "reachable": nexus_ok,
            "note": "OK" if nexus_ok else (
                f"No responde en {nexus_url}. Inicia atlas_nexus_robot (puerto 8002) "
                "o cambia el proveedor a 'desktop_capture' o 'off'."
            ),
        })

        # desktop capture check
        desktop_ok = self._desktop_capture_available()
        checks.append({
            "name": "desktop_capture",
            "reachable": desktop_ok,
            "note": "Disponible" if desktop_ok else "No disponible en este entorno.",
        })

        # atlas_push_bridge check
        bridge_status = self._atlas_push_bridge_status() if provider == "atlas_push_bridge" else None
        bridge_ok = bool((bridge_status or {}).get("ok"))
        if provider == "atlas_push_bridge":
            checks.append({
                "name": "atlas_push_bridge",
                "url": f"{self._push_base_url()}/api/trading/quant/vision-bridge/status",
                "reachable": bridge_ok,
                "detail": bridge_status,
            })

        provider_ready = (
            provider in {"off", "manual"}
            or (provider == "desktop_capture" and desktop_ok)
            or (provider == "direct_nexus" and nexus_ok)
            or (provider == "atlas_push_bridge" and (bridge_ok or nexus_ok))
        )

        reasons_not_ready: list[str] = []
        if not provider_ready:
            if provider == "direct_nexus" and not nexus_ok:
                reasons_not_ready.append(
                    f"direct_nexus: NEXUS robot no responde en {nexus_url}. "
                    "Soluciones: (1) iniciar atlas_nexus_robot, "
                    "(2) POST /operation/vision/provider con provider='desktop_capture', "
                    "(3) POST /operation/vision/provider con provider='off'."
                )
            if provider == "atlas_push_bridge" and not bridge_ok and not nexus_ok:
                reasons_not_ready.append("atlas_push_bridge: ni el bridge ni el NEXUS robot responden.")
            if provider == "desktop_capture" and not desktop_ok:
                reasons_not_ready.append("desktop_capture: no disponible en este entorno.")

        suggestion: str | None = None
        if not provider_ready and desktop_ok:
            suggestion = "desktop_capture disponible. Llama POST /operation/vision/provider con provider='desktop_capture' para activarla."
        elif not provider_ready:
            suggestion = "Ningún proveedor activo disponible. Usa provider='off' para deshabilitar el requisito de visión."

        return {
            "provider": provider,
            "provider_ready": provider_ready,
            "reasons_not_ready": reasons_not_ready,
            "suggestion": suggestion,
            "checks": checks,
            "nexus_robot_url": nexus_url,
            "desktop_capture_available": desktop_ok,
            "last_capture_at": state.get("last_capture_at"),
        }

    def capture_context_snapshot(self, *, label: str = "operation") -> dict[str, Any]:
        state = self._load()
        timestamp = datetime.utcnow().strftime("%Y%m%dT%H%M%SZ")
        provider = str(state.get("provider") or "direct_nexus")
        record: dict[str, Any] = {
            "captured_at": datetime.utcnow().isoformat(),
            "provider": provider,
            "operator_present": bool(state.get("operator_present")),
            "screen_integrity_ok": bool(state.get("screen_integrity_ok")),
            "notes": str(state.get("notes") or ""),
            "capture_path": None,
            "meta_path": None,
            "capture_ok": False,
        }

        if provider == "desktop_capture":
            try:
                from PIL import ImageGrab  # type: ignore

                image = ImageGrab.grab(all_screens=True)
                image_path = self.snapshots_dir / f"{label}_{timestamp}.png"
                image.save(image_path)
                record["capture_path"] = str(image_path)
                record["capture_ok"] = True
            except Exception as exc:
                record["capture_error"] = str(exc)
        elif provider == "direct_nexus":
            calib_path = self._push_calib_path()
            direct_record = self._direct_nexus_capture(
                capture_target="nexus:camera",
                screen_target=self._default_screen_target(calib_path),
                calib_path=calib_path,
                source="camera",
                provider_name="direct_nexus",
            )
            record.update(direct_record)
            if not record.get("capture_ok"):
                record.setdefault("capture_error", "Captura directa robot no disponible")
        elif provider == "atlas_push_bridge":
            calib_path = self._push_calib_path()
            bridge_record = self._atlas_push_bridge_capture(
                capture_target="nexus:camera",
                screen_target=self._default_screen_target(calib_path),
                calib_path=calib_path,
                source="camera",
            )
            record.update(bridge_record)
            if not record.get("capture_ok"):
                record.setdefault("capture_error", "Captura via bridge no disponible")
        elif provider == "insta360_pending":
            record["capture_error"] = "El proveedor Insta360 aun no esta conectado; falta el puente con el SDK."
        else:
            record["capture_error"] = "Proveedor manual activo; no se intento captura automatica."

        meta_path = self.snapshots_dir / f"{label}_{timestamp}.json"
        meta_path.write_text(json.dumps(record, ensure_ascii=True, indent=2), encoding="utf-8")
        state["last_capture_path"] = record.get("capture_path")
        state["last_capture_at"] = record["captured_at"]
        state["last_capture_note"] = record.get("capture_error") or "Captura de contexto guardada"
        self._save(state)
        record["meta_path"] = str(meta_path)
        return record

    @staticmethod
    def _desktop_capture_available() -> bool:
        try:
            from PIL import ImageGrab  # type: ignore  # noqa: F401

            return True
        except Exception:
            return False
