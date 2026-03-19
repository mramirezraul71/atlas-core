"""Governed vision state for paper-first ATLAS operational tests."""
from __future__ import annotations

import json
from copy import deepcopy
from datetime import datetime
from pathlib import Path
from typing import Any

from config.settings import settings


_DEFAULT_STATE = {
    "provider": "manual",
    "operator_present": True,
    "screen_integrity_ok": True,
    "last_capture_path": None,
    "last_capture_at": None,
    "last_capture_note": "",
    "notes": "Modo de vision manual para pruebas simuladas.",
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
        provider = str(state.get("provider") or "manual")
        return {
            **state,
            "supported_modes": ["off", "manual", "desktop_capture", "insta360_pending"],
            "desktop_capture_available": self._desktop_capture_available(),
            "insta360_available": False,
            "provider_ready": provider in {"off", "manual"} or (provider == "desktop_capture" and self._desktop_capture_available()),
        }

    def capture_context_snapshot(self, *, label: str = "operation") -> dict[str, Any]:
        state = self._load()
        timestamp = datetime.utcnow().strftime("%Y%m%dT%H%M%SZ")
        provider = str(state.get("provider") or "manual")
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
