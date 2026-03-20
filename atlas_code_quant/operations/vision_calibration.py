"""Servicios de calibracion de mirada para la camara de Quant."""
from __future__ import annotations

import json
import sys
from copy import deepcopy
from datetime import datetime
from pathlib import Path
from typing import Any

from config.settings import settings


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[2]


def _ensure_repo_root_on_path() -> None:
    repo_root = str(_repo_root())
    if repo_root not in sys.path:
        sys.path.insert(0, repo_root)


_ensure_repo_root_on_path()

from modules.nexus_core.primitives import get_default_screen_calib_path, reach_pose
from modules.nexus_core.screen_gaze import ScreenCalib, ScreenCalibSample, save_calib


_DEFAULT_STATE = {
    "active_calibration_path": None,
    "session_active": False,
    "label": "",
    "monitor": {
        "width": 1920,
        "height": 1080,
        "rows": 3,
        "cols": 3,
        "zoom_center": 1.0,
    },
    "points": [],
    "samples": [],
    "last_pose": None,
    "last_move": None,
    "last_fit": None,
    "notes": "",
}


class VisionCalibrationService:
    def __init__(self, state_path: Path | None = None) -> None:
        base_dir = settings.data_dir.parent / "operation"
        base_dir.mkdir(parents=True, exist_ok=True)
        self.state_path = state_path or (base_dir / "vision_calibration_state.json")
        self.default_calibration_path = get_default_screen_calib_path()
        self._ensure_state()

    def _ensure_state(self) -> None:
        if not self.state_path.exists():
            state = deepcopy(_DEFAULT_STATE)
            state["active_calibration_path"] = str(self.default_calibration_path)
            self._save(state)

    def _load(self) -> dict[str, Any]:
        self._ensure_state()
        try:
            payload = json.loads(self.state_path.read_text(encoding="utf-8"))
            if isinstance(payload, dict):
                state = deepcopy(_DEFAULT_STATE)
                state.update(payload)
                state.setdefault("active_calibration_path", str(self.default_calibration_path))
                return state
        except Exception:
            pass
        state = deepcopy(_DEFAULT_STATE)
        state["active_calibration_path"] = str(self.default_calibration_path)
        return state

    def _save(self, payload: dict[str, Any]) -> dict[str, Any]:
        state = deepcopy(_DEFAULT_STATE)
        state.update(payload or {})
        if not state.get("active_calibration_path"):
            state["active_calibration_path"] = str(self.default_calibration_path)
        self.state_path.write_text(json.dumps(state, ensure_ascii=True, indent=2), encoding="utf-8")
        return state

    @staticmethod
    def _grid_positions(size: int, count: int) -> list[float]:
        if count <= 1:
            return [float(size) / 2.0]
        step = float(size) / float(count - 1)
        return [round(step * idx, 3) for idx in range(count)]

    @classmethod
    def _build_points(cls, width: int, height: int, rows: int, cols: int) -> list[dict[str, Any]]:
        xs = cls._grid_positions(width, cols)
        ys = cls._grid_positions(height, rows)
        points: list[dict[str, Any]] = []
        for row_idx, y in enumerate(ys):
            for col_idx, x in enumerate(xs):
                points.append(
                    {
                        "point_id": f"r{row_idx + 1}c{col_idx + 1}",
                        "row": row_idx + 1,
                        "col": col_idx + 1,
                        "x": float(x),
                        "y": float(y),
                        "u": round(float(x) / float(width or 1), 4),
                        "v": round(float(y) / float(height or 1), 4),
                    }
                )
        return points

    @staticmethod
    def _now() -> str:
        return datetime.utcnow().isoformat()

    def _read_current_pose(self) -> dict[str, Any] | None:
        try:
            from modules.humanoid.medulla.bus import get_medulla

            medulla = get_medulla()
            payload = medulla.shared_state.read("actuator.neck.last")
            if isinstance(payload, dict):
                yaw = payload.get("neck_yaw")
                pitch = payload.get("neck_pitch")
                zoom = payload.get("zoom")
                if yaw is not None and pitch is not None:
                    return {
                        "yaw": float(yaw),
                        "pitch": float(pitch),
                        "zoom": float(zoom or 1.0),
                        "source": payload.get("source") or "camera",
                        "ts": payload.get("ts"),
                        "origin": "medulla",
                    }
        except Exception:
            pass

        pose_path = Path(__file__).resolve().parents[2] / "logs" / "nexus_pose.json"
        try:
            payload = json.loads(pose_path.read_text(encoding="utf-8"))
            if isinstance(payload, dict):
                return {
                    "yaw": float(payload.get("pan") or 0.0),
                    "pitch": float(payload.get("tilt") or 0.0),
                    "zoom": float(payload.get("zoom") or 1.0),
                    "source": payload.get("source") or "camera",
                    "ts": payload.get("ts"),
                    "origin": "nexus_pose_log",
                }
        except Exception:
            return None
        return None

    def status(self) -> dict[str, Any]:
        state = self._load()
        active_path = Path(str(state.get("active_calibration_path") or self.default_calibration_path))
        calibration_payload = None
        if active_path.exists():
            try:
                calibration_payload = json.loads(active_path.read_text(encoding="utf-8"))
            except Exception:
                calibration_payload = None
        return {
            "generated_at": self._now(),
            "active_calibration_path": str(active_path),
            "calibration_exists": active_path.exists(),
            "calibration": calibration_payload,
            "session_active": bool(state.get("session_active")),
            "label": str(state.get("label") or ""),
            "monitor": dict(state.get("monitor") or {}),
            "points": list(state.get("points") or []),
            "samples": list(state.get("samples") or []),
            "sample_count": len(list(state.get("samples") or [])),
            "last_pose": state.get("last_pose"),
            "last_move": state.get("last_move"),
            "last_fit": state.get("last_fit"),
            "notes": str(state.get("notes") or ""),
        }

    def start_session(
        self,
        *,
        width: int,
        height: int,
        rows: int = 3,
        cols: int = 3,
        zoom_center: float = 1.0,
        label: str = "",
        save_path: str | None = None,
    ) -> dict[str, Any]:
        width = max(1, int(width))
        height = max(1, int(height))
        rows = max(1, min(int(rows), 9))
        cols = max(1, min(int(cols), 9))
        zoom_center = max(1.0, min(float(zoom_center), 4.0))
        state = self._load()
        state["session_active"] = True
        state["label"] = str(label or "calibracion_principal")
        state["monitor"] = {
            "width": width,
            "height": height,
            "rows": rows,
            "cols": cols,
            "zoom_center": zoom_center,
        }
        state["points"] = self._build_points(width, height, rows, cols)
        state["samples"] = []
        state["last_fit"] = None
        state["notes"] = "Sesion de calibracion iniciada."
        if save_path:
            state["active_calibration_path"] = str(Path(save_path))
        return self.status() if not self._save(state) else self.status()

    def move_pose(self, *, yaw: float, pitch: float, zoom: float = 1.0, source: str = "camera") -> dict[str, Any]:
        result = reach_pose(float(yaw), float(pitch), float(zoom), source=source)
        state = self._load()
        recorded_at = self._now()
        pose = {
            "yaw": float(yaw),
            "pitch": float(pitch),
            "zoom": float(zoom),
            "source": str(source or "camera"),
            "ts": recorded_at,
        }
        state["last_pose"] = pose
        state["last_move"] = {
            "ok": bool(result.get("ok")),
            "pose": pose,
            "result": result,
            "recorded_at": recorded_at,
        }
        self._save(state)
        return self.status()

    def record_sample(
        self,
        *,
        point_id: str | None = None,
        use_current_pose: bool = True,
        yaw: float | None = None,
        pitch: float | None = None,
        zoom: float | None = None,
        note: str = "",
    ) -> dict[str, Any]:
        state = self._load()
        points = list(state.get("points") or [])
        selected = None
        if point_id:
            selected = next((item for item in points if str(item.get("point_id")) == str(point_id)), None)
            if selected is None:
                raise ValueError(f"Punto de calibracion no encontrado: {point_id}")
        elif points:
            sampled_ids = {str(item.get("point_id")) for item in (state.get("samples") or []) if item.get("point_id")}
            selected = next((item for item in points if str(item.get("point_id")) not in sampled_ids), points[0])
        if selected is None:
            raise ValueError("No hay puntos de calibracion activos.")

        pose = self._read_current_pose() if use_current_pose else None
        if yaw is None:
            yaw = pose.get("yaw") if isinstance(pose, dict) else None
        if pitch is None:
            pitch = pose.get("pitch") if isinstance(pose, dict) else None
        if zoom is None:
            zoom = pose.get("zoom") if isinstance(pose, dict) else None
        if yaw is None or pitch is None:
            raise ValueError("No se pudo leer la pose actual; mueve el cuello o envia yaw/pitch manualmente.")

        sample = {
            "point_id": selected.get("point_id"),
            "x": float(selected.get("x") or 0.0),
            "y": float(selected.get("y") or 0.0),
            "yaw": float(yaw),
            "pitch": float(pitch),
            "zoom": float(zoom or state.get("monitor", {}).get("zoom_center") or 1.0),
            "note": str(note or ""),
            "recorded_at": self._now(),
        }
        samples = [item for item in (state.get("samples") or []) if str(item.get("point_id")) != str(sample["point_id"])]
        samples.append(sample)
        samples.sort(key=lambda item: str(item.get("point_id") or ""))
        state["samples"] = samples
        state["last_pose"] = {
            "yaw": float(sample["yaw"]),
            "pitch": float(sample["pitch"]),
            "zoom": float(sample["zoom"]),
            "origin": "manual_sample",
            "ts": sample["recorded_at"],
        }
        state["notes"] = f"Muestra guardada: {sample['point_id']}"
        self._save(state)
        return self.status()

    def fit_and_save(self, *, save_path: str | None = None) -> dict[str, Any]:
        state = self._load()
        monitor = dict(state.get("monitor") or {})
        samples_payload = list(state.get("samples") or [])
        if len(samples_payload) < 2:
            raise ValueError("Se requieren al menos 2 muestras para ajustar la calibracion.")
        samples = [
            ScreenCalibSample(
                x=float(item.get("x") or 0.0),
                y=float(item.get("y") or 0.0),
                yaw=float(item.get("yaw") or 0.0),
                pitch=float(item.get("pitch") or 0.0),
            )
            for item in samples_payload
        ]
        calib = ScreenCalib.from_samples(
            int(monitor.get("width") or 1920),
            int(monitor.get("height") or 1080),
            samples,
            zoom_center=float(monitor.get("zoom_center") or 1.0),
        )
        output_path = Path(str(save_path or state.get("active_calibration_path") or self.default_calibration_path))
        save_calib(calib, output_path)
        state["active_calibration_path"] = str(output_path)
        state["session_active"] = False
        state["last_fit"] = {
            "ok": True,
            "saved_at": self._now(),
            "path": str(output_path),
            "sample_count": len(samples_payload),
            "calibration": calib.to_dict(),
        }
        state["notes"] = "Calibracion ajustada y guardada."
        self._save(state)
        return self.status()

    def reset_session(self, *, clear_active: bool = False) -> dict[str, Any]:
        state = self._load()
        active_path = state.get("active_calibration_path") or str(self.default_calibration_path)
        state = deepcopy(_DEFAULT_STATE)
        state["active_calibration_path"] = None if clear_active else active_path
        state["notes"] = "Sesion de calibracion reiniciada."
        self._save(state)
        return self.status()
