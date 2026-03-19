from __future__ import annotations

import json
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Optional, Protocol


class NeckPoseController(Protocol):
    """Protocol for neck controllers that can apply a pose."""

    def set_pose(self, *, yaw: float, pitch: float, zoom: float) -> Any:
        """Apply the requested neck pose."""


@dataclass
class ScreenCalibSample:
    """Single manual sample linking screen pixels to neck orientation."""

    x: float
    y: float
    yaw: float
    pitch: float

    def to_dict(self) -> dict[str, float]:
        """Return a JSON-serializable representation."""
        return asdict(self)

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "ScreenCalibSample":
        """Build a sample from a plain dictionary."""
        return cls(
            x=float(data["x"]),
            y=float(data["y"]),
            yaw=float(data["yaw"]),
            pitch=float(data["pitch"]),
        )


def _fit_line(xs: list[float], ys: list[float], axis_name: str) -> tuple[float, float]:
    """Fit y = a*x + b using least squares without external dependencies."""
    if len(xs) != len(ys):
        raise ValueError("Calibration axes must have the same sample count.")
    if len(xs) < 2:
        raise ValueError("At least 2 samples are required to fit a line.")

    mean_x = sum(xs) / len(xs)
    mean_y = sum(ys) / len(ys)
    denom = sum((x - mean_x) ** 2 for x in xs)
    if denom <= 1e-12:
        raise ValueError(
            f"Samples must span the {axis_name} axis to estimate calibration."
        )

    numer = sum((x - mean_x) * (y - mean_y) for x, y in zip(xs, ys))
    slope = numer / denom
    intercept = mean_y - slope * mean_x
    return slope, intercept


@dataclass
class ScreenCalib:
    """Linear screen-to-neck calibration for a specific monitor."""

    W: int
    H: int
    a_y: float
    b_y: float
    a_p: float
    b_p: float
    zoom_center: float = 1.0

    @classmethod
    def from_samples(
        cls,
        W: int,
        H: int,
        samples: list[ScreenCalibSample],
        zoom_center: float = 1.0,
    ) -> "ScreenCalib":
        """Build a calibration from manual screen-to-neck samples."""
        if W <= 0 or H <= 0:
            raise ValueError("Monitor width and height must be positive integers.")
        if len(samples) < 2:
            raise ValueError("Se requieren al menos 2 muestras para calibrar.")

        us = [float(sample.x) / float(W) for sample in samples]
        vs = [float(sample.y) / float(H) for sample in samples]
        yaws = [float(sample.yaw) for sample in samples]
        pitches = [float(sample.pitch) for sample in samples]

        a_y, b_y = _fit_line(us, yaws, "horizontal")
        a_p, b_p = _fit_line(vs, pitches, "vertical")
        return cls(
            W=int(W),
            H=int(H),
            a_y=float(a_y),
            b_y=float(b_y),
            a_p=float(a_p),
            b_p=float(b_p),
            zoom_center=float(zoom_center),
        )

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "ScreenCalib":
        """Build a calibration from JSON-compatible data."""
        return cls(
            W=int(data["W"]),
            H=int(data["H"]),
            a_y=float(data["a_y"]),
            b_y=float(data["b_y"]),
            a_p=float(data["a_p"]),
            b_p=float(data["b_p"]),
            zoom_center=float(data.get("zoom_center", 1.0)),
        )

    def to_dict(self) -> dict[str, float | int]:
        """Return a JSON-serializable representation."""
        return asdict(self)

    def normalize_screen_point(self, x: float, y: float) -> tuple[float, float]:
        """Normalize pixel coordinates into monitor-relative [0, 1] space."""
        if self.W <= 0 or self.H <= 0:
            raise ValueError("Calibration monitor dimensions must be positive.")
        u = min(max(float(x) / float(self.W), 0.0), 1.0)
        v = min(max(float(y) / float(self.H), 0.0), 1.0)
        return u, v

    def screen_to_neck(
        self, x: float, y: float, zoom: Optional[float] = None
    ) -> tuple[float, float, float]:
        """Map screen pixels to neck yaw, pitch and zoom."""
        u, v = self.normalize_screen_point(x, y)
        yaw = (self.a_y * u) + self.b_y
        pitch = (self.a_p * v) + self.b_p
        z = self.zoom_center if zoom is None else float(zoom)
        return float(yaw), float(pitch), float(z)


class ScreenGazeController:
    """High-level controller that maps screen targets to neck poses."""

    def __init__(self, calib: ScreenCalib, neck_controller: NeckPoseController):
        self.calib = calib
        self.neck = neck_controller

    def look_at_screen(
        self, x: float, y: float, zoom: Optional[float] = None
    ) -> dict[str, Any]:
        """Move the neck toward a pixel target on the calibrated monitor."""
        u, v = self.calib.normalize_screen_point(x, y)
        yaw, pitch, z = self.calib.screen_to_neck(x, y, zoom=zoom)
        result = self.neck.set_pose(yaw=yaw, pitch=pitch, zoom=z)

        payload = {
            "ok": True,
            "screen_target": {"x": float(x), "y": float(y), "u": u, "v": v},
            "neck_pose": {
                "neck_yaw": yaw,
                "neck_pitch": pitch,
                "zoom": z,
            },
            "calibration": {
                "W": self.calib.W,
                "H": self.calib.H,
                "zoom_center": self.calib.zoom_center,
            },
        }
        if isinstance(result, dict):
            merged = dict(result)
            merged.setdefault("ok", True)
            merged.setdefault("screen_target", payload["screen_target"])
            merged.setdefault("neck_pose", payload["neck_pose"])
            merged.setdefault("calibration", payload["calibration"])
            return merged
        payload["result"] = result
        return payload


def save_calib(calib: ScreenCalib, path: str | Path) -> Path:
    """Persist a screen calibration as JSON."""
    output_path = Path(path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(
        json.dumps(calib.to_dict(), ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    return output_path


def load_calib(path: str | Path) -> ScreenCalib:
    """Load a screen calibration from JSON."""
    input_path = Path(path)
    data = json.loads(input_path.read_text(encoding="utf-8"))
    return ScreenCalib.from_dict(data)
