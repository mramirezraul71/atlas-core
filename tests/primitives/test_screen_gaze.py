from __future__ import annotations

from pathlib import Path

from modules.nexus_core.primitives import look_at_screen
from modules.nexus_core.screen_gaze import (
    ScreenCalib,
    ScreenCalibSample,
    ScreenGazeController,
    load_calib,
    save_calib,
)


class DummyNeckController:
    def __init__(self):
        self.calls: list[dict[str, float]] = []

    def set_pose(self, *, yaw: float, pitch: float, zoom: float):
        self.calls.append({"yaw": yaw, "pitch": pitch, "zoom": zoom})
        return {"ok": True, "pose": {"pan": yaw, "tilt": pitch, "zoom": zoom}}


def test_screen_calib_from_samples_builds_linear_mapping():
    samples = [
        ScreenCalibSample(x=0, y=0, yaw=-1.0, pitch=-0.5),
        ScreenCalibSample(x=1920, y=1080, yaw=1.0, pitch=0.5),
        ScreenCalibSample(x=960, y=540, yaw=0.0, pitch=0.0),
    ]

    calib = ScreenCalib.from_samples(1920, 1080, samples, zoom_center=1.3)

    yaw, pitch, zoom = calib.screen_to_neck(1440, 270)

    assert round(calib.a_y, 6) == 2.0
    assert round(calib.b_y, 6) == -1.0
    assert round(calib.a_p, 6) == 1.0
    assert round(calib.b_p, 6) == -0.5
    assert round(yaw, 6) == 0.5
    assert round(pitch, 6) == -0.25
    assert zoom == 1.3


def test_save_and_load_screen_calib_round_trip(tmp_path: Path):
    calib = ScreenCalib(W=3840, H=2160, a_y=1.2, b_y=-0.6, a_p=0.8, b_p=-0.4, zoom_center=1.5)
    path = tmp_path / "screen_gaze_calibration.json"

    save_calib(calib, path)
    loaded = load_calib(path)

    assert loaded == calib


def test_screen_gaze_controller_uses_neck_controller():
    calib = ScreenCalib(W=100, H=100, a_y=2.0, b_y=-1.0, a_p=1.0, b_p=-0.5, zoom_center=1.0)
    neck = DummyNeckController()
    controller = ScreenGazeController(calib=calib, neck_controller=neck)

    result = controller.look_at_screen(75, 25, zoom=2.0)

    assert neck.calls == [{"yaw": 0.5, "pitch": -0.25, "zoom": 2.0}]
    assert result["screen_target"] == {"x": 75.0, "y": 25.0, "u": 0.75, "v": 0.25}
    assert result["neck_pose"] == {"neck_yaw": 0.5, "neck_pitch": -0.25, "zoom": 2.0}
    assert result["pose"] == {"pan": 0.5, "tilt": -0.25, "zoom": 2.0}


def test_look_at_screen_loads_calibration_and_calls_reach_pose(monkeypatch, tmp_path: Path):
    calib = ScreenCalib(W=200, H=100, a_y=2.0, b_y=-1.0, a_p=1.0, b_p=-0.5, zoom_center=1.1)
    calib_path = tmp_path / "screen_gaze_calibration.json"
    save_calib(calib, calib_path)

    calls: list[dict[str, float | str]] = []

    def fake_reach_pose(pan: float, tilt: float, zoom: float = 1.0, *, source: str = "camera"):
        calls.append({"pan": pan, "tilt": tilt, "zoom": zoom, "source": source})
        return {"ok": True, "pose": {"pan": pan, "tilt": tilt, "zoom": zoom}, "source": source}

    monkeypatch.setattr("modules.nexus_core.primitives.reach_pose", fake_reach_pose)

    result = look_at_screen(100, 50, calib_path=calib_path, source="screen")

    assert calls == [{"pan": 0.0, "tilt": 0.0, "zoom": 1.1, "source": "screen"}]
    assert result["ok"] is True
    assert result["calibration_path"] == str(calib_path)
    assert result["screen_target"] == {"x": 100.0, "y": 50.0, "u": 0.5, "v": 0.5}
    assert result["neck_pose"] == {"neck_yaw": 0.0, "neck_pitch": 0.0, "zoom": 1.1}
