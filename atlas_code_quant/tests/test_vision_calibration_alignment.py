from __future__ import annotations

from pathlib import Path

from atlas_code_quant.operations.vision_calibration import VisionCalibrationService
from modules.nexus_core.screen_gaze import ScreenCalib, save_calib


def test_evaluate_entry_alignment_returns_ready_when_calibration_exists(tmp_path: Path):
    calib_path = tmp_path / "screen_gaze_calibration.json"
    save_calib(
        ScreenCalib(W=1920, H=1080, a_y=2.0, b_y=-1.0, a_p=1.0, b_p=-0.5, zoom_center=1.2),
        calib_path,
    )
    service = VisionCalibrationService(state_path=tmp_path / "vision_state.json")
    service._save(
        {
            "active_calibration_path": str(calib_path),
            "session_active": False,
            "samples": [{"point_id": "r1c1"}, {"point_id": "r2c2"}, {"point_id": "r3c3"}],
            "last_fit": {"ok": True, "sample_count": 3, "path": str(calib_path)},
        }
    )

    result = service.evaluate_entry_alignment(
        entry_price=100.0,
        signal_side="buy",
        ocr_price=100.1,
        confidence=0.91,
        chart_color="bullish",
        pattern_detected="breakout_pullback",
    )

    assert result["ready"] is True
    assert result["score"] > 0.7
    assert result["sample_count"] == 3


def test_evaluate_entry_alignment_is_neutral_without_calibration(tmp_path: Path):
    service = VisionCalibrationService(state_path=tmp_path / "vision_state_empty.json")
    result = service.evaluate_entry_alignment(
        entry_price=100.0,
        signal_side="sell",
        ocr_price=99.9,
        confidence=0.88,
        chart_color="bearish",
        pattern_detected="breakdown",
    )

    assert result["ready"] is False
    assert result["score"] == 0.5
    assert result["reason"] in {"calibration_unavailable", "calibration_partial"}
