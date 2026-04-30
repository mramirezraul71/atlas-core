from __future__ import annotations

import numpy as np

from atlas_code_quant.operations.visual_input_provider import VisualInputProvider


def test_get_camera_frame_fallback_when_capture_missing(monkeypatch) -> None:
    vip = VisualInputProvider()
    monkeypatch.setattr(vip, "_capture", None)
    out = vip.get_camera_frame()
    assert out.ok is False
    assert out.provider_used == "camera"


def test_get_chart_frame_uses_screen_fallback(monkeypatch) -> None:
    vip = VisualInputProvider()
    monkeypatch.setattr(vip, "_chart_launcher", None)
    monkeypatch.setattr(
        vip,
        "get_screen_frame",
        lambda: type("S", (), {
            "ok": True,
            "frame": np.zeros((10, 10, 3), dtype=np.uint8),
            "provider_used": "desktop_capture",
            "used_fallback": False,
            "fallback_reason": "",
            "screenshot_path": "",
            "metadata": {},
        })(),
    )
    out = vip.get_chart_frame(source="tradingview", symbol="SPY", timeframe="5m")
    assert out.ok is True
    assert out.used_fallback is True
