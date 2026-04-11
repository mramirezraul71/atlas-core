from __future__ import annotations

from unittest.mock import MagicMock

from atlas_code_quant.operations.startup_visual_connect import apply_startup_visual_connections


def test_apply_vision_only_when_configured() -> None:
    vision = MagicMock()
    oc = MagicMock()
    settings = MagicMock()
    settings.default_vision_provider = "desktop_capture"
    settings.startup_chart_warmup_enabled = False
    settings.chart_auto_open_enabled = False
    settings.startup_chart_warmup_symbols = []
    out = apply_startup_visual_connections(vision_service=vision, operation_center=oc, settings=settings)
    vision.update.assert_called_once()
    assert out["vision_provider_applied"] == "desktop_capture"
    assert out["chart_warmup"] is None


def test_warmup_skipped_without_auto_open() -> None:
    vision = MagicMock()
    oc = MagicMock()
    settings = MagicMock()
    settings.default_vision_provider = ""
    settings.startup_chart_warmup_enabled = True
    settings.chart_auto_open_enabled = False
    settings.startup_chart_warmup_symbols = ["SPY"]
    out = apply_startup_visual_connections(vision_service=vision, operation_center=oc, settings=settings)
    assert "chart_warmup_skipped" in out
    oc.chart_execution.ensure_chart_mission.assert_not_called()
