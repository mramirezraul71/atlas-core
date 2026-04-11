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


def test_warmup_calls_ensure_chart_mission() -> None:
    vision = MagicMock()
    oc = MagicMock()
    oc.chart_execution.ensure_chart_mission.return_value = {
        "open_ok": True,
        "execution_state": "manual_required",
    }
    settings = MagicMock()
    settings.default_vision_provider = ""
    settings.startup_chart_warmup_enabled = True
    settings.chart_auto_open_enabled = True
    settings.startup_chart_warmup_symbols = ["SPY"]
    settings.startup_chart_warmup_timeframe = "1h"
    settings.chart_provider_default = "tradingview"
    out = apply_startup_visual_connections(vision_service=vision, operation_center=oc, settings=settings)
    assert out.get("chart_warmup")
    oc.chart_execution.ensure_chart_mission.assert_called_once()
    call_kw = oc.chart_execution.ensure_chart_mission.call_args.kwargs
    assert call_kw["symbol"] == "SPY"
    assert call_kw["chart_plan"].get("targets")
