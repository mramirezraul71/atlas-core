"""
Contrato real de ChartExecutionService: sin targets no hay apertura;
sin auto-open no hay Popen; sin navegador no hay open_ok con auto-open."""
from __future__ import annotations

from unittest.mock import patch

import pytest

from atlas_code_quant.operations import chart_execution as ce
from atlas_code_quant.operations.chart_execution import ChartExecutionService


class _FakeSettings:
    chart_auto_open_enabled = True
    chart_open_cooldown_sec = 90
    chart_verify_after_open = False
    chart_verify_delay_sec = 0.0


class _FakeSettingsOff(_FakeSettings):
    chart_auto_open_enabled = False


def test_ensure_chart_mission_without_targets_is_not_requested() -> None:
    svc = ChartExecutionService()
    with patch.object(ce, "settings", _FakeSettings()):
        out = svc.ensure_chart_mission(chart_plan=None, camera_plan=None, symbol="SPY")
    assert out["execution_state"] == "not_requested"
    assert out["open_attempted"] is False
    assert out["open_mode"] == "not_requested"


def test_ensure_chart_mission_auto_open_disabled_is_manual() -> None:
    svc = ChartExecutionService()
    plan = {
        "targets": [{"url": "https://example.com/chart", "title": "SPY", "timeframe": "5m"}],
    }
    with patch.object(ce, "settings", _FakeSettingsOff()):
        out = svc.ensure_chart_mission(chart_plan=plan, camera_plan=None, symbol="SPY")
    assert out["auto_open_enabled"] is False
    assert out["open_mode"] == "manual_required"
    assert out["manual_required"] is True
    assert out["open_attempted"] is False


def test_ensure_chart_mission_no_browser_with_auto_open() -> None:
    svc = ChartExecutionService()
    plan = {
        "targets": [{"url": "https://example.com/c", "title": "SPY", "timeframe": "5m"}],
    }
    with patch.object(ce, "settings", _FakeSettings()):
        with patch.object(svc, "_detect_browser", return_value=None):
            out = svc.ensure_chart_mission(chart_plan=plan, camera_plan=None, symbol="SPY")
    assert out["open_mode"] == "browser_unavailable"
    assert out["browser_available"] is False
    assert out["open_ok"] is False


def test_ensure_chart_mission_opens_when_enabled_and_browser_present() -> None:
    svc = ChartExecutionService()
    plan = {
        "targets": [{"url": "https://example.com/x", "title": "SPY", "timeframe": "5m"}],
    }
    with patch.object(ce, "settings", _FakeSettings()):
        with patch.object(svc, "_detect_browser", return_value=r"C:\Fake\chrome.exe"):
            with patch.object(ce.subprocess, "Popen") as popen:
                out = svc.ensure_chart_mission(chart_plan=plan, camera_plan=None, symbol="SPY")
    popen.assert_called_once()
    assert out["open_attempted"] is True
    assert out["open_ok"] is True
    assert out["open_mode"] == "browser_urls"
