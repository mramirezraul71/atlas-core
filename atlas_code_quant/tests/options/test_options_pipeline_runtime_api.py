from __future__ import annotations

import asyncio

from atlas_code_quant.api import main


def test_options_runtime_status_reports_runtime_wiring(monkeypatch):
    monkeypatch.setattr(main, "_options_runtime_task", None)
    monkeypatch.setattr(main, "_options_pipeline_runtime_task", None)
    monkeypatch.setattr(main, "options_runtime_loop_enabled", lambda: True)
    monkeypatch.setattr(main, "options_pipeline_runtime_enabled", lambda: True)

    payload = asyncio.run(main.options_runtime_status())

    assert payload["ok"] is True
    assert payload["paper_runtime"]["enabled"] is True
    assert payload["paper_runtime"]["running"] is False
    assert payload["multi_asset_pipeline_runtime"]["enabled"] is True
    assert payload["paper_performance_endpoint"] == "/options/paper-performance"
