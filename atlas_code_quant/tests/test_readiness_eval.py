from __future__ import annotations

import pytest

from atlas_code_quant.operations.readiness_eval import (
    evaluate_operational_readiness,
    readiness_http_body_ok,
    startup_warmup_gate_satisfied,
)


def test_evaluate_ready_when_chart_manual_and_vision_ready() -> None:
    ok, reasons = evaluate_operational_readiness(
        chart_execution={"browser_available": False},
        vision_provider_ready=True,
        chart_auto_open_enabled=False,
    )
    assert ok is True
    assert reasons == []


def test_evaluate_not_ready_when_auto_open_without_browser() -> None:
    ok, reasons = evaluate_operational_readiness(
        chart_execution={"browser_available": False},
        vision_provider_ready=True,
        chart_auto_open_enabled=True,
    )
    assert ok is False
    assert any("browser" in r.lower() for r in reasons)


def test_evaluate_not_ready_when_vision_not_ready() -> None:
    ok, reasons = evaluate_operational_readiness(
        chart_execution={"browser_available": True},
        vision_provider_ready=False,
        chart_auto_open_enabled=False,
    )
    assert ok is False
    assert any("provider_ready" in r for r in reasons)


@pytest.mark.parametrize(
    "body,expected",
    [
        ({"ok": True, "data": {"ready": True}}, True),
        ({"ok": True, "data": {"ready": False}}, False),
        ({"ok": False, "data": {"ready": True}}, False),
        ({}, False),
        ("not-a-dict", False),
    ],
)
def test_readiness_http_body_ok(body: object, expected: bool) -> None:
    assert readiness_http_body_ok(body) is expected


def test_readiness_http_body_ok_missing_data() -> None:
    assert readiness_http_body_ok({"ok": True}) is False


def test_evaluate_not_ready_when_chart_plan_not_buildable() -> None:
    ok, reasons = evaluate_operational_readiness(
        chart_execution={"browser_available": True},
        vision_provider_ready=True,
        chart_auto_open_enabled=False,
        chart_plan_buildable=False,
    )
    assert ok is False
    assert any("chart_plan" in r.lower() for r in reasons)


def test_evaluate_not_ready_when_visual_pipeline_false() -> None:
    ok, reasons = evaluate_operational_readiness(
        chart_execution={"browser_available": True},
        vision_provider_ready=True,
        chart_auto_open_enabled=False,
        visual_pipeline_ok=False,
    )
    assert ok is False
    assert any("visual_pipeline" in r for r in reasons)


def test_startup_warmup_gate_requires_last_payload() -> None:
    assert startup_warmup_gate_satisfied(
        {"last_payload": {}},
        startup_chart_warmup_enabled=True,
        chart_auto_open_enabled=True,
    ) is False
    assert startup_warmup_gate_satisfied(
        {"last_payload": {"open_ok": True, "execution_state": "opened"}},
        startup_chart_warmup_enabled=True,
        chart_auto_open_enabled=True,
    ) is True
