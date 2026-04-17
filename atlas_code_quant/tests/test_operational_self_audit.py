from __future__ import annotations

import sys
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest

ROOT = Path(__file__).resolve().parents[2]
QUANT_ROOT = ROOT / "atlas_code_quant"
if str(QUANT_ROOT) not in sys.path:
    sys.path.insert(0, str(QUANT_ROOT))

from atlas_code_quant.operations.operational_self_audit import (
    OperationalSelfAuditContext,
    operational_self_audit_enabled,
    run_operational_self_audit,
)


def _minimal_settings(**kwargs: object) -> SimpleNamespace:
    base = {
        "market_open_max_positions": 3,
        "market_open_config_loaded": True,
        "equity_kelly_live_enabled": False,
        "tradier_default_scope": "paper",
    }
    base.update(kwargs)
    return SimpleNamespace(**base)


def test_self_audit_clean_run_returns_info(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.delenv("QUANT_OPERATIONAL_SELF_AUDIT_ENABLED", raising=False)
    ctx = OperationalSelfAuditContext(
        settings=_minimal_settings(),
        operation_config={},
        market_open_snapshot={"max_positions": 3},
        protocol={"kind": "trading_self_audit_protocol"},
        scope="paper",
    )
    result = run_operational_self_audit(ctx)
    assert result.overall_severity == "INFO"
    assert result.passed is True
    assert result.scope == "paper"
    sev = {c.severity for c in result.checks}
    assert "ERROR" not in sev
    assert "BLOCK" not in sev


def test_self_audit_flags_risk_limit_mismatch(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.delenv("QUANT_OPERATIONAL_SELF_AUDIT_ENABLED", raising=False)
    ctx = OperationalSelfAuditContext(
        settings=_minimal_settings(market_open_max_positions=5),
        operation_config={},
        market_open_snapshot={"max_positions": 2},
        protocol={},
        scope="paper",
    )
    result = run_operational_self_audit(ctx)
    assert any(c.id == "risk.max_positions_mismatch" for c in result.checks)
    assert any(c.severity in ("WARN", "ERROR") for c in result.checks)


def test_self_audit_does_not_block_submit_in_paper(monkeypatch: pytest.MonkeyPatch) -> None:
    from atlas_code_quant.api import main
    from datetime import datetime
    from zoneinfo import ZoneInfo

    gate_ref = main._entry_pass_runtime_gate
    monkeypatch.delenv("QUANT_OPERATIONAL_SELF_AUDIT_ENABLED", raising=False)
    ctx = OperationalSelfAuditContext(
        settings=_minimal_settings(market_open_max_positions=99),
        operation_config={},
        market_open_snapshot={"max_positions": 99},
        protocol={},
        scope="paper",
    )
    bad = run_operational_self_audit(ctx)
    assert bad.overall_severity == "ERROR"
    assert main._entry_pass_runtime_gate is gate_ref
    monkeypatch.setattr(main, "_broker_open_positions_count", lambda **kwargs: 0)
    monkeypatch.setattr(main.settings, "market_open_max_positions", 3, raising=False)
    monkeypatch.setattr(main.settings, "market_open_schedule_open_et", "09:30", raising=False)
    monkeypatch.setattr(main.settings, "market_open_schedule_close_et", "16:00", raising=False)
    out = main._entry_pass_runtime_gate(
        action="submit",
        account_scope="paper",
        account_id=None,
        now_et=datetime(2026, 4, 14, 10, 0, tzinfo=ZoneInfo("America/New_York")),
    )
    assert out["skip_entry_pass"] is False


def test_self_audit_respects_disabled_flag(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv("QUANT_OPERATIONAL_SELF_AUDIT_ENABLED", "false")
    st = _minimal_settings(tradier_default_scope="paper")
    assert operational_self_audit_enabled(st) is False
    ctx = OperationalSelfAuditContext(
        settings=st,
        operation_config={},
        market_open_snapshot={"max_positions": 99},
        protocol={},
        scope="paper",
    )
    result = run_operational_self_audit(ctx)
    assert result.checks == []
    assert result.overall_severity == "INFO"


def test_readiness_includes_self_audit_summary_when_enabled(monkeypatch: pytest.MonkeyPatch) -> None:
    from atlas_code_quant.api import main

    monkeypatch.delenv("QUANT_OPERATIONAL_SELF_AUDIT_ENABLED", raising=False)
    monkeypatch.setattr(main.settings, "tradier_default_scope", "paper", raising=False)
    fake_oc = MagicMock()
    fake_oc.get_config.return_value = {"account_scope": "paper"}
    fake_oc.chart_execution.status.return_value = {}
    fake_oc.market_hours_readiness.return_value = {"market_hours_ok": True, "market_hours_reason": ""}
    fake_vision = MagicMock()
    fake_vision.status_for_gate.return_value = {"provider_ready": True}

    monkeypatch.setattr(main, "_OPERATION_CENTER", fake_oc, raising=False)
    monkeypatch.setattr(main, "_VISION", fake_vision, raising=False)

    payload = main._operation_readiness_payload_fast()
    assert "self_audit_summary" in payload
    summary = payload["self_audit_summary"]
    assert isinstance(summary, dict)
    assert "overall_severity" in summary
    assert "checks_count" in summary
