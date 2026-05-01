"""Tests F20 — Live readiness checklist (paper-only, no live activation)."""

from __future__ import annotations

import ast
import os
from pathlib import Path

import pytest

from atlas_code_quant.autonomy.live_checklist import (
    LiveChecklistItem,
    LiveChecklistResult,
    build_live_checklist,
)
from atlas_code_quant.autonomy.states import (
    LIVE_FORBIDDEN_STATES,
    AutonomyState,
    IllegalTransition,
    assert_transition,
)
from atlas_code_quant.config.live_readiness import (
    LiveReadinessEnv,
    is_live_mode_requested,
    is_live_mode_safe_to_arm,
    load_live_readiness_env,
)


# ---------------------------------------------------------------------------
# load_live_readiness_env
# ---------------------------------------------------------------------------


@pytest.fixture
def clean_env(monkeypatch: pytest.MonkeyPatch) -> None:
    for k in (
        "ATLAS_ENV",
        "ATLAS_LIVETRADINGENABLED",
        "ATLAS_TRADIERDRYRUN",
        "ATLAS_VISION_REQUIRED_FOR_LIVE",
        "ATLAS_KILLSWITCH_FILE",
        "ATLAS_MAX_DAILY_LOSS_USD",
        "ATLAS_MAX_POSITION_NOTIONAL_USD",
        "ATLAS_MAX_ORDERS_PER_MINUTE",
    ):
        monkeypatch.delenv(k, raising=False)


def test_load_defaults_are_paper_safe(clean_env: None) -> None:
    e = load_live_readiness_env()
    assert e.atlas_env == "paper"
    assert e.live_trading_enabled is False
    assert e.tradier_dry_run is True
    assert e.vision_required_for_live is True
    assert e.max_daily_loss_usd == 500.0
    assert e.max_position_notional_usd == 2_500.0
    assert e.max_orders_per_minute == 30


def test_load_parses_truthy_flags(
    clean_env: None, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setenv("ATLAS_ENV", "LIVE")
    monkeypatch.setenv("ATLAS_LIVETRADINGENABLED", "yes")
    monkeypatch.setenv("ATLAS_TRADIERDRYRUN", "false")
    monkeypatch.setenv("ATLAS_VISION_REQUIRED_FOR_LIVE", "no")
    monkeypatch.setenv("ATLAS_MAX_DAILY_LOSS_USD", "1234.5")
    monkeypatch.setenv("ATLAS_MAX_POSITION_NOTIONAL_USD", "9999")
    monkeypatch.setenv("ATLAS_MAX_ORDERS_PER_MINUTE", "60")
    e = load_live_readiness_env()
    assert e.atlas_env == "live"
    assert e.live_trading_enabled is True
    assert e.tradier_dry_run is False
    assert e.vision_required_for_live is False
    assert e.max_daily_loss_usd == 1234.5
    assert e.max_position_notional_usd == 9999.0
    assert e.max_orders_per_minute == 60


def test_load_corrupt_inputs_fall_back_to_defaults(
    clean_env: None, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setenv("ATLAS_LIVETRADINGENABLED", "perhaps")
    monkeypatch.setenv("ATLAS_MAX_DAILY_LOSS_USD", "abc")
    monkeypatch.setenv("ATLAS_MAX_ORDERS_PER_MINUTE", "x")
    e = load_live_readiness_env()
    assert e.live_trading_enabled is False
    assert e.max_daily_loss_usd == 500.0
    assert e.max_orders_per_minute == 30


# ---------------------------------------------------------------------------
# is_live_mode_requested / is_live_mode_safe_to_arm
# ---------------------------------------------------------------------------


def test_is_live_mode_requested_default_false(clean_env: None) -> None:
    assert is_live_mode_requested() is False


def test_is_live_mode_requested_requires_env_and_flag(
    clean_env: None, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setenv("ATLAS_ENV", "live")
    assert is_live_mode_requested() is False
    monkeypatch.setenv("ATLAS_LIVETRADINGENABLED", "true")
    assert is_live_mode_requested() is True


def test_is_live_mode_safe_to_arm_requires_no_dry_run(
    clean_env: None, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setenv("ATLAS_ENV", "live")
    monkeypatch.setenv("ATLAS_LIVETRADINGENABLED", "true")
    # default tradier_dry_run=True → not safe
    assert is_live_mode_safe_to_arm() is False
    monkeypatch.setenv("ATLAS_TRADIERDRYRUN", "false")
    assert is_live_mode_safe_to_arm() is True


# ---------------------------------------------------------------------------
# build_live_checklist
# ---------------------------------------------------------------------------


def _names(result: LiveChecklistResult) -> dict[str, LiveChecklistItem]:
    return {it.name: it for it in result.items}


def test_checklist_default_paper_overall_not_ok(clean_env: None) -> None:
    res = build_live_checklist()
    assert res.overall_ok is False
    by = _names(res)
    assert by["env_live_requested"].ok is False
    assert by["tradier_not_dry_run"].ok is False
    # invariantes que sí deberían pasar:
    assert by["risk_limits_configured"].ok is True
    assert by["killswitch_path_configured"].ok is True
    assert by["fsm_paper_only_invariant"].ok is True


def test_checklist_blocks_when_killswitch_present(
    clean_env: None, tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    ks = tmp_path / "ks"
    ks.write_text("KILL")
    monkeypatch.setenv("ATLAS_KILLSWITCH_FILE", str(ks))
    res = build_live_checklist()
    by = _names(res)
    assert by["killswitch_clear"].ok is False
    assert res.overall_ok is False


def test_checklist_killswitch_clear_when_file_absent(
    clean_env: None, tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setenv("ATLAS_KILLSWITCH_FILE", str(tmp_path / "absent"))
    res = build_live_checklist()
    by = _names(res)
    assert by["killswitch_clear"].ok is True


def test_checklist_full_green_requires_explicit_env(
    clean_env: None, tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setenv("ATLAS_ENV", "live")
    monkeypatch.setenv("ATLAS_LIVETRADINGENABLED", "true")
    monkeypatch.setenv("ATLAS_TRADIERDRYRUN", "false")
    monkeypatch.setenv("ATLAS_VISION_REQUIRED_FOR_LIVE", "true")
    monkeypatch.setenv("ATLAS_KILLSWITCH_FILE", str(tmp_path / "absent"))
    res = build_live_checklist(camera_available=True)
    assert res.overall_ok is True, res.to_dict()
    for it in res.items:
        assert it.ok is True, (it.name, it.reason)


def test_checklist_vision_required_unknown_camera_fails(
    clean_env: None, tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setenv("ATLAS_ENV", "live")
    monkeypatch.setenv("ATLAS_LIVETRADINGENABLED", "true")
    monkeypatch.setenv("ATLAS_TRADIERDRYRUN", "false")
    monkeypatch.setenv("ATLAS_VISION_REQUIRED_FOR_LIVE", "true")
    monkeypatch.setenv("ATLAS_KILLSWITCH_FILE", str(tmp_path / "absent"))
    res = build_live_checklist()  # camera_available=None
    by = _names(res)
    assert by["vision_available_or_optional"].ok is False
    assert res.overall_ok is False


def test_checklist_vision_optional_passes_without_camera(
    clean_env: None, tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setenv("ATLAS_ENV", "live")
    monkeypatch.setenv("ATLAS_LIVETRADINGENABLED", "true")
    monkeypatch.setenv("ATLAS_TRADIERDRYRUN", "false")
    monkeypatch.setenv("ATLAS_VISION_REQUIRED_FOR_LIVE", "false")
    monkeypatch.setenv("ATLAS_KILLSWITCH_FILE", str(tmp_path / "absent"))
    res = build_live_checklist()  # camera unknown but not required
    by = _names(res)
    assert by["vision_available_or_optional"].ok is True
    assert res.overall_ok is True


def test_checklist_invalid_risk_limits_fail(
    clean_env: None, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setenv("ATLAS_MAX_DAILY_LOSS_USD", "0")
    res = build_live_checklist()
    by = _names(res)
    assert by["risk_limits_configured"].ok is False


def test_checklist_to_dict_serializable(clean_env: None) -> None:
    res = build_live_checklist()
    d = res.to_dict()
    assert isinstance(d, dict)
    assert "items" in d and isinstance(d["items"], list)
    for entry in d["items"]:
        assert {"name", "ok", "reason", "evidence"}.issubset(entry.keys())


# ---------------------------------------------------------------------------
# Invariante FSM: F20 NO desbloquea live
# ---------------------------------------------------------------------------


def test_fsm_live_states_remain_forbidden_in_paper_mode() -> None:
    assert AutonomyState.LIVE_ARMED in LIVE_FORBIDDEN_STATES
    assert AutonomyState.LIVE_EXECUTING in LIVE_FORBIDDEN_STATES


def test_assert_transition_still_rejects_live_in_paper_mode() -> None:
    # Construimos una transición plausible hacia LIVE_ARMED desde
    # PAPER_READY (debe ser rechazada con allow_live=False).
    with pytest.raises(IllegalTransition):
        assert_transition(
            AutonomyState.PAPER_READY,
            AutonomyState.LIVE_ARMED,
            allow_live=False,
        )


def test_full_green_checklist_does_not_unlock_fsm(
    clean_env: None, tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """Aunque la checklist diga overall_ok=True, F17 sigue bloqueando."""
    monkeypatch.setenv("ATLAS_ENV", "live")
    monkeypatch.setenv("ATLAS_LIVETRADINGENABLED", "true")
    monkeypatch.setenv("ATLAS_TRADIERDRYRUN", "false")
    monkeypatch.setenv("ATLAS_VISION_REQUIRED_FOR_LIVE", "false")
    monkeypatch.setenv("ATLAS_KILLSWITCH_FILE", str(tmp_path / "absent"))
    res = build_live_checklist()
    assert res.overall_ok is True
    with pytest.raises(IllegalTransition):
        assert_transition(
            AutonomyState.PAPER_READY,
            AutonomyState.LIVE_ARMED,
            allow_live=False,
        )


# ---------------------------------------------------------------------------
# AST guards: F20 no introduce imports prohibidos
# ---------------------------------------------------------------------------


_F20_FILES = [
    "atlas_code_quant/config/live_readiness.py",
    "atlas_code_quant/autonomy/live_checklist.py",
]

_FORBIDDEN_IMPORTS = {
    "tradier_execution",
    "broker_router",
    "tradier_controls",
    "tradier_pdt_ledger",
    "auton_executor",
    "live_authorization",
    "live_loop",
    "live_switch",
    "operation_center",
    "signal_executor",
    "start_paper_trading",
    "production.live_activation",
    "atlas_adapter",
}


def _project_root() -> Path:
    return Path(__file__).resolve().parents[2]


@pytest.mark.parametrize("relpath", _F20_FILES)
def test_f20_files_have_no_forbidden_imports(relpath: str) -> None:
    src = (_project_root() / relpath).read_text("utf-8")
    tree = ast.parse(src)
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            for alias in node.names:
                for tok in _FORBIDDEN_IMPORTS:
                    assert tok not in alias.name, (relpath, alias.name)
        if isinstance(node, ast.ImportFrom):
            mod = node.module or ""
            for tok in _FORBIDDEN_IMPORTS:
                assert tok not in mod, (relpath, mod)


def test_live_checklist_does_not_call_assert_transition_with_allow_live() -> (
    None
):
    """F20 NO debe activar live: ningún allow_live=True hardcoded."""
    src = (
        _project_root() / "atlas_code_quant" / "autonomy" / "live_checklist.py"
    ).read_text("utf-8")
    assert "allow_live=True" not in src
    assert "allow_live = True" not in src
