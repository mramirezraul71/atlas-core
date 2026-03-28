from __future__ import annotations

import time
from pathlib import Path

from atlas_code_quant.operations.operation_center import OperationCenter
from config.settings import settings


class _SlowTracker:
    def __init__(self) -> None:
        self.calls = 0

    def build_summary(self, **_: object) -> dict:
        self.calls += 1
        time.sleep(2.0)
        return {
            "account_session": {"scope": "paper"},
            "balances": {"total_equity": 100000.0},
            "alerts": [],
            "totals": {"open_positions": 0},
            "strategies": [],
            "pdt_status": {},
            "refresh_interval_sec": 5,
        }


class _Vision:
    def status(self) -> dict:
        return {"provider": "desktop_capture", "provider_ready": True}


class _Executor:
    def __init__(self) -> None:
        self.kill_switch_active = False

    def status(self) -> dict:
        return {"mode": "paper_api", "kill_switch_active": self.kill_switch_active}

    def emergency_stop(self, reason: str = "manual_stop") -> None:
        self.kill_switch_active = True

    def clear_emergency_stop(self) -> None:
        self.kill_switch_active = False


class _Brain:
    def status(self) -> dict:
        return {"last_memory_ok": True, "last_error": ""}


class _SlowBrain(_Brain):
    def __init__(self) -> None:
        self.emit_calls = 0

    def emit(self, **_: object) -> dict:
        self.emit_calls += 1
        time.sleep(2.0)
        return {"ok": True}


class _Learning:
    def status(self, account_scope: str | None = None) -> dict:
        return {"account_scope": account_scope, "policy_ready": True}


class _Journal:
    def snapshot(self, limit: int = 3) -> dict:
        return {"recent_entries_count": 0, "recent_entries": [], "limit": limit}

    def position_management_snapshot(self, account_type: str | None = None, limit: int = 12) -> dict:
        return {
            "enabled": True,
            "account_type": account_type,
            "summary": {"open_positions": 0, "watchlist_count": 0},
            "alerts": [],
            "watchlist": [],
            "limit": limit,
        }

    def exit_governance_snapshot(self, account_type: str | None = None, limit: int = 10) -> dict:
        return {
            "enabled": True,
            "account_type": account_type,
            "summary": {"exit_now_count": 0, "de_risk_count": 0, "take_profit_count": 0},
            "alerts": [],
            "recommendations": [],
            "limit": limit,
        }

    def post_trade_learning_snapshot(self, account_type: str | None = None, limit: int = 10) -> dict:
        return {
            "enabled": True,
            "account_type": account_type,
            "summary": {"closed_trades": 0, "policy_candidate_count": 0},
            "root_cause_breakdown": [],
            "strategy_learning": [],
            "policy_candidates": [],
            "limit": limit,
        }


def _scorecard_payload() -> dict:
    return {
        "available": True,
        "generated_at": "2026-03-28T00:00:00",
        "headline": {
            "atlas_process_compliance_score": 60.0,
            "atlas_process_compliance_status": "watch",
            "atlas_implementation_usefulness_score": 20.0,
            "atlas_implementation_usefulness_status": "critical",
        },
        "metrics": {
            "process_compliance_score": {"value": 60.0, "status": "watch"},
            "implementation_usefulness_score": {"value": 20.0, "status": "critical"},
        },
        "supporting_indicators": {
            "brain_delivery_ratio_pct": 53.43,
            "open_untracked_ratio_pct": 100.0,
        },
        "next_actions": ["seguir auditando"],
    }


def test_operation_status_uses_timeout_fallback_without_blocking(tmp_path: Path):
    original_timeout = settings.tradier_timeout_sec
    original_ttl = settings.tradier_monitor_cache_ttl_sec
    settings.tradier_timeout_sec = 1
    settings.tradier_monitor_cache_ttl_sec = 300
    try:
        center = OperationCenter(
            tracker=_SlowTracker(),
            journal=_Journal(),
            vision=_Vision(),
            executor=_Executor(),
            brain=_Brain(),
            learning=_Learning(),
            state_path=tmp_path / "operation_center_state.json",
            scorecard_provider=_scorecard_payload,
        )

        started = time.perf_counter()
        payload = center.status()
        elapsed = time.perf_counter() - started
    finally:
        settings.tradier_timeout_sec = original_timeout
        settings.tradier_monitor_cache_ttl_sec = original_ttl

    assert elapsed < 1.8
    assert payload["config"]["kill_switch_active"] is False
    assert payload["monitor_summary"]["balances"] == {}
    assert payload["monitor_summary"]["totals"] == {}
    assert payload["monitor_summary"]["alerts"]
    assert "timeout" in payload["monitor_summary"]["alerts"][0]["message"].lower()


def test_operation_status_reuses_inflight_monitor_refresh(tmp_path: Path):
    original_timeout = settings.tradier_timeout_sec
    original_ttl = settings.tradier_monitor_cache_ttl_sec
    settings.tradier_timeout_sec = 1
    settings.tradier_monitor_cache_ttl_sec = 300
    tracker = _SlowTracker()
    try:
        center = OperationCenter(
            tracker=tracker,
            journal=_Journal(),
            vision=_Vision(),
            executor=_Executor(),
            brain=_Brain(),
            learning=_Learning(),
            state_path=tmp_path / "operation_center_state.json",
            scorecard_provider=_scorecard_payload,
        )

        first = center.status()
        started = time.perf_counter()
        second = center.status()
        elapsed = time.perf_counter() - started
        time.sleep(1.2)
    finally:
        settings.tradier_timeout_sec = original_timeout
        settings.tradier_monitor_cache_ttl_sec = original_ttl

    assert tracker.calls == 1
    assert first["monitor_summary"]["alerts"]
    assert second["monitor_summary"]["alerts"] or second["monitor_summary"]["balances"]["total_equity"] == 100000.0
    assert elapsed < 1.2


def test_emergency_stop_does_not_block_on_slow_brain_emit(tmp_path: Path):
    tracker = _JournalingFastTracker()
    brain = _SlowBrain()
    center = OperationCenter(
        tracker=tracker,
        journal=_Journal(),
        vision=_Vision(),
        executor=_Executor(),
        brain=brain,
        learning=_Learning(),
        state_path=tmp_path / "operation_center_state.json",
        scorecard_provider=_scorecard_payload,
    )

    started = time.perf_counter()
    payload = center.emergency_stop(reason="test")
    elapsed = time.perf_counter() - started

    assert elapsed < 0.8
    assert payload["last_decision"]["decision"] == "emergency_stop"


def test_operation_status_exposes_scorecard_snapshot(tmp_path: Path):
    center = OperationCenter(
        tracker=_JournalingFastTracker(),
        journal=_Journal(),
        vision=_Vision(),
        executor=_Executor(),
        brain=_Brain(),
        learning=_Learning(),
        state_path=tmp_path / "operation_center_state.json",
        scorecard_provider=_scorecard_payload,
    )

    payload = center.status()

    assert payload["scorecard"]["available"] is True
    assert payload["scorecard"]["headline"]["atlas_process_compliance_score"] == 60.0
    assert payload["scorecard"]["supporting_indicators"]["brain_delivery_ratio_pct"] == 53.43


def test_operation_status_exposes_position_management_snapshot(tmp_path: Path):
    center = OperationCenter(
        tracker=_JournalingFastTracker(),
        journal=_Journal(),
        vision=_Vision(),
        executor=_Executor(),
        brain=_Brain(),
        learning=_Learning(),
        state_path=tmp_path / "operation_center_state.json",
        scorecard_provider=_scorecard_payload,
    )

    payload = center.status()

    assert payload["position_management"]["enabled"] is True
    assert payload["position_management"]["account_type"] == "paper"
    assert payload["position_management"]["summary"]["open_positions"] == 0


def test_operation_status_exposes_exit_governance_snapshot(tmp_path: Path):
    center = OperationCenter(
        tracker=_JournalingFastTracker(),
        journal=_Journal(),
        vision=_Vision(),
        executor=_Executor(),
        brain=_Brain(),
        learning=_Learning(),
        state_path=tmp_path / "operation_center_state.json",
        scorecard_provider=_scorecard_payload,
    )

    payload = center.status()

    assert payload["exit_governance"]["enabled"] is True
    assert payload["exit_governance"]["account_type"] == "paper"
    assert payload["exit_governance"]["summary"]["exit_now_count"] == 0


def test_operation_status_exposes_post_trade_learning_snapshot(tmp_path: Path):
    center = OperationCenter(
        tracker=_JournalingFastTracker(),
        journal=_Journal(),
        vision=_Vision(),
        executor=_Executor(),
        brain=_Brain(),
        learning=_Learning(),
        state_path=tmp_path / "operation_center_state.json",
        scorecard_provider=_scorecard_payload,
    )

    payload = center.status()

    assert payload["post_trade_learning"]["enabled"] is True
    assert payload["post_trade_learning"]["account_type"] == "paper"
    assert payload["post_trade_learning"]["summary"]["closed_trades"] == 0


class _JournalingFastTracker:
    def build_summary(self, **_: object) -> dict:
        return {
            "account_session": {"scope": "paper"},
            "balances": {"total_equity": 100000.0},
            "alerts": [],
            "totals": {"open_positions": 0},
            "strategies": [],
            "pdt_status": {},
            "refresh_interval_sec": 5,
        }
