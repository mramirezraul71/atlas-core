from __future__ import annotations

import json
import time
from threading import Event
from pathlib import Path

import atlas_code_quant.operations.operation_center as operation_center_module
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
    def __init__(self) -> None:
        self.emit_calls = 0
        self.last_emit: dict | None = None
        self.emit_event = Event()

    def status(self) -> dict:
        return {"last_memory_ok": True, "last_error": ""}

    def emit(self, **payload: object) -> dict:
        self.emit_calls += 1
        self.last_emit = dict(payload)
        self.emit_event.set()
        return {"ok": True}


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

    def attribution_integrity_snapshot(self, account_type: str | None = None, limit: int = 10) -> dict:
        return {
            "enabled": True,
            "account_type": account_type,
            "summary": {
                "entries_total": 0,
                "open_positions": 0,
                "open_untracked_count": 0,
                "recent_flagged_count": 0,
                "attributed_open_positions_pct": 100.0,
                "open_untracked_ratio_pct": 0.0,
            },
            "alerts": [],
            "flagged_entries": [],
            "limit": limit,
        }

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

    def options_governance_adoption_snapshot(self, account_type: str | None = None, limit: int = 10) -> dict:
        return {
            "enabled": True,
            "account_type": account_type,
            "summary": {
                "option_entries_total": 0,
                "open_option_positions": 0,
                "closed_option_trades": 0,
                "time_spread_count": 0,
                "vertical_count": 0,
                "neutral_theta_count": 0,
                "single_leg_count": 0,
                "strategy_diversity_count": 0,
                "time_spread_share_pct": 0.0,
                "vertical_share_pct": 0.0,
            },
            "alerts": [],
            "family_mix": [],
            "strategy_mix": [],
            "limit": limit,
        }


class _CountingJournal(_Journal):
    def __init__(self) -> None:
        self.calls: dict[str, int] = {
            "snapshot": 0,
            "attribution_integrity": 0,
            "position_management": 0,
            "exit_governance": 0,
            "post_trade_learning": 0,
            "options_governance_adoption": 0,
        }

    def snapshot(self, limit: int = 3) -> dict:
        self.calls["snapshot"] += 1
        return super().snapshot(limit=limit)

    def attribution_integrity_snapshot(self, account_type: str | None = None, limit: int = 10) -> dict:
        self.calls["attribution_integrity"] += 1
        return super().attribution_integrity_snapshot(account_type=account_type, limit=limit)

    def position_management_snapshot(self, account_type: str | None = None, limit: int = 12) -> dict:
        self.calls["position_management"] += 1
        return super().position_management_snapshot(account_type=account_type, limit=limit)

    def exit_governance_snapshot(self, account_type: str | None = None, limit: int = 10) -> dict:
        self.calls["exit_governance"] += 1
        return super().exit_governance_snapshot(account_type=account_type, limit=limit)

    def post_trade_learning_snapshot(self, account_type: str | None = None, limit: int = 10) -> dict:
        self.calls["post_trade_learning"] += 1
        return super().post_trade_learning_snapshot(account_type=account_type, limit=limit)

    def options_governance_adoption_snapshot(self, account_type: str | None = None, limit: int = 10) -> dict:
        self.calls["options_governance_adoption"] += 1
        return super().options_governance_adoption_snapshot(account_type=account_type, limit=limit)


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
            "visual_benchmark_feedback_score": {"value": 92.0, "status": "healthy"},
            "options_strategy_governance_feedback_score": {"value": 88.0, "status": "healthy"},
        },
        "supporting_indicators": {
            "brain_delivery_ratio_pct": 53.43,
            "open_untracked_ratio_pct": 100.0,
            "visual_benchmark_source_count": 5,
            "visual_benchmark_translation_pct": 100.0,
            "options_governance_source_count": 3,
            "options_governance_translation_pct": 100.0,
        },
        "next_actions": ["seguir auditando"],
    }


def _slow_scorecard_payload() -> dict:
    time.sleep(2.0)
    return _scorecard_payload()


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


def test_operation_status_scorecard_paths_follow_builder_payload(tmp_path: Path, monkeypatch) -> None:
    expected_report = tmp_path / "reports" / "atlas_quant_implementation_scorecard_latest.md"
    expected_json = tmp_path / "reports" / "atlas_quant_implementation_scorecard.json"

    def _fake_build_scorecard(*, root: Path, **_: object) -> dict:
        assert root == tmp_path
        return {
            "generated_at": "2026-03-30T00:00:00",
            "headline": _scorecard_payload()["headline"],
            "metrics": {
                "process_compliance_score": {"value": 60.0, "status": "watch"},
            },
            "supporting_indicators": _scorecard_payload()["supporting_indicators"],
            "next_actions": ["seguir auditando"],
        }

    monkeypatch.setattr(operation_center_module, "build_trading_implementation_scorecard", _fake_build_scorecard)
    monkeypatch.setattr(
        operation_center_module,
        "write_trading_implementation_scorecard_json",
        lambda payload, path: expected_json,
    )
    monkeypatch.setattr(
        operation_center_module,
        "write_trading_implementation_scorecard_report",
        lambda payload, path: expected_report,
    )

    center = OperationCenter(
        tracker=_JournalingFastTracker(),
        journal=_CountingJournal(),
        vision=_Vision(),
        executor=_Executor(),
        brain=_Brain(),
        learning=_Learning(),
        state_path=tmp_path / "operation_center_state.json",
    )
    center.root_path = tmp_path

    payload = center.status()

    assert payload["scorecard"]["report_path"] == str(expected_report)
    assert payload["scorecard"]["json_path"] == str(expected_json)
    assert payload["scorecard"]["headline"]["atlas_process_compliance_score"] == 60.0


def test_operation_status_lite_skips_heavy_journal_snapshots(tmp_path: Path):
    journal = _CountingJournal()
    center = OperationCenter(
        tracker=_JournalingFastTracker(),
        journal=journal,
        vision=_Vision(),
        executor=_Executor(),
        brain=_Brain(),
        learning=_Learning(),
        state_path=tmp_path / "operation_center_state.json",
        scorecard_provider=_scorecard_payload,
    )

    first = center.status_lite()
    time.sleep(0.1)
    payload = center.status_lite()

    assert payload["config"]["account_scope"] == "paper"
    assert first["monitor_summary"]["balances"] == {}
    assert payload["monitor_summary"]["balances"]["total_equity"] == 100000.0
    assert payload["scorecard"]["headline"]["atlas_process_compliance_score"] == 60.0
    assert payload["selector_session"]["mode"] == "balanced"
    assert all(count == 0 for count in journal.calls.values())


def test_operation_status_lite_returns_fast_with_background_monitor_and_scorecard_refresh(tmp_path: Path):
    original_timeout = settings.tradier_timeout_sec
    original_ttl = settings.tradier_monitor_cache_ttl_sec
    settings.tradier_timeout_sec = 5
    settings.tradier_monitor_cache_ttl_sec = 300
    try:
        center = OperationCenter(
            tracker=_SlowTracker(),
            journal=_CountingJournal(),
            vision=_Vision(),
            executor=_Executor(),
            brain=_Brain(),
            learning=_Learning(),
            state_path=tmp_path / "operation_center_state.json",
            scorecard_provider=_slow_scorecard_payload,
        )

        started = time.perf_counter()
        payload = center.status_lite()
        elapsed = time.perf_counter() - started
    finally:
        settings.tradier_timeout_sec = original_timeout
        settings.tradier_monitor_cache_ttl_sec = original_ttl

    assert elapsed < 1.0
    assert payload["monitor_summary"]["balances"] == {}
    assert payload["scorecard"]["available"] is False
    assert payload["scorecard"]["status"] == "refreshing"


def test_operation_status_reuses_short_lived_status_cache(tmp_path: Path):
    journal = _CountingJournal()
    center = OperationCenter(
        tracker=_JournalingFastTracker(),
        journal=journal,
        vision=_Vision(),
        executor=_Executor(),
        brain=_Brain(),
        learning=_Learning(),
        state_path=tmp_path / "operation_center_state.json",
        scorecard_provider=_scorecard_payload,
    )

    first = center.status()
    second = center.status()

    assert first["generated_at"] == second["generated_at"]
    assert journal.calls["snapshot"] == 1
    assert journal.calls["attribution_integrity"] == 1
    assert journal.calls["position_management"] == 1
    assert journal.calls["exit_governance"] == 1
    assert journal.calls["post_trade_learning"] == 1
    assert journal.calls["options_governance_adoption"] == 1


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


def test_operation_status_exposes_attribution_integrity_snapshot(tmp_path: Path):
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

    assert payload["attribution_integrity"]["enabled"] is True
    assert payload["attribution_integrity"]["account_type"] == "paper"
    assert payload["attribution_integrity"]["summary"]["open_untracked_count"] == 0


class _JournalWithAttributionAlert(_Journal):
    def attribution_integrity_snapshot(self, account_type: str | None = None, limit: int = 10) -> dict:
        return {
            "enabled": True,
            "account_type": account_type,
            "summary": {
                "entries_total": 2,
                "open_positions": 1,
                "open_untracked_count": 1,
                "recent_flagged_count": 1,
                "attributed_open_positions_pct": 0.0,
                "open_untracked_ratio_pct": 100.0,
            },
            "alerts": [{"level": "critical", "code": "recent_entries_flagged"}],
            "flagged_entries": [{"symbol": "AAPL", "strategy_type": "untracked"}],
            "limit": limit,
        }


def test_operation_status_emits_brain_event_when_attribution_integrity_appears(tmp_path: Path):
    brain = _Brain()
    center = OperationCenter(
        tracker=_JournalingFastTracker(),
        journal=_JournalWithAttributionAlert(),
        vision=_Vision(),
        executor=_Executor(),
        brain=brain,
        learning=_Learning(),
        state_path=tmp_path / "operation_center_state.json",
        scorecard_provider=_scorecard_payload,
    )

    payload = center.status()
    assert payload["attribution_integrity"]["summary"]["recent_flagged_count"] == 1
    assert brain.emit_event.wait(1.0)
    assert brain.emit_calls == 1
    assert brain.last_emit is not None
    assert brain.last_emit["kind"] == "attribution_integrity_alert"

    brain.emit_event.clear()
    center.status()
    assert not brain.emit_event.wait(0.2)
    assert brain.emit_calls == 1


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


def test_operation_status_exposes_visual_benchmark_snapshot(tmp_path: Path):
    reports_dir = tmp_path / "reports"
    reports_dir.mkdir(parents=True, exist_ok=True)
    protocol = {
        "visual_entry_benchmark_focus": {
            "current_focus": "visual_entry_optimization",
            "human_best_practice": ["contexto", "nivel", "trigger", "invalidacion"],
            "automation_translation": ["chart_plan", "expected_visual", "ocr", "gate"],
            "recommended_metrics": ["visual_readiness_score_pct", "visual_alignment_score_pct"],
            "web_feedback_loop": ["detectar", "buscar", "comparar", "persistir"],
        }
    }
    (reports_dir / "trading_self_audit_protocol.json").write_text(json.dumps(protocol), encoding="utf-8")
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
    center.root_path = tmp_path

    payload = center.status()

    assert payload["visual_benchmark"]["enabled"] is True
    assert payload["visual_benchmark"]["current_focus"] == "visual_entry_optimization"
    assert payload["visual_benchmark"]["score"] == 92.0
    assert payload["visual_benchmark"]["visual_source_count"] == 5
    assert payload["visual_benchmark"]["translation_pct"] == 100.0


def test_operation_status_exposes_visual_gate_metrics_snapshot(tmp_path: Path):
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
    state = center._load()
    state["visual_gate_stats"] = {
        "evaluated_count": 12,
        "applies_count": 9,
        "blocked_count": 4,
        "passed_count": 5,
        "manual_review_count": 6,
        "last_status": "manual_review",
        "last_blocked": True,
        "last_manual_required": True,
        "last_readiness_score_pct": 68.5,
        "last_alignment_score_pct": 54.0,
        "last_updated_at": "2026-03-29T12:00:00",
        "last_symbol": "AAPL",
        "last_action": "preview",
        "last_blocking_reason": "visual thesis is misaligned with OCR evidence",
    }
    center._save(state)

    payload = center.status()

    assert payload["visual_gate_metrics"]["evaluated_count"] == 12
    assert payload["visual_gate_metrics"]["blocked_count"] == 4
    assert payload["visual_gate_metrics"]["last_status"] == "manual_review"
    assert payload["visual_gate_metrics"]["last_blocked"] is True
    assert payload["visual_gate_metrics"]["last_readiness_score_pct"] == 68.5


def test_operation_status_exposes_market_telemetry_snapshot(tmp_path: Path) -> None:
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

    payload = center.status()

    assert payload["market_telemetry"]["scanner_options_flow_enabled"] in {True, False}
    assert payload["market_telemetry"]["runtime_mode"] in {"hybrid_options_intradia", "proxy_intradia"}
    assert "telemetry" in payload["market_telemetry"]
    assert "active_backend" in payload["market_telemetry"]["telemetry"]


class _VisionWithCapture:
    def status(self) -> dict:
        return {
            "provider": "insta360",
            "provider_ready": True,
            "supported_modes": ["off", "manual", "desktop_capture", "direct_nexus", "atlas_push_bridge", "insta360"],
            "last_capture_at": "2026-03-28T00:00:00",
            "last_capture_note": "Ultima captura guardada correctamente.",
            "notes": "Vision activa en modo Insta360.",
        }


def test_operation_status_exposes_vision_pipeline_snapshot(tmp_path: Path):
    center = OperationCenter(
        tracker=_JournalingFastTracker(),
        journal=_Journal(),
        vision=_VisionWithCapture(),
        executor=_Executor(),
        brain=_Brain(),
        learning=_Learning(),
        state_path=tmp_path / "operation_center_state.json",
        scorecard_provider=_scorecard_payload,
    )
    state = center._load()
    state["visual_gate_stats"] = {
        "evaluated_count": 10,
        "applies_count": 8,
        "blocked_count": 2,
        "passed_count": 6,
        "manual_review_count": 3,
        "last_status": "visual_ready",
        "last_blocked": False,
        "last_manual_required": False,
        "last_readiness_score_pct": 92.0,
        "last_alignment_score_pct": 88.0,
        "last_updated_at": "2026-03-28T00:00:00",
        "last_symbol": "AAPL",
        "last_action": "submit",
        "last_blocking_reason": None,
    }
    center._save(state)

    payload = center.status()

    assert payload["vision_pipeline"]["provider"] == "insta360"
    assert payload["vision_pipeline"]["provider_active"] is True
    assert payload["vision_pipeline"]["provider_ready"] is True
    assert payload["vision_pipeline"]["activation_status"] == "active"
    assert payload["vision_pipeline"]["coverage_status"] == "healthy"
    assert payload["vision_pipeline"]["trade_coverage_pct"] == 80.0
    assert payload["vision_pipeline"]["pass_rate_pct"] == 75.0
    assert payload["vision_pipeline"]["block_rate_pct"] == 25.0
    assert payload["vision_pipeline"]["manual_review_rate_pct"] == 37.5
    assert payload["vision_pipeline"]["last_symbol"] == "AAPL"


def test_operation_status_exposes_brain_council_snapshot(tmp_path: Path, monkeypatch) -> None:
    monkeypatch.setenv("ANTHROPIC_API_KEY", "test-anthropic")
    monkeypatch.setenv("ATLAS_OLLAMA_URL", "http://localhost:11434/api/generate")

    class _BrainCouncilReady(_Brain):
        def status(self) -> dict:
            return {
                "enabled": True,
                "last_memory_ok": True,
                "last_error": "",
                "last_event_kind": "operation_cycle",
                "last_event_at": "2026-03-28T00:00:00",
            }

    center = OperationCenter(
        tracker=_JournalingFastTracker(),
        journal=_Journal(),
        vision=_Vision(),
        executor=_Executor(),
        brain=_BrainCouncilReady(),
        learning=_Learning(),
        state_path=tmp_path / "operation_center_state.json",
        scorecard_provider=_scorecard_payload,
    )

    payload = center.status()

    assert payload["brain_council"]["enabled"] is True
    assert payload["brain_council"]["consensus_mode"] == "2_of_3"
    assert payload["brain_council"]["available_members_count"] == 3
    assert payload["brain_council"]["quorum_ready"] is True
    assert payload["brain_council"]["activation_status"] == "active"
    assert {member["member"] for member in payload["brain_council"]["members"]} == {
        "claude",
        "deepseek_local",
        "atlas_brain",
    }


def test_operation_status_exposes_options_strategy_governance_snapshot(tmp_path: Path):
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
    state = center._load()
    state["last_options_strategy_governance"] = {
        "enabled": True,
        "benchmark_framework": "iv_guides_credit_vs_debit",
        "strategy_type": "call_calendar_spread",
        "structure_family": "term_structure_time_spread",
        "term_structure_slope": 1.18,
        "preferred_but_unavailable_strategy": None,
    }
    center._save(state)

    payload = center.status()

    assert payload["options_strategy_governance"]["enabled"] is True
    assert payload["options_strategy_governance"]["strategy_type"] == "call_calendar_spread"
    assert payload["options_strategy_governance"]["structure_family"] == "term_structure_time_spread"


def test_operation_status_exposes_options_governance_adoption_snapshot(tmp_path: Path):
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

    assert payload["options_governance_adoption"]["enabled"] is True
    assert payload["options_governance_adoption"]["summary"]["option_entries_total"] == 0
    assert payload["options_governance_adoption"]["summary"]["time_spread_count"] == 0


def test_operation_status_exposes_selector_session_settings(tmp_path: Path, monkeypatch):
    monkeypatch.setattr(settings, "selector_session_mode", "options_only")
    monkeypatch.setattr(settings, "selector_options_require_available", True)
    monkeypatch.setattr(settings, "options_enabled", True)
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

    assert payload["selector_session"]["mode"] == "options_only"
    assert payload["selector_session"]["require_optionable_candidate"] is True
    assert payload["selector_session"]["options_enabled"] is True


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
