from __future__ import annotations

from pathlib import Path

import modules.humanoid.core.event_handlers as event_handlers
import modules.humanoid.supervisor.autoprogrammer as supervisor_autoprogrammer
from modules.humanoid.supervisor.autoprogrammer import AutoProgrammer
from modules.humanoid.supervisor.models import ChangeProposal, PatchBlock
from modules.humanoid.supervisor.policy_engine import PolicyEngine


def _make_repo_tree(tmp_path: Path) -> Path:
    (tmp_path / "modules" / "humanoid" / "supervisor").mkdir(parents=True, exist_ok=True)
    (tmp_path / "tests" / "humanoid").mkdir(parents=True, exist_ok=True)
    return tmp_path


def test_policy_engine_blocks_protected_path(tmp_path: Path) -> None:
    repo = _make_repo_tree(tmp_path)
    engine = PolicyEngine(repo_root=repo)
    decision = engine.evaluate("config/secret.py", mode="safe", allow_new_file=False)
    assert decision.allowed is False
    assert "protected" in decision.reason


def test_autoprogrammer_rolls_back_on_compile_failure(tmp_path: Path, monkeypatch) -> None:
    repo = _make_repo_tree(tmp_path)
    monkeypatch.setenv("ATLAS_AUTOPROGRAMMER_MEMORY_ENABLED", "false")

    target = repo / "modules" / "humanoid" / "supervisor" / "sample.py"
    target.write_text("def value():\n    return 1\n", encoding="utf-8")

    proposal = ChangeProposal(
        topic="watchdog.logs.error",
        source="test",
        action="apply_patch",
        target_file=str(target),
        patches=[PatchBlock(old_text="return 1", new_text="return (")],
        allow_new_file=False,
    )

    runner = AutoProgrammer(repo_root=repo)
    result = runner.process_proposal(proposal, mode="safe")

    assert result.ok is False
    assert result.rollback_applied is True
    assert target.read_text(encoding="utf-8") == "def value():\n    return 1\n"


def test_autoprogrammer_off_mode_is_recommendation_only(tmp_path: Path, monkeypatch) -> None:
    repo = _make_repo_tree(tmp_path)
    monkeypatch.setenv("ATLAS_AUTOPROGRAMMER_MEMORY_ENABLED", "false")

    target = repo / "modules" / "humanoid" / "supervisor" / "noop.py"
    target.write_text("X = 1\n", encoding="utf-8")

    proposal = ChangeProposal(
        topic="watchdog.change.detected",
        source="test",
        action="apply_patch",
        target_file=str(target),
        patches=[PatchBlock(old_text="X = 1", new_text="X = 2")],
        allow_new_file=False,
    )

    runner = AutoProgrammer(repo_root=repo)
    result = runner.process_proposal(proposal, mode="off")

    assert result.ok is True
    assert result.rollback_applied is False
    assert result.validation_result.get("reason") == "recommendation_only"
    assert target.read_text(encoding="utf-8") == "X = 1\n"


def test_event_handler_deduplicates_supervisor_queue(monkeypatch) -> None:
    event_handlers._EVENT_DEDUP_CACHE.clear()
    calls = {"count": 0}

    def _fake_queue_supervisor_review(*args, **kwargs):
        calls["count"] += 1
        return {"ok": True, "queued": True}

    monkeypatch.setattr(
        supervisor_autoprogrammer,
        "queue_supervisor_review",
        _fake_queue_supervisor_review,
    )
    payload = {"source": "test", "rule": "same_rule", "component": "scheduler"}
    first = event_handlers._queue_supervisor_review(
        "watchdog.logs.error",
        payload,
        recommended_action="inspect",
        source="test",
        details={},
    )
    second = event_handlers._queue_supervisor_review(
        "watchdog.logs.error",
        payload,
        recommended_action="inspect",
        source="test",
        details={},
    )

    assert first["deduped"] is False
    assert second["deduped"] is True
    assert calls["count"] == 1
