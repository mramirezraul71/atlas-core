"""Autoprogramación gobernada: policy -> backup -> patch -> validate -> rollback."""
from __future__ import annotations

import json
import logging
import os
import threading
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional

from .models import (
    AUTONOMY_BUILD,
    AUTONOMY_OFF,
    AUTONOMY_SAFE,
    DEFAULT_AUTONOMY_MODE,
    MAX_PATCHES_PER_CYCLE,
    AutoProgrammerResult,
    ChangeProposal,
)
from .patch_executor import PatchExecutor
from .patch_planner import PatchPlanner
from .patch_validator import PatchValidator
from .policy_engine import PolicyEngine
from .rollback_manager import BackupRecord, RollbackManager

_log = logging.getLogger("atlas.supervisor.autoprogrammer")
_LOG_LOCK = threading.Lock()
_AUTOPROGRAMMER: Optional["AutoProgrammer"] = None


def _repo_root() -> Path:
    root = (
        os.getenv("ATLAS_REPO_PATH")
        or os.getenv("ATLAS_PUSH_ROOT")
        or os.getenv("ATLAS_ROOT")
        or ""
    ).strip()
    if root:
        return Path(root).resolve()
    return Path(__file__).resolve().parents[3]


def _attempt_log_path(repo_root: Path) -> Path:
    path = repo_root / "logs" / "hybrid_event_handlers.jsonl"
    path.parent.mkdir(parents=True, exist_ok=True)
    return path


def _append_jsonl(path: Path, record: Dict[str, Any]) -> None:
    line = json.dumps(record, ensure_ascii=True, default=str)
    with _LOG_LOCK:
        with path.open("a", encoding="utf-8") as fh:
            fh.write(line + "\n")


def _memory_enabled() -> bool:
    raw = (os.getenv("ATLAS_AUTOPROGRAMMER_MEMORY_ENABLED") or "true").strip().lower()
    return raw in ("1", "true", "yes", "y", "on")


def _resolve_mode() -> str:
    raw = (os.getenv("ATLAS_AUTOPROGRAMMER_MODE") or "").strip().lower()
    if raw in (AUTONOMY_OFF, AUTONOMY_SAFE, AUTONOMY_BUILD):
        return raw

    try:
        from modules.humanoid.mode.config import get_system_mode

        system_mode = get_system_mode()
    except Exception:
        system_mode = "safe"

    if system_mode == "observe":
        return AUTONOMY_OFF
    if system_mode in ("aggressive", "ultra"):
        return AUTONOMY_BUILD
    return DEFAULT_AUTONOMY_MODE


class AutoProgrammer:
    """Orquesta ejecución segura y trazable de propuestas de cambio."""

    def __init__(self, repo_root: Optional[Path] = None) -> None:
        self.repo_root = (repo_root or _repo_root()).resolve()
        self.policy_engine = PolicyEngine(self.repo_root)
        self.rollback_manager = RollbackManager(self.repo_root)
        self.patch_executor = PatchExecutor(self.repo_root)
        self.patch_validator = PatchValidator(self.repo_root)
        self.patch_planner = PatchPlanner()

    def _record_attempt(
        self,
        *,
        topic: str,
        source: str,
        target_file: str,
        action: str,
        ok: bool,
        validation_result: Dict[str, Any],
        rollback_applied: bool,
        details: Dict[str, Any],
    ) -> None:
        timestamp = datetime.now(timezone.utc).isoformat()
        record = {
            "ts": timestamp,
            "event_type": "supervisor.autoprogramming",
            "source": source,
            "action": action,
            "ok": bool(ok),
            "payload": {
                "topic": topic,
                "target_file": target_file,
            },
            "details": details,
            "timestamp": timestamp,
            "topic": topic,
            "target_file": target_file,
            "validation_result": validation_result,
            "rollback_applied": bool(rollback_applied),
        }
        _append_jsonl(_attempt_log_path(self.repo_root), record)
        self._persist_memory(record)

    def _persist_memory(self, record: Dict[str, Any]) -> None:
        if not _memory_enabled():
            return
        try:
            from modules.humanoid.memory_engine import ensure_thread, memory_write

            thread_id = ensure_thread(None, "AUTOPROGRAMMER")
            memory_write(
                thread_id,
                "summary",
                {
                    "content": (
                        f"autoprogramming topic={record.get('topic')} "
                        f"action={record.get('action')} ok={record.get('ok')}"
                    ),
                    "record": record,
                },
            )
        except Exception as exc:
            _log.debug("memory persistence skipped: %s", exc)

    def process_proposal(
        self,
        proposal: ChangeProposal,
        *,
        mode: Optional[str] = None,
    ) -> AutoProgrammerResult:
        mode_used = (mode or _resolve_mode()).strip().lower()
        target_file = proposal.target_file
        source = proposal.source or "unknown"
        action = proposal.action or "apply_patch"

        policy = self.policy_engine.evaluate(
            target_file,
            mode=mode_used,
            allow_new_file=proposal.allow_new_file,
            operation="patch",
        )

        if mode_used == AUTONOMY_OFF:
            validation = {"ok": True, "mode": mode_used, "reason": "recommendation_only"}
            details = {
                "policy": policy.to_dict(),
                "recommended_only": True,
                "proposal_id": proposal.proposal_id,
            }
            self._record_attempt(
                topic=proposal.topic,
                source=source,
                target_file=target_file,
                action=action,
                ok=True,
                validation_result=validation,
                rollback_applied=False,
                details=details,
            )
            return AutoProgrammerResult(
                ok=True,
                topic=proposal.topic,
                source=source,
                target_file=target_file,
                action=action,
                rollback_applied=False,
                validation_result=validation,
                details=details,
            )

        if not policy.allowed:
            validation = {"ok": False, "mode": mode_used, "reason": "policy_denied"}
            details = {"policy": policy.to_dict(), "proposal_id": proposal.proposal_id}
            self._record_attempt(
                topic=proposal.topic,
                source=source,
                target_file=target_file,
                action=action,
                ok=False,
                validation_result=validation,
                rollback_applied=False,
                details=details,
            )
            return AutoProgrammerResult(
                ok=False,
                topic=proposal.topic,
                source=source,
                target_file=target_file,
                action=action,
                rollback_applied=False,
                validation_result=validation,
                details=details,
            )

        trimmed = proposal
        if len(proposal.patches) > MAX_PATCHES_PER_CYCLE:
            trimmed = ChangeProposal(
                topic=proposal.topic,
                source=proposal.source,
                action=proposal.action,
                target_file=proposal.target_file,
                patches=proposal.patches[:MAX_PATCHES_PER_CYCLE],
                allow_new_file=proposal.allow_new_file,
                metadata={**proposal.metadata, "patches_truncated": True},
                test_paths=proposal.test_paths,
                proposal_id=proposal.proposal_id,
            )

        backup = self.rollback_manager.create_backup(trimmed.target_file)
        if not backup.ok:
            validation = {"ok": False, "mode": mode_used, "reason": "backup_failed"}
            details = {
                "policy": policy.to_dict(),
                "backup": backup.to_dict(),
                "proposal_id": trimmed.proposal_id,
            }
            self._record_attempt(
                topic=trimmed.topic,
                source=source,
                target_file=trimmed.target_file,
                action=action,
                ok=False,
                validation_result=validation,
                rollback_applied=False,
                details=details,
            )
            return AutoProgrammerResult(
                ok=False,
                topic=trimmed.topic,
                source=source,
                target_file=trimmed.target_file,
                action=action,
                rollback_applied=False,
                validation_result=validation,
                details=details,
            )

        exec_result = self.patch_executor.apply_text_patch(trimmed)
        rollback_applied = False
        rollback_details: Dict[str, Any] = {}

        if not exec_result.ok:
            rollback_details = self.rollback_manager.restore(backup)
            rollback_applied = bool(rollback_details.get("ok"))
            validation = {"ok": False, "mode": mode_used, "reason": "patch_failed"}
            details = {
                "policy": policy.to_dict(),
                "backup": backup.to_dict(),
                "patch_execution": exec_result.to_dict(),
                "rollback": rollback_details,
                "proposal_id": trimmed.proposal_id,
            }
            self._record_attempt(
                topic=trimmed.topic,
                source=source,
                target_file=trimmed.target_file,
                action=action,
                ok=False,
                validation_result=validation,
                rollback_applied=rollback_applied,
                details=details,
            )
            return AutoProgrammerResult(
                ok=False,
                topic=trimmed.topic,
                source=source,
                target_file=trimmed.target_file,
                action=action,
                rollback_applied=rollback_applied,
                validation_result=validation,
                details=details,
            )

        validation_obj = self.patch_validator.validate(exec_result.changed_files, proposal=trimmed)
        if not validation_obj.ok:
            rollback_details = self.rollback_manager.restore(backup)
            rollback_applied = bool(rollback_details.get("ok"))

        final_ok = bool(exec_result.ok and validation_obj.ok)
        validation = validation_obj.to_dict()
        details = {
            "policy": policy.to_dict(),
            "backup": backup.to_dict(),
            "patch_execution": exec_result.to_dict(),
            "rollback": rollback_details,
            "mode": mode_used,
            "proposal_id": trimmed.proposal_id,
        }
        self._record_attempt(
            topic=trimmed.topic,
            source=source,
            target_file=trimmed.target_file,
            action=action,
            ok=final_ok,
            validation_result=validation,
            rollback_applied=rollback_applied,
            details=details,
        )
        return AutoProgrammerResult(
            ok=final_ok,
            topic=trimmed.topic,
            source=source,
            target_file=trimmed.target_file,
            action=action,
            rollback_applied=rollback_applied,
            validation_result=validation,
            details=details,
        )

    def run_cycle(
        self,
        proposals: List[ChangeProposal],
        *,
        mode: Optional[str] = None,
    ) -> List[AutoProgrammerResult]:
        mode_used = mode or _resolve_mode()
        selected = proposals[:MAX_PATCHES_PER_CYCLE]
        return [self.process_proposal(p, mode=mode_used) for p in selected]


def get_autoprogrammer() -> AutoProgrammer:
    global _AUTOPROGRAMMER
    if _AUTOPROGRAMMER is None:
        _AUTOPROGRAMMER = AutoProgrammer()
    return _AUTOPROGRAMMER


def queue_supervisor_review(
    topic: str,
    payload: Optional[Dict[str, Any]] = None,
    *,
    recommended_action: str,
    source: str = "event_handlers",
    details: Optional[Dict[str, Any]] = None,
) -> Dict[str, Any]:
    """Recibe evento, genera revisión y ejecuta patch solo si viene explícitamente autofix=true."""
    data = dict(payload or {})
    mode = _resolve_mode()
    ap = get_autoprogrammer()

    review = ap.patch_planner.build_review(topic, data, recommended_action)
    proposal = ap.patch_planner.proposal_from_event(topic, data, recommended_action)
    allow_autofix = bool(data.get("autofix", False))

    if proposal and allow_autofix and mode != AUTONOMY_OFF:
        started = time.perf_counter()
        result = ap.process_proposal(proposal, mode=mode)
        return {
            "ok": result.ok,
            "queued": True,
            "autofix_attempted": True,
            "review": review,
            "mode": mode,
            "result": result.to_dict(),
            "duration_ms": int((time.perf_counter() - started) * 1000),
        }

    target_file = review.get("target_file") or ""
    validation = {
        "ok": True,
        "mode": mode,
        "review_only": True,
        "autofix_attempted": False,
    }
    log_details = {
        "review": review,
        "recommended_action": recommended_action,
        "extra_details": details or {},
        "reason": "queued_for_supervisor_review",
    }
    ap._record_attempt(
        topic=topic,
        source=source or str(data.get("source") or "unknown"),
        target_file=target_file,
        action="queue_supervisor_review",
        ok=True,
        validation_result=validation,
        rollback_applied=False,
        details=log_details,
    )
    return {
        "ok": True,
        "queued": True,
        "autofix_attempted": False,
        "review": review,
        "mode": mode,
        "result": {
            "ok": True,
            "topic": topic,
            "source": source,
            "target_file": target_file,
            "action": "queue_supervisor_review",
            "validation_result": validation,
            "rollback_applied": False,
            "details": log_details,
        },
    }

