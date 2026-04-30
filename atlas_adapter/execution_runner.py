from __future__ import annotations

import time
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List

STATE_RECEIVE = "RECEIVE"
STATE_PLAN = "PLAN"
STATE_EXECUTE = "EXECUTE"
STATE_VERIFY = "VERIFY"
STATE_REPORT = "REPORT"


class ExecutionRunner:
    """Tracks execution lifecycle, evidence, and rollback-safe snapshots."""

    def __init__(self, objective: str, atlas_root: Path):
        self.objective = objective
        self.atlas_root = atlas_root
        self.started_at = time.perf_counter()
        self.state = STATE_RECEIVE
        self.timeline: List[Dict[str, Any]] = []
        self.task_contract: Dict[str, Any] = {}
        self.pre_checks: List[Dict[str, Any]] = []
        self.post_checks: List[Dict[str, Any]] = []
        self.tool_results: List[Dict[str, Any]] = []
        self._file_snapshots: Dict[str, Dict[str, Any]] = {}
        self._transition(STATE_RECEIVE, "task_received")

    def _transition(self, new_state: str, reason: str = "") -> None:
        self.state = new_state
        self.timeline.append(
            {
                "ts": datetime.utcnow().isoformat() + "Z",
                "state": new_state,
                "reason": reason,
            }
        )

    def move_to_plan(self, contract: Dict[str, Any]) -> None:
        self.task_contract = contract or {}
        self._transition(STATE_PLAN, "contract_built")

    def move_to_execute(self, pre_checks: List[Dict[str, Any]]) -> None:
        self.pre_checks = pre_checks or []
        self._transition(STATE_EXECUTE, "execution_started")

    def before_tool(self, tool_name: str, tool_input: Dict[str, Any]) -> None:
        if tool_name not in ("write_file", "edit_file"):
            return
        path_raw = (tool_input or {}).get("path")
        if not isinstance(path_raw, str) or not path_raw.strip():
            return
        path = Path(path_raw)
        key = (
            str(path.resolve())
            if path.is_absolute()
            else str((self.atlas_root / path).resolve())
        )
        if key in self._file_snapshots:
            return
        if path.exists():
            try:
                self._file_snapshots[key] = {
                    "path": key,
                    "existed": True,
                    "content": path.read_text(encoding="utf-8", errors="replace"),
                }
            except Exception:
                self._file_snapshots[key] = {
                    "path": key,
                    "existed": True,
                    "content": None,
                }
        else:
            self._file_snapshots[key] = {"path": key, "existed": False, "content": None}

    def record_tool_result(
        self, tool_name: str, ok: bool, ms: int, output_preview: str
    ) -> None:
        self.tool_results.append(
            {
                "tool": tool_name,
                "ok": bool(ok),
                "ms": int(ms),
                "evidence": (output_preview or "")[:180],
            }
        )

    def move_to_verify(self, post_checks: List[Dict[str, Any]]) -> None:
        self.post_checks = post_checks or []
        self._transition(STATE_VERIFY, "post_checks_completed")

    def move_to_report(self, reason: str = "report_generated") -> None:
        self._transition(STATE_REPORT, reason)

    def should_auto_rollback(self, verification_passed: bool) -> bool:
        return (not verification_passed) and bool(self._file_snapshots)

    def rollback(self) -> List[Dict[str, Any]]:
        results: List[Dict[str, Any]] = []
        # Restore in reverse order for safer nested edits.
        for snap in reversed(list(self._file_snapshots.values())):
            p = Path(snap["path"])
            try:
                if snap.get("existed"):
                    if snap.get("content") is None:
                        results.append(
                            {
                                "path": str(p),
                                "ok": False,
                                "action": "restore",
                                "error": "snapshot_missing_content",
                            }
                        )
                    else:
                        p.parent.mkdir(parents=True, exist_ok=True)
                        p.write_text(snap["content"], encoding="utf-8")
                        results.append(
                            {"path": str(p), "ok": True, "action": "restore"}
                        )
                else:
                    if p.exists():
                        p.unlink()
                    results.append({"path": str(p), "ok": True, "action": "delete"})
            except Exception as e:
                results.append(
                    {"path": str(p), "ok": False, "action": "rollback", "error": str(e)}
                )
        return results

    def kpis(self) -> Dict[str, Any]:
        total_ms = int((time.perf_counter() - self.started_at) * 1000)
        ok_tools = sum(1 for t in self.tool_results if t.get("ok"))
        fail_tools = len(self.tool_results) - ok_tools
        return {
            "total_ms": total_ms,
            "total_tools": len(self.tool_results),
            "ok_tools": ok_tools,
            "failed_tools": fail_tools,
            "pre_checks_ok": all(c.get("ok") for c in self.pre_checks)
            if self.pre_checks
            else True,
            "post_checks_ok": all(c.get("ok") for c in self.post_checks)
            if self.post_checks
            else False,
            "states_visited": [t.get("state") for t in self.timeline],
        }
