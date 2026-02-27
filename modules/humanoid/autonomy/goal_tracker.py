"""Goal tracker: track current goal and progress."""
from __future__ import annotations

from typing import Any, Dict, List, Optional


class GoalTracker:
    """Track a single goal and its steps (done/pending)."""

    def __init__(self) -> None:
        self._goal: Optional[str] = None
        self._steps: List[str] = []
        self._done: List[str] = []

    def set_goal(self, goal: str, steps: Optional[List[str]] = None) -> None:
        self._goal = goal
        self._steps = steps or []
        self._done = []

    def mark_done(self, step_index: int) -> bool:
        if 0 <= step_index < len(self._steps) and step_index >= len(self._done):
            self._done.append(self._steps[step_index])
            return True
        return False

    def status(self) -> Dict[str, Any]:
        return {
            "goal": self._goal,
            "steps": self._steps,
            "done": self._done,
            "pending": self._steps[len(self._done):] if self._steps else [],
            "progress": len(self._done) / len(self._steps) if self._steps else 0.0,
        }

    def clear(self) -> None:
        self._goal = None
        self._steps = []
        self._done = []
