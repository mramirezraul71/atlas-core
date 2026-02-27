"""Humanoid autonomy: planner, goal_tracker."""
from __future__ import annotations

from typing import Any, Dict

from modules.humanoid.kernel import BaseModule, HealthCheckMixin
from .goal_tracker import GoalTracker
from .planner import TaskPlanner


class AutonomyModule(BaseModule, HealthCheckMixin):
    name = "autonomy"

    def __init__(self) -> None:
        self.planner = TaskPlanner()
        self.tracker = GoalTracker()

    def init(self) -> None:
        pass

    def set_brain(self, brain: Any) -> None:
        self.planner.set_brain(brain)

    def health_check(self) -> Dict[str, Any]:
        return {"ok": True, "message": "ok", "details": {}}

    def info(self) -> Dict[str, Any]:
        return {"module": self.name, "goal": self.tracker._goal}


__all__ = ["AutonomyModule", "TaskPlanner", "GoalTracker"]
