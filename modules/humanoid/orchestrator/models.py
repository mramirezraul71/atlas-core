"""Orchestrator models: Task, Step, Plan, Result."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional


@dataclass
class Step:
    """Single step in a plan with definition of done."""
    id: str
    description: str
    definition_of_done: Optional[str] = None
    status: str = "pending"  # pending | approved | running | success | failed
    result: Optional[Dict[str, Any]] = None
    error: Optional[str] = None
    artifacts: List[str] = field(default_factory=list)


@dataclass
class Plan:
    """Plan: goal + steps from decomposer."""
    goal: str
    steps: List[Step]
    task_id: Optional[str] = None
    mode: str = "plan_only"  # plan_only | execute
    raw_steps: List[str] = field(default_factory=list)


@dataclass
class Task:
    """Task with context and execution log."""
    id: str
    goal: str
    plan: Plan
    execution_log: List[Dict[str, Any]] = field(default_factory=list)
    artifacts: List[str] = field(default_factory=list)
    status: str = "planned"  # planned | running | completed | failed


@dataclass
class Result:
    """Result of step execution or full goal run."""
    ok: bool
    data: Optional[Dict[str, Any]] = None
    error: Optional[str] = None
    artifacts: List[str] = field(default_factory=list)
    ms: int = 0
