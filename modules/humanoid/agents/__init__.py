"""Multi-agent system: Executive orchestrates Strategist, Architect, Engineer, Reviewer, Researcher, Ops, Optimizer."""
from __future__ import annotations

from .base import BaseAgent
from .registry import AgentRegistry, get_agent_registry
from .executive import ExecutiveAgent, run_multi_agent_goal

__all__ = [
    "BaseAgent",
    "AgentRegistry",
    "get_agent_registry",
    "ExecutiveAgent",
    "run_multi_agent_goal",
]
