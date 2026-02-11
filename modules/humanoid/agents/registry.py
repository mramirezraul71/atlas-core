"""Registry of agents. Executive uses this to delegate."""
from __future__ import annotations

from typing import Any, Dict, Optional, Type

from .base import BaseAgent


class AgentRegistry:
    """Register and resolve agents by name."""

    def __init__(self) -> None:
        self._agents: Dict[str, BaseAgent] = {}

    def register(self, agent: BaseAgent) -> None:
        self._agents[agent.name] = agent

    def get(self, name: str) -> Optional[BaseAgent]:
        return self._agents.get(name)

    def all_names(self) -> list:
        return list(self._agents.keys())


_registry: Optional[AgentRegistry] = None


def get_agent_registry() -> AgentRegistry:
    global _registry
    if _registry is None:
        _registry = AgentRegistry()
        from . import executive, strategist, architect, engineer, reviewer, researcher, ops, optimizer
        for mod in (executive, strategist, architect, engineer, reviewer, researcher, ops, optimizer):
            if hasattr(mod, "agent"):
                a = getattr(mod, "agent")
                if isinstance(a, BaseAgent):
                    _registry.register(a)
    return _registry
