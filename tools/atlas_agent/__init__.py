"""ATLAS autonomous agent package."""

try:
    from .agent import AtlasAutonomousAgent
    from .config import AgentConfig
except Exception:  # pragma: no cover - script-mode fallback
    from agent import AtlasAutonomousAgent
    from config import AgentConfig

__all__ = ["AtlasAutonomousAgent", "AgentConfig"]
