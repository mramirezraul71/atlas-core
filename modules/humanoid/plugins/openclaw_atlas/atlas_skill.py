"""Compatibility wrapper for the legacy OpenClaw ATLAS skill path."""

from modules.humanoid.openclaw import (
    AtlasRobotSkill,
    DEFAULT_CONFIG,
    ParsedCommand,
    RobotBackend,
    SensorAdapter,
    load_config,
    register,
)

__all__ = [
    "AtlasRobotSkill",
    "DEFAULT_CONFIG",
    "ParsedCommand",
    "RobotBackend",
    "SensorAdapter",
    "load_config",
    "register",
]
