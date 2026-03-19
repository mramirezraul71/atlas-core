"""Compatibility package for legacy OpenClaw ATLAS imports."""

from .atlas_skill import (
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
