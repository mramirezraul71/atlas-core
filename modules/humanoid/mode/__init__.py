"""ATLAS_MODE: lite|pro|ultra. Feature gating by resources."""
from __future__ import annotations

from .config import (
    get_atlas_mode,
    get_mode_capabilities,
    is_screen_act_allowed,
    is_record_replay_allowed,
    is_benchmark_allowed,
)

__all__ = [
    "get_atlas_mode",
    "get_mode_capabilities",
    "is_screen_act_allowed",
    "is_record_replay_allowed",
    "is_benchmark_allowed",
]
