"""Supervisor package for governed autoprogramming."""
from __future__ import annotations

from .autoprogrammer import AutoProgrammer, get_autoprogrammer, queue_supervisor_review
from .models import (
    ALLOW_DELETE_FILES,
    ALLOW_NEW_FILES,
    AUTO_COMMIT,
    AUTO_PUSH,
    MAX_PATCHES_PER_CYCLE,
    REQUIRE_COMPILE_CHECK,
    REQUIRE_TESTS_IF_PRESENT,
)

__all__ = [
    "AutoProgrammer",
    "get_autoprogrammer",
    "queue_supervisor_review",
    "MAX_PATCHES_PER_CYCLE",
    "AUTO_COMMIT",
    "AUTO_PUSH",
    "ALLOW_NEW_FILES",
    "ALLOW_DELETE_FILES",
    "REQUIRE_COMPILE_CHECK",
    "REQUIRE_TESTS_IF_PRESENT",
]

