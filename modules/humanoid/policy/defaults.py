"""Default policy rules for humanoid."""
from __future__ import annotations

from .models import PolicyRule


def get_default_policies() -> list[PolicyRule]:
    """Return default policy rules (hands safe mode, update apply off, etc.)."""
    return [
        PolicyRule(
            id="hands_safe_mode",
            scope="hands",
            condition={"safe_mode": True},
            action="allow",
            meta={"description": "Only allowlisted commands when HANDS_SAFE_MODE=true"},
        ),
        PolicyRule(
            id="update_plan_only",
            scope="update",
            condition={"apply": False},
            action="allow",
            meta={"description": "Update apply disabled by default (plan_only)"},
        ),
    ]
