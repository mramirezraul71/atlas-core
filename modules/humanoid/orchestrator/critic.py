"""Critic: validations for coherence, JSON, and step results."""
from __future__ import annotations

import json
import re
from typing import Any, Dict, List, Optional, Tuple


def validate_steps_format(steps: List[Any]) -> Tuple[bool, Optional[str]]:
    """Steps should be list of strings (descriptions)."""
    if not isinstance(steps, list):
        return False, "steps must be a list"
    for i, s in enumerate(steps):
        if not isinstance(s, str) or not s.strip():
            return False, f"step {i+1} must be non-empty string"
    return True, None


def validate_json(s: str) -> Tuple[bool, Optional[Any], Optional[str]]:
    """Return (ok, parsed, error)."""
    try:
        parsed = json.loads(s)
        return True, parsed, None
    except json.JSONDecodeError as e:
        return False, None, str(e)


def validate_step_result(result: Dict[str, Any]) -> Tuple[bool, Optional[str]]:
    """Check step execution result has ok and no dangerous output."""
    if not isinstance(result, dict):
        return False, "result must be a dict"
    if "ok" not in result:
        return False, "result must contain 'ok'"
    return True, None


def validate_plan_structure(plan: Dict[str, Any]) -> Tuple[bool, Optional[str]]:
    """Plan should have goal and steps."""
    if not isinstance(plan, dict):
        return False, "plan must be a dict"
    if not plan.get("goal") or not isinstance(plan.get("goal"), str):
        return False, "plan must have non-empty goal"
    steps = plan.get("steps")
    ok, err = validate_steps_format(steps if isinstance(steps, list) else [])
    if not ok:
        return False, err or "invalid steps"
    return True, None


def validate_no_destructive(description: str) -> bool:
    """Heuristic: block obviously destructive step descriptions."""
    lower = description.lower()
    blocked = ("rm -rf", "format c:", "del /f /s", "drop database", "truncate", "delete from")
    return not any(b in lower for b in blocked)
