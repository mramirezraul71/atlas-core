"""Ops: suggests Hands/Scheduler/Update actions. Does not execute; Executive authorizes."""
from __future__ import annotations

from typing import Any, Dict

from .base import BaseAgent


class OpsAgent(BaseAgent):
    name = "ops"
    timeout_sec = 15

    def process(self, context: Dict[str, Any]) -> Dict[str, Any]:
        steps = context.get("steps") or []
        # Classify steps as shell / scheduler / update; return suggested actions only
        suggested = []
        for s in steps[:20]:
            desc = (s if isinstance(s, str) else s.get("description", ""))
            if not desc:
                continue
            lower = desc.lower()
            if "schedule" in lower or "cron" in lower:
                suggested.append({"type": "scheduler", "description": desc[:200]})
            elif "update" in lower or "pip install" in lower:
                suggested.append({"type": "update_check", "description": desc[:200]})
            else:
                suggested.append({"type": "shell", "description": desc[:200]})
        return {"ok": True, "output": {"suggested_actions": suggested}, "error": None, "ms": 0}


agent = OpsAgent()
