"""Multi-objective planning and reprioritization for atlas_agent."""
from __future__ import annotations

import json
from typing import Any, Dict, List

try:
    from .config import AgentConfig
    from .openai_runner import OpenAIPlanner
except Exception:  # pragma: no cover - script-mode fallback
    from config import AgentConfig
    from openai_runner import OpenAIPlanner


def _priority_value(value: str) -> int:
    v = (value or "").strip().lower()
    if v in {"p0", "critical", "high"}:
        return 0
    if v in {"p1", "medium"}:
        return 1
    return 2


class ObjectivePlanner:
    """Builds prioritized objectives and can re-rank them based on observations."""

    def __init__(self, config: AgentConfig, planner: OpenAIPlanner):
        self.config = config
        self.planner = planner

    def build(self, goal: str) -> List[Dict[str, Any]]:
        messages = [
            {
                "role": "system",
                "content": (
                    "Split the user goal into a short prioritized execution list. "
                    "Return strict JSON: {\"objectives\":[{\"id\":\"o1\",\"title\":\"...\",\"priority\":\"high|medium|low\"}]}. "
                    f"Max objectives: {self.config.objective_max_items}."
                ),
            },
            {"role": "user", "content": goal},
        ]
        try:
            reply = self.planner.complete(messages)
            data = self.planner.parse_json(reply.raw_text)
            objs = data.get("objectives") if isinstance(data, dict) else None
            parsed: List[Dict[str, Any]] = []
            if isinstance(objs, list):
                for idx, obj in enumerate(objs[: self.config.objective_max_items], start=1):
                    if not isinstance(obj, dict):
                        continue
                    title = str(obj.get("title") or "").strip()
                    if not title:
                        continue
                    parsed.append(
                        {
                            "id": str(obj.get("id") or f"o{idx}"),
                            "title": title[:220],
                            "priority": str(obj.get("priority") or "medium").lower(),
                            "status": "pending",
                        }
                    )
            if parsed:
                parsed.sort(key=lambda x: _priority_value(x.get("priority", "medium")))
                return parsed
        except Exception:
            pass
        return [
            {
                "id": "o1",
                "title": goal[:220],
                "priority": "high",
                "status": "pending",
            }
        ]

    def reprioritize(
        self,
        objectives: List[Dict[str, Any]],
        latest_observation: Dict[str, Any],
    ) -> List[Dict[str, Any]]:
        pending = [x for x in objectives if x.get("status") != "done"]
        if len(pending) <= 1:
            return objectives

        obs_txt = json.dumps(latest_observation, ensure_ascii=False)[:2500]
        obj_txt = json.dumps(pending, ensure_ascii=False)
        messages = [
            {
                "role": "system",
                "content": (
                    "Reorder pending objectives based on latest observation. "
                    "Return strict JSON: {\"ordered_ids\":[\"o2\",\"o1\",...]} and do not invent ids."
                ),
            },
            {
                "role": "user",
                "content": f"pending={obj_txt}\nobservation={obs_txt}",
            },
        ]
        try:
            reply = self.planner.complete(messages)
            data = self.planner.parse_json(reply.raw_text)
            ordered = data.get("ordered_ids") if isinstance(data, dict) else None
            if isinstance(ordered, list):
                rank = {str(v): i for i, v in enumerate(ordered)}
                pending.sort(key=lambda x: rank.get(str(x.get("id")), 999))
                done = [x for x in objectives if x.get("status") == "done"]
                return done + pending
        except Exception:
            pass
        return objectives
