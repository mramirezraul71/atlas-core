"""Core autonomous loop (Clawdbot-style) for ATLAS."""
from __future__ import annotations

import json
import time
from typing import Any, Dict, Iterable, List

try:
    from .config import AgentConfig
    from .episodic_memory import EpisodicMemory
    from .memory import SessionStore
    from .objectives import ObjectivePlanner
    from .openai_runner import OpenAIPlanner
    from .policy import ApprovalPolicyEngine
    from .toolkit import AtlasToolkit
except Exception:  # pragma: no cover - script-mode fallback
    from config import AgentConfig
    from episodic_memory import EpisodicMemory
    from memory import SessionStore
    from objectives import ObjectivePlanner
    from openai_runner import OpenAIPlanner
    from policy import ApprovalPolicyEngine
    from toolkit import AtlasToolkit


SYSTEM_PROMPT = """You are ATLAS_AGENT, an autonomous engineering agent.
You must solve the user's goal using iterative tool use.

Response format rules:
1) Always return a single JSON object.
2) JSON schema:
{
  "thought": "short reasoning",
  "action": {
    "type": "tool_call" | "objective_done" | "final",
    "tool": "tool name when tool_call",
    "args": { "tool arguments object" },
    "objective_id": "required when objective_done",
    "content": "summary for objective_done or final"
  }
}
3) Never include markdown code fences.
4) Never invent tool outputs.

Execution policy:
- Prefer small safe steps.
- Use tools to gather evidence before deciding.
- Do not ask for user approval directly: if a tool is blocked the system will return
  approval_required in TOOL_RESULT and you must adapt your plan.
- Mark objective completion using action.type=objective_done before final when relevant.
- When done, return type=final with concise actionable summary.
"""


def _safe_int_usage(value: Any) -> int:
    return value if isinstance(value, int) else 0


def _pending_objectives(objectives: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    return [obj for obj in objectives if str(obj.get("status")) != "done"]


def _find_objective(
    objectives: List[Dict[str, Any]],
    objective_id: str,
) -> Dict[str, Any] | None:
    for obj in objectives:
        if str(obj.get("id")) == objective_id:
            return obj
    return None


class AtlasAutonomousAgent:
    """Autonomous agent runner with persistent session logs."""

    def __init__(
        self,
        config: AgentConfig,
        *,
        approval_overrides: Dict[str, bool] | None = None,
        approved_tools: Iterable[str] | None = None,
    ):
        self.config = config
        self.toolkit = AtlasToolkit(config)
        self.planner = OpenAIPlanner(config)
        self.store = SessionStore(config)
        self.policy = ApprovalPolicyEngine(
            config,
            approval_overrides=approval_overrides,
            approved_tools=approved_tools,
        )
        self.objective_planner = ObjectivePlanner(config, self.planner)
        self.episodic_memory = EpisodicMemory(config)

    def _tool_instructions(self) -> str:
        tools = self.toolkit.describe_tools()
        return json.dumps({"tools": tools}, ensure_ascii=False, indent=2)

    def _state_payload(
        self,
        *,
        goal: str,
        objectives: List[Dict[str, Any]],
        memory_hits: List[Dict[str, Any]],
    ) -> str:
        current = (_pending_objectives(objectives) or [None])[0]
        payload = {
            "goal": goal,
            "mode": self.config.mode,
            "current_objective": current,
            "objectives": objectives,
            "recent_memory": [
                {
                    "id": m.get("id"),
                    "goal": m.get("goal"),
                    "summary": m.get("summary"),
                    "success": m.get("success"),
                    "score": m.get("score"),
                }
                for m in memory_hits
            ],
        }
        return json.dumps(payload, ensure_ascii=False)

    def _persist_episode(
        self,
        *,
        goal: str,
        summary: str,
        success: bool,
        objectives: List[Dict[str, Any]],
    ) -> Dict[str, Any]:
        done = len([x for x in objectives if x.get("status") == "done"])
        total = len(objectives)
        tags = ["atlas_agent", self.config.mode, f"done:{done}/{total}"]
        try:
            return self.episodic_memory.add_episode(
                goal=goal,
                summary=(summary or "(sin resumen)")[:4000],
                success=success,
                tags=tags,
            )
        except Exception as exc:
            return {"ok": False, "error": str(exc)}

    def run(self, goal: str) -> Dict[str, Any]:
        if not (goal or "").strip():
            raise ValueError("goal is required")

        start_ts = time.time()
        objectives = self.objective_planner.build(goal.strip())
        memory_hits: List[Dict[str, Any]] = []
        try:
            memory_hits = self.episodic_memory.similar(
                goal.strip(),
                top_k=max(1, self.config.memory_top_k),
            )
        except Exception:
            memory_hits = []

        messages: List[Dict[str, str]] = [
            {
                "role": "system",
                "content": SYSTEM_PROMPT + "\n\nAvailable tools:\n" + self._tool_instructions(),
            },
            {
                "role": "user",
                "content": "INITIAL_STATE:\n"
                + self._state_payload(
                    goal=goal.strip(),
                    objectives=objectives,
                    memory_hits=memory_hits,
                ),
            },
        ]

        self.store.log(
            step=0,
            kind="goal",
            payload={
                "goal": goal,
                "mode": self.config.mode,
                "objectives": objectives,
                "memory_hits": memory_hits,
            },
        )

        usage_agg: Dict[str, int] = {"prompt_tokens": 0, "completion_tokens": 0, "total_tokens": 0}

        for step in range(1, self.config.max_steps + 1):
            t0 = time.perf_counter()
            state_message = {
                "role": "user",
                "content": "EXECUTION_STATE:\n"
                + self._state_payload(goal=goal, objectives=objectives, memory_hits=memory_hits),
            }
            reply = self.planner.complete(messages + [state_message])
            latency_ms = int((time.perf_counter() - t0) * 1000)
            data = self.planner.parse_json(reply.raw_text)

            usage = reply.usage or {}
            usage_agg["prompt_tokens"] += _safe_int_usage(usage.get("prompt_tokens"))
            usage_agg["completion_tokens"] += _safe_int_usage(usage.get("completion_tokens"))
            usage_agg["total_tokens"] += _safe_int_usage(usage.get("total_tokens"))

            self.store.log(
                step=step,
                kind="llm_reply",
                payload={
                    "model": reply.model,
                    "latency_ms": latency_ms,
                    "raw_text": reply.raw_text,
                    "parsed": data,
                    "usage": usage,
                },
            )

            action = data.get("action") if isinstance(data, dict) else None
            if not isinstance(action, dict):
                messages.append({"role": "assistant", "content": reply.raw_text})
                messages.append(
                    {
                        "role": "user",
                        "content": (
                            "Your last output was not valid JSON with action. "
                            "Reply again with the required schema."
                        ),
                    }
                )
                continue

            action_type = str(action.get("type") or "").strip().lower()

            if action_type == "final":
                content = str(action.get("content") or "").strip()
                memory_write = self._persist_episode(
                    goal=goal,
                    summary=content,
                    success=True,
                    objectives=objectives,
                )
                result = {
                    "ok": True,
                    "status": "completed",
                    "goal": goal,
                    "final_answer": content,
                    "steps_used": step,
                    "session_id": self.store.session_id,
                    "session_dir": str(self.store.session_dir),
                    "objectives": objectives,
                    "usage": usage_agg,
                    "elapsed_ms": int((time.time() - start_ts) * 1000),
                    "episode_memory": memory_write,
                }
                self.store.log(step=step, kind="final", payload={"content": content})
                self.store.finalize(result)
                return result

            if action_type == "objective_done":
                pending_before = _pending_objectives(objectives)
                current = pending_before[0] if pending_before else None
                objective_id = str(action.get("objective_id") or (current or {}).get("id") or "").strip()
                note = str(action.get("content") or "").strip()
                changed = False
                obj = _find_objective(objectives, objective_id) if objective_id else None
                if obj and obj.get("status") != "done":
                    obj["status"] = "done"
                    obj["done_step"] = step
                    if note:
                        obj["note"] = note[:400]
                    changed = True
                elif current and current.get("status") != "done":
                    current["status"] = "done"
                    current["done_step"] = step
                    if note:
                        current["note"] = note[:400]
                    changed = True
                    objective_id = str(current.get("id") or objective_id)

                self.store.log(
                    step=step,
                    kind="objective_done",
                    payload={
                        "objective_id": objective_id,
                        "content": note,
                        "changed": changed,
                        "objectives": objectives,
                    },
                )
                messages.append({"role": "assistant", "content": reply.raw_text})
                messages.append(
                    {
                        "role": "user",
                        "content": "OBJECTIVE_UPDATE:\n"
                        + json.dumps(
                            {
                                "objective_id": objective_id,
                                "changed": changed,
                                "remaining": len(_pending_objectives(objectives)),
                                "objectives": objectives,
                            },
                            ensure_ascii=False,
                        ),
                    }
                )
                continue

            if action_type == "tool_call":
                tool_name = str(action.get("tool") or "").strip()
                args = action.get("args") if isinstance(action.get("args"), dict) else {}
                decision = self.policy.decide(tool_name, args)

                t1 = time.perf_counter()
                if not decision.allowed:
                    if decision.requires_approval:
                        tool_result = {
                            "ok": False,
                            "error": "approval_required",
                            "tool": tool_name,
                            "policy_reason": decision.reason,
                            "mode": self.config.mode,
                        }
                    else:
                        tool_result = {
                            "ok": False,
                            "error": "tool_blocked",
                            "tool": tool_name,
                            "policy_reason": decision.reason,
                        }
                else:
                    tool_result = self.toolkit.dispatch(tool_name, args or {})
                tool_ms = int((time.perf_counter() - t1) * 1000)

                observation = {
                    "tool": tool_name,
                    "args": args,
                    "policy": {
                        "allowed": decision.allowed,
                        "requires_approval": decision.requires_approval,
                        "reason": decision.reason,
                    },
                    "latency_ms": tool_ms,
                    "result": tool_result,
                }
                self.store.log(step=step, kind="tool_result", payload=observation)

                new_objectives = self.objective_planner.reprioritize(objectives, observation)
                if new_objectives != objectives:
                    objectives = new_objectives
                    self.store.log(
                        step=step,
                        kind="objective_reprioritized",
                        payload={"objectives": objectives},
                    )

                messages.append({"role": "assistant", "content": reply.raw_text})
                messages.append(
                    {
                        "role": "user",
                        "content": "TOOL_RESULT:\n"
                        + json.dumps(
                            {
                                "observation": observation,
                                "objectives": objectives,
                            },
                            ensure_ascii=False,
                        ),
                    }
                )
                continue

            messages.append({"role": "assistant", "content": reply.raw_text})
            messages.append(
                {
                    "role": "user",
                    "content": (
                        "Invalid action.type. Use tool_call, objective_done, or final exactly."
                    ),
                }
            )

        pending = _pending_objectives(objectives)
        failure_summary = (
            "Agent reached max steps without final action. "
            f"Pending objectives: {len(pending)}."
        )
        memory_write = self._persist_episode(
            goal=goal,
            summary=failure_summary,
            success=False,
            objectives=objectives,
        )
        result = {
            "ok": False,
            "status": "max_steps_reached",
            "goal": goal,
            "steps_used": self.config.max_steps,
            "session_id": self.store.session_id,
            "session_dir": str(self.store.session_dir),
            "objectives": objectives,
            "usage": usage_agg,
            "elapsed_ms": int((time.time() - start_ts) * 1000),
            "error": "Agent reached max steps without final action",
            "episode_memory": memory_write,
        }
        self.store.finalize(result)
        return result
