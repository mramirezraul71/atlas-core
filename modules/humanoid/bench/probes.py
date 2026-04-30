"""Micro-tasks for benchmark: chat, code, reason, tools, vision."""
from __future__ import annotations

PROBES = {
    "chat": "Say hello in one word.",
    "code": "Write a Python function that returns the sum of two numbers. One line only.",
    "reason": "What is 2+2? Answer with one number.",
    "tools": "List the keys: name, age. Output as JSON.",
    "vision": "[skip if no vision model] Describe a red square.",
}


def get_probes(level: str = "quick") -> dict:
    """level: quick (chat,reason) | full (all)."""
    if level == "quick":
        return {k: v for k, v in PROBES.items() if k in ("chat", "reason")}
    return dict(PROBES)
