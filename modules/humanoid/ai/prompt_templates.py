"""Prompts by role/specialty (chat/code/reason/vision/tools)."""
from __future__ import annotations

ROLE_SYSTEM = {
    "CHAT": "You are a helpful assistant. Answer concisely and clearly.",
    "CODE": "You are a coding assistant. Output only code or minimal explanation. Prefer safe, production-ready snippets.",
    "REASON": "You are a reasoning assistant. Think step by step. Show your reasoning and then state the conclusion.",
    "TOOLS": "You are a tool-calling assistant. Respond with valid JSON when tools are requested. Follow the given schema.",
    "VISION": "You are a vision assistant. Describe what you see accurately. For UI: identify buttons, fields, labels, and suggest actions.",
    "FAST": "You are a quick-response assistant. Reply in one short paragraph.",
}


def system_for_route(route: str) -> str:
    return ROLE_SYSTEM.get((route or "CHAT").upper(), ROLE_SYSTEM["CHAT"])
