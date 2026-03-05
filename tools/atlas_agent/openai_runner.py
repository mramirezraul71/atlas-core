"""OpenAI wrapper for autonomous planning."""
from __future__ import annotations

import json
import os
from dataclasses import dataclass
from typing import Any, Dict, List

try:
    from .config import AgentConfig
except Exception:  # pragma: no cover - script-mode fallback
    from config import AgentConfig


@dataclass
class LlmReply:
    raw_text: str
    usage: Dict[str, Any]
    model: str


class OpenAIPlanner:
    """Small adapter around OpenAI chat completion calls."""

    def __init__(self, config: AgentConfig):
        self.config = config
        self.api_key = (os.getenv("OPENAI_API_KEY") or "").strip()
        self._client = None

    def _ensure_client(self):
        if self._client is not None:
            return self._client
        if not self.api_key:
            raise RuntimeError("OPENAI_API_KEY is missing")
        try:
            from openai import OpenAI
        except Exception as exc:
            raise RuntimeError(
                "openai package is not available. Install with: pip install openai"
            ) from exc

        self._client = OpenAI(api_key=self.api_key, timeout=self.config.request_timeout_sec)
        return self._client

    def complete(self, messages: List[Dict[str, str]]) -> LlmReply:
        client = self._ensure_client()
        resp = client.chat.completions.create(
            model=self.config.model,
            messages=messages,
            temperature=self.config.temperature,
            max_tokens=self.config.max_tokens,
            response_format={"type": "json_object"},
        )
        text = ((resp.choices or [None])[0].message.content or "").strip()  # type: ignore[index]
        usage = {
            "prompt_tokens": getattr(resp.usage, "prompt_tokens", None),
            "completion_tokens": getattr(resp.usage, "completion_tokens", None),
            "total_tokens": getattr(resp.usage, "total_tokens", None),
        }
        return LlmReply(raw_text=text, usage=usage, model=resp.model)

    @staticmethod
    def parse_json(text: str) -> Dict[str, Any]:
        payload = (text or "").strip()
        if not payload:
            return {}
        try:
            return json.loads(payload)
        except Exception:
            pass
        # Fallback for responses wrapped in markdown code fences.
        if payload.startswith("```"):
            payload = payload.strip("`")
            parts = payload.split("\n", 1)
            if len(parts) == 2:
                maybe = parts[1]
                if maybe.endswith("```"):
                    maybe = maybe[:-3]
                try:
                    return json.loads(maybe.strip())
                except Exception:
                    return {}
        return {}
