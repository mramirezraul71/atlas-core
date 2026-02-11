"""Request/response schemas for the /llm API (Pydantic models)."""
from __future__ import annotations

from typing import Any, Dict, List, Literal, Optional

from pydantic import BaseModel, Field

RouteName = Literal["FAST", "CHAT", "CODE", "REASON", "TOOLS"]


class LLMRequest(BaseModel):
    """Body for POST /llm. Prompt required; route/model/system/temperature optional."""

    prompt: str = Field(..., min_length=1, description="User prompt to send to the LLM.")
    route: Optional[RouteName] = None
    model: Optional[str] = None
    system: Optional[str] = None
    temperature: Optional[float] = 0.2
    top_p: Optional[float] = 0.9
    max_tokens: Optional[int] = None
    stream: Optional[bool] = False
    tools: Optional[List[Dict[str, Any]]] = None


class LLMResponse(BaseModel):
    """Response from POST /llm: ok, output, route, model_used, ms, tokens_est, error, meta."""

    ok: bool
    output: str = ""
    route: Optional[RouteName] = None
    model_used: Optional[str] = None
    ms: int = 0
    tokens_est: Optional[int] = None
    error: Optional[str] = None
    meta: Dict[str, Any] = Field(default_factory=dict)
