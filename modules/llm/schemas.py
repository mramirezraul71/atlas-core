from __future__ import annotations
from pydantic import BaseModel, Field
from typing import Optional, Literal, Dict, Any, List

RouteName = Literal["FAST", "CHAT", "CODE", "REASON", "TOOLS"]

class LLMRequest(BaseModel):
    prompt: str = Field(..., min_length=1)
    route: Optional[RouteName] = None
    model: Optional[str] = None
    system: Optional[str] = None
    temperature: Optional[float] = 0.2
    top_p: Optional[float] = 0.9
    max_tokens: Optional[int] = None
    stream: Optional[bool] = False
    tools: Optional[List[Dict[str, Any]]] = None

class LLMResponse(BaseModel):
    ok: bool
    output: str = ""
    route: Optional[RouteName] = None
    model_used: Optional[str] = None
    ms: int = 0
    tokens_est: Optional[int] = None
    error: Optional[str] = None
    meta: Dict[str, Any] = Field(default_factory=dict)
