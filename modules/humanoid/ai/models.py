"""AI layer data models."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional


@dataclass
class Provider:
    """Provider id and capabilities."""
    id: str
    name: str
    is_free: bool
    supports_vision: bool
    available: bool
    models: List[str] = field(default_factory=list)


@dataclass
class ModelSpec:
    """Provider:model spec."""
    provider_id: str
    model_name: str
    full_key: str  # e.g. ollama:llama3.2:3b
    is_free: bool
    route: str  # FAST|CHAT|CODE|REASON|TOOLS|VISION


@dataclass
class RouteDecision:
    """Routing decision: provider + model + reason."""
    provider_id: str
    model_key: str
    route: str
    reason: str
    is_free: bool
    cost_estimate_usd: float = 0.0


@dataclass
class TaskProfile:
    """Inferred profile for routing."""
    intent: str  # chat|code|reason|tools|vision|ops|web|docs
    complexity: str  # low|med|high
    latency_need: str  # fast|normal
    modality: str  # text|image|screen|web
    safety: str  # low|med|high|critical
