"""
Multi-AI layer: provider registry, router free-first, telemetry, budgets, fallbacks.

Sistema de IA automático con especialistas:
- FAST: Respuestas rápidas (llama3.2:3b)
- CHAT: Conversación general (llama3.1)
- CODE: Código (deepseek-coder)
- REASON: Razonamiento complejo (deepseek-r1)
- TOOLS: Herramientas (qwen2.5)
- VISION: Imágenes (llama3.2-vision)
- ARCHITECT: Diseño (deepseek-r1)
- OPTIMIZER: Optimización (deepseek-coder)
"""
from __future__ import annotations

from .models import RouteDecision, TaskProfile
from .router import route_and_run
from .auto_router import (
    AutoRouter,
    TaskType,
    ModelConfig,
    RouteResult,
    get_auto_router,
    auto_route,
)
from .brain_state import get_brain_state, set_brain_state

__all__ = [
    # Legacy
    "RouteDecision",
    "TaskProfile",
    "route_and_run",
    # Auto Router
    "AutoRouter",
    "TaskType",
    "ModelConfig",
    "RouteResult",
    "get_auto_router",
    "auto_route",
    # Brain State
    "get_brain_state",
    "set_brain_state",
]
